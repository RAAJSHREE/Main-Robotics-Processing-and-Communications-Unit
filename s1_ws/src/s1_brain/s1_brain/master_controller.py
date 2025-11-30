import rclpy
from rclpy.node import Node
from s1_interfaces.msg import S1Command, HealthReport, LogEntry, Telemetry

class MasterController(Node):
    def __init__(self):
        super().__init__("s1_master_controller")

        # ROS2 Subscribers
        self.sub_cmd = self.create_subscription(
            S1Command,
            "/s1/cmd",
            self.cb_cmd,
            10
        )

        self.sub_diag = self.create_subscription(
            HealthReport,
            "/s1/diagnostics/report",
            self.cb_diag,
            10
        )
        
        self.sub_telemetry = self.create_subscription(
            Telemetry,
            "/s1/telemetry/raw",
            self.cb_telemetry,
            10
        )

        # ROS2 Publishers (outbound routing to subsystems)
        self.pub_motion = self.create_publisher(S1Command, "/s2/motion/cmd", 10)
        self.pub_perception = self.create_publisher(S1Command, "/s3/perception/cmd", 10)
        self.pub_actuation = self.create_publisher(S1Command, "/s4/actuation/cmd", 10)

        # ROS2 Publishers (direct to hardware drivers for E-STOP)
        self.pub_p2_driver = self.create_publisher(S1Command, "/p2_driver/cmd", 10)
        self.pub_p3_driver = self.create_publisher(S1Command, "/p3_driver/cmd", 10)
        self.pub_p4_driver = self.create_publisher(S1Command, "/p4_driver/cmd", 10)

        # For logging
        self.pub_log = self.create_publisher(LogEntry, "/s1/logs/raw", 10)

        # Internal state
        self.estop_active = False
        self.system_status = "OK"       # OK, WARN, CRITICAL
        self.last_issues = []
        self.telemetry = None
        self.command_count = 0
        
        self.get_logger().info("🧠 S1 Master Controller started")
        self.get_logger().info("   Listening: /s1/cmd, /s1/diagnostics/report, /s1/telemetry/raw")
        self.get_logger().info("   Publishing: /s2/motion/cmd, /s3/perception/cmd, /s4/actuation/cmd")

    def log(self, level, msg):
        log = LogEntry()
        log.stamp = self.get_clock().now().to_msg()
        log.level = level
        log.source_node = "master_controller"
        log.message = msg
        self.pub_log.publish(log)
        self.get_logger().info(f"[MC] {level}: {msg}")

    def cb_telem(self, msg: Telemetry):
        self.telemetry = msg
        # Update estop status from telemetry
        # Only allow telemetry to ACTIVATE estop, not clear it
        # Clearing must come from explicit clear_estop command
        if msg.estop and not self.estop_active:
            self.estop_active = True
            self.log("CRITICAL", "E-STOP ACTIVATED from telemetry")

    def cb_diag(self, msg: HealthReport):
        prev_status = self.system_status
        self.system_status = msg.status
        self.last_issues = list(msg.issues)
        
        # Check for estop in issues
        estop_in_issues = any("EMERGENCY STOP" in issue for issue in msg.issues)
        
        if estop_in_issues and not self.estop_active:
            self.estop_active = True
            self.log("CRITICAL", "E-STOP detected in diagnostics")
        
        # Log status changes
        if prev_status != self.system_status:
            self.get_logger().info(
                f"System status changed: {prev_status} → {self.system_status}"
            )
            if self.system_status == "CRITICAL":
                self.log("CRITICAL", f"System CRITICAL: {', '.join(msg.issues[:3])}")
            elif self.system_status == "WARN":
                self.log("WARN", f"System WARNING: {', '.join(msg.issues[:3])}")

    def cb_cmd(self, cmd: S1Command):
        self.command_count += 1
        
        self.get_logger().info(
            f"📨 Command #{self.command_count}: target={cmd.target}, action={cmd.action}"
        )
        
        # Handle E-STOP activation immediately (bypass all gates)
        if cmd.action == "estop":
            self.estop_active = True
            self.log("CRITICAL", "🛑 E-STOP ACTIVATED BY COMMAND")
            
            # Broadcast E-STOP to ALL subsystems AND drivers
            estop_cmd = S1Command()
            estop_cmd.stamp = self.get_clock().now().to_msg()
            estop_cmd.action = "estop"
            
            # To subsystems (for state management)
            self.pub_motion.publish(estop_cmd)
            self.pub_perception.publish(estop_cmd)
            self.pub_actuation.publish(estop_cmd)
            
            # To drivers (for hardware E-STOP)
            self.pub_p2_driver.publish(estop_cmd)
            self.pub_p3_driver.publish(estop_cmd)
            self.pub_p4_driver.publish(estop_cmd)
            
            self.log("CRITICAL", "⚠️ E-STOP broadcast to ALL subsystems + drivers")
            return
        
        # Safety Gate 1: E-STOP Check
        if self.estop_active:
            if cmd.action == "clear_estop":
                self.estop_active = False
                
                # Broadcast CLEAR_ESTOP to ALL subsystems AND drivers
                clear_cmd = S1Command()
                clear_cmd.stamp = self.get_clock().now().to_msg()
                clear_cmd.action = "clear_estop"
                
                # To subsystems
                self.pub_motion.publish(clear_cmd)
                self.pub_perception.publish(clear_cmd)
                self.pub_actuation.publish(clear_cmd)
                
                # To drivers
                self.pub_p2_driver.publish(clear_cmd)
                self.pub_p3_driver.publish(clear_cmd)
                self.pub_p4_driver.publish(clear_cmd)
                
                self.log("INFO", "✅ E-STOP cleared - broadcast to ALL subsystems + drivers")
                return
            else:
                self.log("CRITICAL", f"❌ BLOCKED '{cmd.action}' - E-STOP is ACTIVE")
                return

        # Safety Gate 2: System Critical Check
        if self.system_status == "CRITICAL":
            # Only allow safe commands during critical state
            safe_commands = ["led_on", "led_off", "clear_estop", "stop", "status"]
            if cmd.action not in safe_commands:
                self.log(
                    "CRITICAL", 
                    f"❌ BLOCKED '{cmd.action}' - System CRITICAL: {self.last_issues[:2]}"
                )
                return

        # Safety Gate 3: Degraded Mode
        if self.system_status == "WARN":
            self.get_logger().warn(f"⚠️  System in WARN state, executing '{cmd.action}' with caution")

        # Route commands to appropriate subsystems
        if cmd.target == "motion":
            self.pub_motion.publish(cmd)
            self.log("INFO", f"✅ Forwarded motion command: {cmd.action}")

        elif cmd.target == "perception":
            self.pub_perception.publish(cmd)
            self.log("INFO", f"✅ Forwarded perception command: {cmd.action}")

        elif cmd.target == "actuation":
            self.pub_actuation.publish(cmd)
            self.log("INFO", f"✅ Forwarded actuation command: {cmd.action}")

        elif cmd.target == "system":
            # Handle system-level commands locally
            if cmd.action == "status":
                status_msg = (
                    f"System Status: {self.system_status} | "
                    f"E-STOP: {'ACTIVE' if self.estop_active else 'CLEAR'} | "
                    f"Issues: {len(self.last_issues)}"
                )
                self.log("INFO", status_msg)
            elif cmd.action == "led_on" or cmd.action == "led_off":
                self.pub_actuation.publish(cmd)
                self.log("INFO", f"✅ LED command: {cmd.action}")
            else:
                self.log("INFO", f"System command: {cmd.action}")

        else:
            self.log("WARN", f"⚠️  Unknown command target: {cmd.target}")

def main(args=None):
    rclpy.init(args=args)
    node = MasterController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Master Controller shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
