#!/usr/bin/env python3
"""
S3 Motion Subsystem
Simulates robot movement, velocity control, and odometry updates.

Receives:  /s2/motion/cmd (S1Command)
Publishes: /s3/motion/state (Telemetry)
"""

import rclpy
from rclpy.node import Node
from s1_interfaces.msg import S1Command, Telemetry
import math
import random


class S3Motion(Node):
    def __init__(self):
        super().__init__("s3_motion")

        # Subscribe to motion commands from master controller
        self.sub_cmd = self.create_subscription(
            S1Command,
            "/s2/motion/cmd",
            self.cb_cmd,
            10
        )

        # Publish motion state back to S1
        self.pub_telemetry = self.create_publisher(
            Telemetry,
            "/s3/motion/state",
            10
        )

        # Publish commands to P3 driver (hardware control)
        self.pub_driver_cmd = self.create_publisher(
            S1Command,
            "/p3_driver/cmd",
            10
        )

        # High-rate motion update timer (10Hz for smooth motion)
        self.timer = self.create_timer(0.1, self.update_motion)

        # Motion parameters
        self.speed = 0.5  # m/s
        self.angular_speed = 0.3  # rad/s
        self.direction = None
        
        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Battery drain simulation
        self.battery = 100.0
        
        # E-STOP latch - persists until CLEAR_ESTOP received
        self.estop_active = False
        
        self.get_logger().info("🏎️ S3 Motion subsystem started")

    def cb_cmd(self, msg: S1Command):
        """Handle incoming motion commands"""
        action = msg.action
        
        # --- SAFETY: E-STOP LATCH LOGIC (HIGHEST PRIORITY) ---
        if action == "estop":
            self.estop_active = True
            self.direction = None  # Force stop
            self.get_logger().error("🔴 HARD E-STOP LATCHED - All motion FROZEN")
            return
        
        elif action == "clear_estop":
            self.estop_active = False
            self.get_logger().info("🟢 E-STOP cleared - Motion commands enabled")
            return
        
        # Block ALL commands if E-STOP is active
        if self.estop_active:
            self.get_logger().warn(f"⚠️ BLOCKED '{action}' - E-STOP is ACTIVE")
            return
        
        # --- NORMAL MOTION COMMANDS (only if E-STOP is clear) ---
        if action == "move_forward":
            self.direction = "forward"
            self.get_logger().info("⬆️ Moving forward")
            # Forward command to P3 driver
            self._send_to_driver(action, msg.params)
        
        elif action == "move_back":
            self.direction = "back"
            self.get_logger().info("⬇️ Moving backward")
            # Forward command to P3 driver
            self._send_to_driver(action, msg.params)
        
        elif action == "turn_left":
            self.direction = "left"
            self.get_logger().info("⬅️ Turning left")
            # Forward command to P3 driver
            self._send_to_driver(action, msg.params)
        
        elif action == "turn_right":
            self.direction = "right"
            self.get_logger().info("➡️ Turning right")
            # Forward command to P3 driver
            self._send_to_driver(action, msg.params)
        
        elif action == "stop":
            self.direction = None
            self.get_logger().info("🛑 Motion stopped")
            # Forward command to P3 driver
            self._send_to_driver(action, msg.params)
        
        elif action == "set_speed":
            params = msg.params
            if len(params) > 0:
                try:
                    self.speed = float(params[0])
                    self.get_logger().info(f"⚙️ Speed set to {self.speed} m/s")
                except ValueError:
                    self.get_logger().warn(f"⚠️ Invalid speed parameter: {params[0]}")
        
        else:
            self.get_logger().warn(f"⚠️ Unknown motion command: {action}")
    
    def _send_to_driver(self, action: str, params: list):
        """Forward command to P3 hardware driver"""
        driver_cmd = S1Command()
        driver_cmd.stamp = self.get_clock().now().to_msg()
        driver_cmd.action = action
        driver_cmd.params = params
        self.pub_driver_cmd.publish(driver_cmd)
        self.get_logger().debug(f"→ P3 driver: {action}")

    def update_motion(self):
        """Update robot position and publish state"""
        dt = 0.1  # Time step (10Hz)
        
        # ENFORCE E-STOP: No motion while E-STOP is active
        if self.estop_active:
            self.direction = None  # Force motors to 0
            # Skip motion updates but still publish frozen state at end
        
        # Update pose based on current direction (skipped if E-STOP active)
        if not self.estop_active:
            if self.direction == "forward":
                self.x += self.speed * math.cos(self.theta) * dt
                self.y += self.speed * math.sin(self.theta) * dt
                self.battery -= 0.01  # Drain faster when moving
            
            elif self.direction == "back":
                self.x -= self.speed * math.cos(self.theta) * dt
                self.y -= self.speed * math.sin(self.theta) * dt
                self.battery -= 0.01
            
            elif self.direction == "left":
                self.theta += self.angular_speed * dt
                self.battery -= 0.005
            
            elif self.direction == "right":
                self.theta -= self.angular_speed * dt
                self.battery -= 0.005
        
        # Keep theta in [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Clamp battery
        self.battery = max(0.0, self.battery)
        
        # Publish telemetry
        msg = Telemetry()
        msg.robot_id = "robot-alpha-s3"
        msg.safety_state = self.direction if self.direction else "idle"
        msg.position_x = self.x
        msg.position_y = self.y
        msg.position_theta = self.theta
        msg.battery_pct = self.battery
        
        # Simulate motor temperature (increases with motion)
        if self.direction:
            msg.temperature_c = 45.0 + random.uniform(0, 10)
        else:
            msg.temperature_c = 35.0 + random.uniform(0, 5)
        
        # P3 interface status (motor controllers via Ethernet)
        msg.p3_status = "UP"
        msg.p3_ethernet = random.uniform(10, 30)
        msg.p3_cpu_percent = random.uniform(20, 50) if self.direction else random.uniform(5, 15)
        msg.p3_mem_percent = random.uniform(30, 50)
        msg.p3_disk_free = random.uniform(70, 80)
        
        # Voltage drops under load
        msg.voltage_v = 24.0 if not self.direction else 23.5 + random.uniform(-0.3, 0.1)
        
        self.pub_telemetry.publish(msg)


def main():
    rclpy.init()
    node = S3Motion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
