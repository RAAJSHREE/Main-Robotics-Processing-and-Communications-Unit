#!/usr/bin/env python3
"""
S2 Perception Subsystem
Simulates pose estimation, obstacle detection, and camera/LiDAR sensor updates.

Receives:  /s3/perception/cmd (S1Command)
Publishes: /s2/perception/state (Telemetry)
"""

import rclpy
from rclpy.node import Node
from s1_interfaces.msg import S1Command, Telemetry
import random


class S2Perception(Node):
    def __init__(self):
        super().__init__("s2_perception")

        # Subscribe to perception commands from master controller
        self.sub_cmd = self.create_subscription(
            S1Command,
            "/s3/perception/cmd",
            self.cb_cmd,
            10
        )

        # Publish perception state back to S1
        self.pub_telemetry = self.create_publisher(
            Telemetry,
            "/s2/perception/state",
            10
        )

        # Publish commands to P2 driver (hardware control)
        self.pub_driver_cmd = self.create_publisher(
            S1Command,
            "/p2_driver/cmd",
            10
        )

        # State update timer (2Hz)
        self.timer = self.create_timer(0.5, self.publish_state)
        
        # Internal state
        self.current_mode = "normal"
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.obstacles_detected = 0
        
        # E-STOP latch - persists until CLEAR_ESTOP received
        self.estop_active = False
        
        self.get_logger().info("🔭 S2 Perception subsystem started")

    def cb_cmd(self, msg: S1Command):
        """Handle incoming perception commands"""
        action = msg.action
        
        # --- SAFETY: E-STOP LATCH LOGIC (HIGHEST PRIORITY) ---
        if action == "estop":
            self.estop_active = True
            self.current_mode = "emergency_stop"
            self.get_logger().error("🔴 HARD E-STOP LATCHED - Perception FROZEN")
            return
        
        elif action == "clear_estop":
            self.estop_active = False
            self.current_mode = "normal"
            self.get_logger().info("🟢 E-STOP cleared - Perception commands enabled")
            return
        
        # Block ALL commands if E-STOP is active
        if self.estop_active:
            self.get_logger().warn(f"⚠️ BLOCKED '{action}' - E-STOP is ACTIVE")
            return
        
        # --- NORMAL PERCEPTION COMMANDS (only if E-STOP is clear) ---
        if action == "scan_area":
            self.current_mode = "scanning"
            self.obstacles_detected = random.randint(0, 5)
            self.get_logger().info(f"🔍 Scanning area... detected {self.obstacles_detected} obstacles")
            # Forward command to P2 driver
            self._send_to_driver(action, msg.params)
        
        elif action == "idle":
            self.current_mode = "normal"
            self.get_logger().info("💤 Perception idle")
        
        elif action == "calibrate_sensors":
            self.current_mode = "calibrating"
            self.get_logger().info("🎯 Calibrating sensors...")
            # Forward command to P2 driver
            self._send_to_driver(action, msg.params)
        
        elif action == "track_target":
            self.current_mode = "tracking"
            params = msg.params
            target_id = params[0] if len(params) > 0 else "unknown"
            self.get_logger().info(f"🎯 Tracking target: {target_id}")
            # Forward command to P2 driver
            self._send_to_driver(action, msg.params)
        
        else:
            self.get_logger().warn(f"⚠️ Unknown perception command: {action}")
    
    def _send_to_driver(self, action: str, params: list):
        """Forward command to P2 hardware driver"""
        driver_cmd = S1Command()
        driver_cmd.stamp = self.get_clock().now().to_msg()
        driver_cmd.action = action
        driver_cmd.params = params
        self.pub_driver_cmd.publish(driver_cmd)
        self.get_logger().debug(f"→ P2 driver: {action}")

    def publish_state(self):
        """Publish simulated perception state"""
        # ENFORCE E-STOP: If E-STOP active, report emergency state
        if self.estop_active:
            self.current_mode = "emergency_stop"
        
        msg = Telemetry()
        
        # Robot identification
        msg.robot_id = "robot-alpha-s2"
        msg.safety_state = self.current_mode
        
        # Simulate pose drift (random walk)
        self.x += random.uniform(-0.01, 0.01)
        self.y += random.uniform(-0.01, 0.01)
        self.theta += random.uniform(-0.02, 0.02)
        
        msg.position_x = self.x
        msg.position_y = self.y
        msg.position_theta = self.theta
        
        # Simulate sensor readings
        msg.temperature_c = 30.0 + random.random() * 3.0  # 30-33°C
        msg.battery_pct = 100.0  # Perception doesn't drain battery much
        
        # P2 interface status (camera/LiDAR via CANopen)
        msg.p2_status = "UP"
        msg.p2_canopen = random.uniform(50, 100)  # High data rate for vision
        msg.p2_cpu_percent = random.uniform(40, 70)  # Vision processing is intensive
        msg.p2_mem_percent = random.uniform(50, 75)
        msg.p2_disk_free = random.uniform(60, 70)
        
        self.pub_telemetry.publish(msg)


def main():
    rclpy.init()
    node = S2Perception()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
