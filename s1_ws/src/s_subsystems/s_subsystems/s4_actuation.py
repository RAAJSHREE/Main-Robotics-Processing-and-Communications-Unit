#!/usr/bin/env python3
"""
S4 Actuation Subsystem
Simulates LED control, buzzer, gripper, and other actuators.

Receives:  /s4/actuation/cmd (S1Command)
Publishes: /s4/actuation/state (Telemetry)
"""

import rclpy
from rclpy.node import Node
from s1_interfaces.msg import S1Command, Telemetry
import random


class S4Actuation(Node):
    def __init__(self):
        super().__init__("s4_actuation")

        # Subscribe to actuation commands from master controller
        self.sub_cmd = self.create_subscription(
            S1Command,
            "/s4/actuation/cmd",
            self.cb_cmd,
            10
        )

        # Publish actuation state back to S1
        self.pub_state = self.create_publisher(
            Telemetry,
            "/s4/actuation/state",
            10
        )

        # Publish commands to P4 driver (hardware control)
        self.pub_driver_cmd = self.create_publisher(
            S1Command,
            "/p4_driver/cmd",
            10
        )

        # State update timer (5Hz)
        self.timer = self.create_timer(0.2, self.publish_state)
        
        # Actuator states
        self.led_on = False
        self.buzzer_active = False
        self.gripper_closed = False
        self.servo_angle = 0.0
        
        # E-STOP latch - persists until CLEAR_ESTOP received
        self.estop_active = False
        
        self.get_logger().info("🔦 S4 Actuation subsystem started")

    def cb_cmd(self, msg: S1Command):
        """Handle incoming actuation commands"""
        action = msg.action
        
        # --- SAFETY: E-STOP LATCH LOGIC (HIGHEST PRIORITY) ---
        if action == "estop":
            self.estop_active = True
            self.led_on = False  # Force all actuators OFF
            self.buzzer_active = False
            self.gripper_closed = False
            self.get_logger().error("🔴 HARD E-STOP LATCHED - All actuators DISABLED")
            return
        
        elif action == "clear_estop":
            self.estop_active = False
            self.get_logger().info("🟢 E-STOP cleared - Actuation commands enabled")
            return
        
        # Block ALL commands if E-STOP is active (except safe LED commands)
        if self.estop_active:
            # Allow LED commands even during E-STOP (for status indication)
            if action not in ["led_on", "led_off"]:
                self.get_logger().warn(f"⚠️ BLOCKED '{action}' - E-STOP is ACTIVE")
                return
        
        # --- NORMAL ACTUATION COMMANDS (only if E-STOP is clear) ---
        if action == "led_on":
            self.led_on = True
            self.get_logger().info("💡 LED turned ON")
            # Forward command to P4 driver
            self._send_to_driver(action, msg.params)
        
        elif action == "led_off":
            self.led_on = False
            self.get_logger().info("🔲 LED turned OFF")
            # Forward command to P4 driver
            self._send_to_driver(action, msg.params)
        
        elif action == "beep":
            self.buzzer_active = True
            self.get_logger().info("🔊 Buzzer BEEP!")
            # Forward command to P4 driver
            self._send_to_driver(action, msg.params)
            # Auto-reset buzzer after publishing
        
        elif action == "gripper_open":
            self.gripper_closed = False
            self.get_logger().info("🤏 Gripper opened")
            # Forward command to P4 driver
            self._send_to_driver(action, msg.params)
        
        elif action == "gripper_close":
            self.gripper_closed = True
            self.get_logger().info("✊ Gripper closed")
            # Forward command to P4 driver
            self._send_to_driver(action, msg.params)
        
        elif action == "set_servo":
            params = msg.params
            if len(params) > 0:
                try:
                    self.servo_angle = float(params[0])
                    self.get_logger().info(f"🎚️ Servo angle set to {self.servo_angle}°")
                except ValueError:
                    self.get_logger().warn(f"⚠️ Invalid servo angle: {params[0]}")
        
        else:
            self.get_logger().warn(f"⚠️ Unknown actuation command: {action}")
        
        # Immediately publish state after command
        self.publish_state()
    
    def _send_to_driver(self, action: str, params: list):
        """Forward command to P4 hardware driver"""
        driver_cmd = S1Command()
        driver_cmd.stamp = self.get_clock().now().to_msg()
        driver_cmd.action = action
        driver_cmd.params = params
        self.pub_driver_cmd.publish(driver_cmd)
        self.get_logger().debug(f"→ P4 driver: {action}")

    def publish_state(self):
        """Publish current actuation state"""
        # ENFORCE E-STOP: Keep actuators disabled while E-STOP active
        if self.estop_active:
            self.buzzer_active = False
            self.gripper_closed = False
            # LED allowed for status indication
        
        msg = Telemetry()
        
        # Robot identification
        msg.robot_id = "robot-alpha-s4"
        msg.led = self.led_on
        
        # State description
        if self.buzzer_active:
            msg.safety_state = "buzzer"
            self.buzzer_active = False  # Auto-clear buzzer
        elif self.gripper_closed:
            msg.safety_state = "gripper_closed"
        elif self.led_on:
            msg.safety_state = "led_on"
        else:
            msg.safety_state = "idle"
        
        # P4 interface status (actuators via Optical/LVDS)
        msg.p4_status = "UP"
        msg.p4_lvds = random.uniform(5, 15)
        msg.p4_optical = random.uniform(1, 5)
        msg.p4_cpu_percent = random.uniform(10, 30)
        msg.p4_mem_percent = random.uniform(20, 40)
        msg.p4_disk_free = random.uniform(75, 85)
        
        # Actuators don't draw much power
        msg.battery_pct = 100.0
        msg.temperature_c = 28.0 + random.uniform(0, 2)
        msg.voltage_v = 24.0 + random.uniform(-0.1, 0.1)
        
        self.pub_state.publish(msg)


def main():
    rclpy.init()
    node = S4Actuation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
