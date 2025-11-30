#!/usr/bin/env python3
"""
Command Bridge: MQTT → ROS2
Receives commands from dashboard via MQTT and publishes to ROS2 topics
Enables dashboard button clicks to control robot subsystems

MQTT Subscribe: robot/+/cmd
ROS2 Publish: /s1/cmd (S1Command format for master_controller routing)
"""

import rclpy
from rclpy.node import Node
import json
import paho.mqtt.client as mqtt
from s1_interfaces.msg import S1Command


class CommandBridge(Node):
    def __init__(self):
        super().__init__("command_bridge")

        # ROS2 Publisher - uses S1Command which master_controller already understands
        self.cmd_publisher = self.create_publisher(S1Command, "/s1/cmd", 10)

        # MQTT Client Setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        mqtt_host = "localhost"
        mqtt_port = 1883
        
        try:
            self.mqtt_client.connect(mqtt_host, mqtt_port, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info(f"🚀 Command Bridge started: MQTT({mqtt_host}:{mqtt_port}) → ROS2(/s1/cmd)")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to connect to MQTT broker: {e}")

    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback when connected to MQTT broker"""
        if rc == 0:
            self.get_logger().info("✅ Connected to MQTT broker")
            # Subscribe to all robot command topics
            client.subscribe("robot/+/cmd")
            self.get_logger().info("📡 Subscribed to: robot/+/cmd")
        else:
            self.get_logger().error(f"❌ MQTT connection failed with code: {rc}")

    def on_mqtt_message(self, client, userdata, msg):
        """
        Callback when MQTT command received from dashboard
        Converts dashboard command format to ROS2 S1Command format
        """
        try:
            # Parse MQTT payload
            payload = json.loads(msg.payload.decode())
            self.get_logger().info(f"📥 MQTT Command received on {msg.topic}: {payload}")

            # Extract command fields from dashboard format
            cmd_type = payload.get("type", "")
            action = payload.get("action", "").lower()  # Normalize to lowercase
            direction = payload.get("direction", "")
            value = payload.get("value", "")
            robot_id = payload.get("robot_id", "robot-alpha")

            # Map dashboard command to ROS2 S1Command format
            ros_cmd = S1Command()
            ros_cmd.stamp = self.get_clock().now().to_msg()
            
            # NORMALIZATION LAYER - Convert various command formats to standard ROS2 format
            
            # Safety commands (highest priority)
            if action in ["e_stop", "estop", "emergency_stop"]:
                ros_cmd.target = "system"
                ros_cmd.action = "estop"
            elif action in ["clear_estop", "clear_e_stop"]:
                ros_cmd.target = "system"
                ros_cmd.action = "clear_estop"
            
            # Motion commands
            elif action in ["forward", "back", "left", "right"] or cmd_type in ["move", "motion"]:
                ros_cmd.target = "motion"
                if action == "forward" or direction == "forward":
                    ros_cmd.action = "move_forward"
                elif action == "back" or direction == "back":
                    ros_cmd.action = "move_back"
                elif action == "left" or direction == "left":
                    ros_cmd.action = "turn_left"
                elif action == "right" or direction == "right":
                    ros_cmd.action = "turn_right"
                elif action == "stop":
                    ros_cmd.action = "stop"
                else:
                    ros_cmd.action = action
            
            # LED/Actuation commands
            elif action.startswith("led") or cmd_type in ["led", "actuation"]:
                ros_cmd.target = "actuation"
                if action in ["on", "led_on"] or action.endswith("/on"):
                    ros_cmd.action = "led_on"
                elif action in ["off", "led_off"] or action.endswith("/off"):
                    ros_cmd.action = "led_off"
                else:
                    ros_cmd.action = action
            
            # Perception commands
            elif cmd_type == "perception":
                ros_cmd.target = "perception"
                ros_cmd.action = action
            
            else:
                # Default fallback
                ros_cmd.target = cmd_type if cmd_type else "system"
                ros_cmd.action = action if action else "status"
            
            # Add parameters if provided
            if value:
                ros_cmd.params = [value]
            
            # Publish to ROS2 for master_controller to route
            self.cmd_publisher.publish(ros_cmd)
            
            self.get_logger().info(
                f"✅ Command bridged → ROS2: target={ros_cmd.target}, action={ros_cmd.action}"
            )

        except json.JSONDecodeError as e:
            self.get_logger().error(f"❌ Invalid JSON in MQTT payload: {e}")
        except Exception as e:
            self.get_logger().error(f"❌ Error processing MQTT command: {e}")

    def destroy_node(self):
        """Cleanup on shutdown"""
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CommandBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Command Bridge shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
