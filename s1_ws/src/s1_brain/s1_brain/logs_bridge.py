import rclpy
from rclpy.node import Node
import json
import time
import paho.mqtt.client as mqtt
from s1_interfaces.msg import LogEntry

class LogsBridge(Node):
    def __init__(self):
        super().__init__('logs_bridge')

        # ROS2 subscriber
        self.sub = self.create_subscription(
            LogEntry,
            '/s1/logs/raw',
            self.cb,
            10
        )

        # MQTT setup
        self.client = mqtt.Client()
        try:
            self.client.connect("localhost", 1883, 60)
            self.client.loop_start()
            self.get_logger().info("Connected to MQTT broker at localhost:1883")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT: {e}")

    def cb(self, msg):
        # Convert ROS2 LogEntry → MQTT JSON matching dashboard expectations
        payload = {
            "type": "log",
            "robot_id": "robot-alpha",
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "level": msg.level,
            "message": msg.message
        }

        self.client.publish("robot/robot-alpha/log", json.dumps(payload))
        self.get_logger().info(f"[{msg.source_node}] MQTT log sent: [{msg.level}] {msg.message}")

def main(args=None):
    rclpy.init(args=args)
    node = LogsBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.client.loop_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
