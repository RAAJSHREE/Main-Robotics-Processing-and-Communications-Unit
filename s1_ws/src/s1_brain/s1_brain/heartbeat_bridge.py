import rclpy
from rclpy.node import Node
import json
import time
import psutil
import paho.mqtt.client as mqtt
from s1_interfaces.msg import Heartbeat

class HeartbeatBridge(Node):
    def __init__(self):
        super().__init__('heartbeat_bridge')

        # ROS2 subscriber
        self.sub = self.create_subscription(
            Heartbeat,
            '/s1/heartbeat/raw',
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

        self.counter = 0

    def cb(self, msg):
        self.counter += 1
        
        # Convert ROS2 Heartbeat → MQTT JSON matching dashboard expectations
        payload = {
            "type": "heartbeat",
            "robot_id": "robot-alpha",
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "uptime_s": int(time.time()),
            "status": msg.status,
            "heartbeat": 1,
            "raw_beat": self.counter + (time.time() % 1),  # for graph wave
            "cpu_percent": psutil.cpu_percent(),
            "mem_percent": psutil.virtual_memory().percent
        }

        self.client.publish("robot/robot-alpha/heartbeat", json.dumps(payload))
        self.get_logger().info(f"[{msg.node_name}] MQTT heartbeat sent: {msg.status}")

def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatBridge()
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
