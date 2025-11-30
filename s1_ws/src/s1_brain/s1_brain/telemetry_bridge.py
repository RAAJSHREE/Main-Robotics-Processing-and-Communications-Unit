import rclpy
from rclpy.node import Node
import json
import time
import paho.mqtt.client as mqtt
from s1_interfaces.msg import Telemetry

class TelemetryBridge(Node):
    def __init__(self):
        super().__init__('telemetry_bridge')

        # ROS2 subscriber
        self.sub = self.create_subscription(
            Telemetry,
            '/s1/telemetry/raw',
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

    def cb(self, msg: Telemetry):
        # Convert ROS2 Telemetry → MQTT JSON matching dashboard expectations
        payload = {
            "type": "telemetry",
            "robot_id": msg.robot_id,
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "battery_pct": round(msg.battery_pct, 1),
            "voltage_v": round(msg.voltage_v, 2),
            "temperature_c": round(msg.temperature_c, 2),
            "position": {
                "x": round(msg.position_x, 3),
                "y": round(msg.position_y, 3),
                "theta": round(msg.position_theta, 3)
            },
            "load": {
                "motor1": 0.0,  # Can be added to message later
                "motor2": 0.0
            },
            "interfaces": {
                "P2": {
                    "canopen": round(msg.p2_canopen, 2),
                    "ethernet": round(msg.p2_ethernet, 2),
                    "lvds": round(msg.p2_lvds, 2),
                    "optical": round(msg.p2_optical, 2),
                    "status": msg.p2_status
                },
                "P3": {
                    "canopen": round(msg.p3_canopen, 2),
                    "ethernet": round(msg.p3_ethernet, 2),
                    "lvds": round(msg.p3_lvds, 2),
                    "optical": round(msg.p3_optical, 2),
                    "status": msg.p3_status
                },
                "P4": {
                    "canopen": round(msg.p4_canopen, 2),
                    "ethernet": round(msg.p4_ethernet, 2),
                    "lvds": round(msg.p4_lvds, 2),
                    "optical": round(msg.p4_optical, 2),
                    "status": msg.p4_status
                }
            },
            "interface_status": {
                "p2_cpu_percent": round(msg.p2_cpu_percent, 1),
                "p2_mem_percent": round(msg.p2_mem_percent, 1),
                "p2_disk_free": round(msg.p2_disk_free, 1),
                "p3_cpu_percent": round(msg.p3_cpu_percent, 1),
                "p3_mem_percent": round(msg.p3_mem_percent, 1),
                "p3_disk_free": round(msg.p3_disk_free, 1),
                "p4_cpu_percent": round(msg.p4_cpu_percent, 1),
                "p4_mem_percent": round(msg.p4_mem_percent, 1),
                "p4_disk_free": round(msg.p4_disk_free, 1)
            },
            "led": msg.led,
            "estop": msg.estop,
            "heartbeat": 0
        }

        self.client.publish("robot/robot-alpha/telemetry", json.dumps(payload))
        self.get_logger().info(f"[TELEMETRY → MQTT] Sent telemetry for {msg.robot_id}")

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryBridge()
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
