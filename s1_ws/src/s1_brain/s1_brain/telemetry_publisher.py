import rclpy
from rclpy.node import Node
import random
from s1_interfaces.msg import Telemetry

class TelemetryPublisher(Node):
    def __init__(self):
        super().__init__('telemetry_publisher')

        self.pub = self.create_publisher(Telemetry, '/s1/telemetry/raw', 10)
        self.timer = self.create_timer(3.0, self.publish_telemetry)
        
        self.battery_pct = 100.0
        self.position_x = 0.0
        self.position_y = 0.0
        
        self.get_logger().info("Telemetry publisher started")

    def publish_telemetry(self):
        msg = Telemetry()
        
        # Basic info
        msg.stamp = self.get_clock().now().to_msg()
        msg.robot_id = "robot-alpha"
        msg.safety_state = "NORMAL"
        
        # Battery and power (simulate drain)
        self.battery_pct = max(0.0, self.battery_pct - random.uniform(0.0, 0.2))
        msg.battery_pct = self.battery_pct
        msg.voltage_v = 24.0 + random.uniform(-0.5, 0.5)
        msg.temperature_c = 30.0 + random.uniform(0, 10)
        
        # Position (random walk)
        self.position_x += random.uniform(-0.1, 0.1)
        self.position_y += random.uniform(-0.1, 0.1)
        msg.position_x = self.position_x
        msg.position_y = self.position_y
        msg.position_theta = random.uniform(-3.14, 3.14)
        
        # P2 Interface (CANOpen dominant)
        msg.p2_canopen = random.uniform(50, 200)
        msg.p2_ethernet = random.uniform(0, 50)
        msg.p2_lvds = 0.0
        msg.p2_optical = 0.0
        msg.p2_status = "UP" if random.random() > 0.05 else "DOWN"
        msg.p2_cpu_percent = random.uniform(10, 60)
        msg.p2_mem_percent = random.uniform(20, 50)
        msg.p2_disk_free = random.uniform(60, 95)
        
        # P3 Interface (Ethernet + LVDS)
        msg.p3_canopen = random.uniform(0, 20)
        msg.p3_ethernet = random.uniform(200, 1000)
        msg.p3_lvds = random.uniform(10, 50)
        msg.p3_optical = 0.0
        msg.p3_status = "UP" if random.random() > 0.05 else "DOWN"
        msg.p3_cpu_percent = random.uniform(15, 70)
        msg.p3_mem_percent = random.uniform(25, 60)
        msg.p3_disk_free = random.uniform(50, 90)
        
        # P4 Interface (Optical + Mixed)
        msg.p4_canopen = random.uniform(0, 30)
        msg.p4_ethernet = random.uniform(100, 500)
        msg.p4_lvds = random.uniform(0, 20)
        msg.p4_optical = random.uniform(5, 15)
        msg.p4_status = "UP" if random.random() > 0.05 else "DOWN"
        msg.p4_cpu_percent = random.uniform(20, 80)
        msg.p4_mem_percent = random.uniform(30, 70)
        # Simulate occasional disk space warning for P4
        msg.p4_disk_free = random.uniform(60, 95) if random.random() < 0.8 else random.uniform(5, 25)
        
        # System state
        msg.led = random.random() > 0.5
        msg.estop = False  # Normal operation
        
        self.pub.publish(msg)
        self.get_logger().info(
            f"Published telemetry: Battery={msg.battery_pct:.1f}%, "
            f"P2={msg.p2_status}, P3={msg.p3_status}, P4={msg.p4_status}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
