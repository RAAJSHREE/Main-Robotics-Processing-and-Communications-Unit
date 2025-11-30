import rclpy
from rclpy.node import Node
from s1_interfaces.msg import S1Command
import sys

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.publisher = self.create_publisher(S1Command, '/s1/cmd', 10)
        
    def send_command(self, target, action, params=None):
        msg = S1Command()
        msg.stamp = self.get_clock().now().to_msg()
        msg.target = target
        msg.action = action
        msg.params = params if params else []
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent command: {target}/{action}')

def main(args=None):
    rclpy.init(args=args)
    node = CommandPublisher()
    
    if len(sys.argv) < 3:
        print("Usage: ros2 run s1_brain command_test <target> <action> [params...]")
        print("Examples:")
        print("  ros2 run s1_brain command_test motion move_forward")
        print("  ros2 run s1_brain command_test motion stop")
        print("  ros2 run s1_brain command_test actuation led_on")
        print("  ros2 run s1_brain command_test system status")
        rclpy.shutdown()
        return
    
    target = sys.argv[1]
    action = sys.argv[2]
    params = sys.argv[3:] if len(sys.argv) > 3 else []
    
    node.send_command(target, action, params)
    
    # Give it time to send
    rclpy.spin_once(node, timeout_sec=1.0)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
