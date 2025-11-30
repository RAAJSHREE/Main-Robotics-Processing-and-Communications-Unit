from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # C++ Driver Nodes (simulated hardware)
        Node(
            package='s1_drivers',
            executable='driver_node',
            name='p2_driver',
            parameters=[{'driver_name': 'p2_driver'}],
            output='screen'
        ),
        Node(
            package='s1_drivers',
            executable='driver_node',
            name='p3_driver',
            parameters=[{'driver_name': 'p3_driver'}],
            output='screen'
        ),
        Node(
            package='s1_drivers',
            executable='driver_node',
            name='p4_driver',
            parameters=[{'driver_name': 'p4_driver'}],
            output='screen'
        ),
        
        # S1 Brain Bridges (ROS2 → MQTT)
        Node(
            package='s1_brain',
            executable='heartbeat_bridge',
            name='heartbeat_bridge',
            output='screen'
        ),
        Node(
            package='s1_brain',
            executable='logs_bridge',
            name='logs_bridge',
            output='screen'
        ),
        Node(
            package='s1_brain',
            executable='telemetry_publisher',
            name='telemetry_publisher',
            output='screen'
        ),
        Node(
            package='s1_brain',
            executable='telemetry_bridge',
            name='telemetry_bridge',
            output='screen'
        ),
        Node(
            package='s1_brain',
            executable='diagnostics_node',
            name='diagnostics_node',
            output='screen'
        ),
        Node(
            package='s1_brain',
            executable='master_controller',
            name='master_controller',
            output='screen'
        ),
    ])
