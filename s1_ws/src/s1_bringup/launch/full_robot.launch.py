#!/usr/bin/env python3
"""
Full Robot Launch System
Launches all 12 nodes for complete S1 robot simulation:
- 3 C++ hardware drivers (P2/P3/P4)
- 6 Python brain nodes (telemetry, diagnostics, bridges, controller)
- 3 Python subsystem simulators (S2/S3/S4)

Usage:
    ros2 launch s1_bringup full_robot.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for complete S1 robot system"""

    # ======================================================================
    # LAYER 1: HARDWARE DRIVERS (C++)
    # ======================================================================
    # Simulates P2/P3/P4 interface hardware
    
    p2_driver = Node(
        package='s1_drivers',
        executable='driver_node',
        name='p2_driver',
        parameters=[{'driver_name': 'p2_driver'}],
        output='screen',
        emulate_tty=True
    )

    p3_driver = Node(
        package='s1_drivers',
        executable='driver_node',
        name='p3_driver',
        parameters=[{'driver_name': 'p3_driver'}],
        output='screen',
        emulate_tty=True
    )

    p4_driver = Node(
        package='s1_drivers',
        executable='driver_node',
        name='p4_driver',
        parameters=[{'driver_name': 'p4_driver'}],
        output='screen',
        emulate_tty=True
    )

    # ======================================================================
    # LAYER 2: S1 BRAIN NODES (Python)
    # ======================================================================
    # Core intelligence layer: telemetry, diagnostics, bridges, controller

    # Generates comprehensive robot telemetry (P2/P3/P4 interfaces)
    telemetry_publisher = Node(
        package='s1_brain',
        executable='telemetry_publisher',
        name='telemetry_publisher',
        output='screen',
        emulate_tty=True
    )

    # Monitors health and generates diagnostic reports
    diagnostics_node = Node(
        package='s1_brain',
        executable='diagnostics_node',
        name='diagnostics_node',
        output='screen',
        emulate_tty=True
    )

    # Master controller with safety enforcement
    master_controller = Node(
        package='s1_brain',
        executable='master_controller',
        name='master_controller',
        output='screen',
        emulate_tty=True
    )

    # ROS2 → MQTT bridges for dashboard integration
    heartbeat_bridge = Node(
        package='s1_brain',
        executable='heartbeat_bridge',
        name='heartbeat_bridge',
        output='screen',
        emulate_tty=True
    )

    logs_bridge = Node(
        package='s1_brain',
        executable='logs_bridge',
        name='logs_bridge',
        output='screen',
        emulate_tty=True
    )

    telemetry_bridge = Node(
        package='s1_brain',
        executable='telemetry_bridge',
        name='telemetry_bridge',
        output='screen',
        emulate_tty=True
    )

    # MQTT → ROS2 Command Bridge (enables dashboard button control)
    command_bridge = Node(
        package='s1_brain',
        executable='command_bridge',
        name='command_bridge',
        output='screen',
        emulate_tty=True
    )

    # ======================================================================
    # LAYER 3: SUBSYSTEM SIMULATORS (Python)
    # ======================================================================
    # Mock S2/S3/S4 subsystems for closed-loop testing

    # S2: Perception (vision, LiDAR, pose estimation)
    s2_perception = Node(
        package='s_subsystems',
        executable='s2_perception',
        name='s2_perception',
        output='screen',
        emulate_tty=True
    )

    # S3: Motion (movement, odometry, velocity control)
    s3_motion = Node(
        package='s_subsystems',
        executable='s3_motion',
        name='s3_motion',
        output='screen',
        emulate_tty=True
    )

    # S4: Actuation (LED, buzzer, gripper, servos)
    s4_actuation = Node(
        package='s_subsystems',
        executable='s4_actuation',
        name='s4_actuation',
        output='screen',
        emulate_tty=True
    )

    # ======================================================================
    # LAUNCH ALL NODES
    # ======================================================================
    return LaunchDescription([
        # Layer 1: Hardware (3 nodes)
        p2_driver,
        p3_driver,
        p4_driver,

        # Layer 2: S1 Brain (7 nodes - added command_bridge)
        telemetry_publisher,
        diagnostics_node,
        master_controller,
        command_bridge,
        heartbeat_bridge,
        logs_bridge,
        telemetry_bridge,

        # Layer 3: Subsystems (3 nodes)
        s2_perception,
        s3_motion,
        s4_actuation,
    ])
