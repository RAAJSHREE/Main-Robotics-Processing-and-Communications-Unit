#!/bin/bash
# Launch Full S1 Robot System
# Run this inside WSL

set -e

echo "🚀 Launching Full S1 Robot System..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Navigate to workspace
cd /mnt/c/Project/Main-Robotics-Processing-and-Communications-Unit/Main-Robotics-Processing-and-Communications-Unit/s1_ws

# Source ROS2
source /opt/ros/humble/setup.bash

# Source workspace
source install/setup.bash

echo "✅ ROS2 Humble sourced"
echo "✅ Workspace sourced"
echo ""
echo "📡 Launching 13 nodes (3 drivers + 7 brain + 3 subsystems)..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Launch full robot
ros2 launch s1_bringup full_robot.launch.py
