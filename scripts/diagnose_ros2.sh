#!/bin/bash
# Diagnostic script to check ROS2 system state

echo "=== ROS2 System Diagnostics ==="
echo ""

cd /mnt/c/Project/Main-Robotics-Processing-and-Communications-Unit/Main-Robotics-Processing-and-Communications-Unit/s1_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "1️⃣ Active ROS2 Nodes:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
ros2 node list
echo ""

echo "2️⃣ Looking for master_controller specifically:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
ros2 node list | grep -i master || echo "❌ master_controller NOT FOUND"
echo ""

echo "3️⃣ Topics available:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
ros2 topic list | grep -E "/s1/cmd|/s2/motion|/s3/perception|/s4/actuation"
echo ""

echo "4️⃣ Who is subscribed to /s1/cmd:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
ros2 topic info /s1/cmd --verbose || echo "❌ Topic /s1/cmd doesn't exist or has no subscribers"
echo ""

echo "5️⃣ Echo /s1/cmd for 3 seconds (press E-STOP now):"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
timeout 3 ros2 topic echo /s1/cmd || echo "No messages received"
