#!/bin/bash
# Emergency launch: Start master_controller standalone
# Run this in a separate terminal while other nodes are running

source /opt/ros/humble/setup.bash
source install/setup.bash

echo "🚀 Starting master_controller..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "This node will:"
echo "  • Subscribe to /s1/cmd (from command_bridge)"  
echo "  • Broadcast E-STOP to all subsystems & drivers"
echo "  • Route commands with safety gates"
echo ""
echo "Press Ctrl+C to stop"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

ros2 run s1_brain master_controller
