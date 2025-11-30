# Command message for MQTT → ROS2 control bridge
# Used by dashboard to send control commands to robot subsystems

string type          # "move" / "led" / "safety" / "perception" / "actuation"
string action        # "forward" / "back" / "stop" / "E_STOP" / "led_on" / etc.
string direction     # "forward" / "back" / "left" / "right" (optional)
string value         # Additional parameter value (optional)
string robot_id      # Robot identifier (e.g., "robot-alpha")
