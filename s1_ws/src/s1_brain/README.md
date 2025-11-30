# S1 Brain - ROS2 to MQTT Bridge 🧠

## Architecture

```
┌─────────────────┐       ┌──────────────┐       ┌──────────────┐       ┌──────────────┐
│  C++ Drivers    │──ROS2─▶│  S1 Brain    │──MQTT─▶│  Mosquitto   │──WS──▶│  Dashboard   │
│  (P2/P3/P4)     │       │  (Bridges)   │       │  Broker      │       │  (Web UI)    │
└─────────────────┘       └──────────────┘       └──────────────┘       └──────────────┘
     driver_node          heartbeat_bridge                               index.html
                          logs_bridge                                    advanced.html
                          telemetry_bridge
```

## What This Does

### ROS2 Layer (C++)
- **s1_drivers**: Simulates P2/P3/P4 hardware interfaces
  - Publishes to `/s1/heartbeat/raw` (100ms)
  - Publishes to `/s1/logs/raw` (3s)

### S1 Brain Layer (Python)
- **heartbeat_bridge**: `/s1/heartbeat/raw` → `robot/robot-alpha/heartbeat`
- **logs_bridge**: `/s1/logs/raw` → `robot/robot-alpha/log`
- **telemetry_bridge**: Generates telemetry → `robot/robot-alpha/telemetry`

### MQTT Topics (Dashboard expects these)
- `robot/robot-alpha/heartbeat` - System health
- `robot/robot-alpha/telemetry` - Robot state, P2/P3/P4 status
- `robot/robot-alpha/log` - Log messages

## Quick Start

### Terminal 1: Start MQTT Broker (if not running)
```bash
# Check if mosquitto is running
ps aux | grep mosquitto

# If not, start it
mosquitto -c Dashboard/mosquitto/config/mosquitto.conf
```

### Terminal 2: Launch Everything (ROS2 + Bridges)
```bash
cd ~/s1_ws
source install/setup.bash
ros2 launch s1_brain s1_bringup.launch.py
```

This starts:
- 3 driver nodes (p2_driver, p3_driver, p4_driver)
- 3 bridge nodes (heartbeat, logs, telemetry)

### Terminal 3: Monitor MQTT Traffic
```bash
mosquitto_sub -t 'robot/robot-alpha/#' -v
```

You should see:
```
robot/robot-alpha/heartbeat {"type":"heartbeat","robot_id":"robot-alpha",...}
robot/robot-alpha/log {"type":"log","level":"INFO",...}
robot/robot-alpha/telemetry {"type":"telemetry","battery_pct":95.3,...}
```

### Terminal 4: Start Dashboard Backend
```bash
cd Dashboard/backend
uvicorn main:app --host 0.0.0.0 --port 8000
```

### Browser: Open Dashboard
```
http://localhost:8000/static/index.html
http://localhost:8000/static/advanced.html
```

## Manual Testing

### Test Individual Components

**Run single driver:**
```bash
ros2 run s1_drivers driver_node --ros-args -p driver_name:=p2_driver
```

**Run single bridge:**
```bash
ros2 run s1_brain heartbeat_bridge
```

**Check ROS2 topics:**
```bash
ros2 topic list
ros2 topic echo /s1/heartbeat/raw
ros2 topic echo /s1/logs/raw
```

**Check MQTT topics:**
```bash
mosquitto_sub -t 'robot/robot-alpha/heartbeat'
mosquitto_sub -t 'robot/robot-alpha/log'
mosquitto_sub -t 'robot/robot-alpha/telemetry'
```

## Dependencies

### Python packages needed:
```bash
pip3 install paho-mqtt psutil
```

### ROS2 packages:
- rclpy
- s1_interfaces
- s1_drivers
- s1_brain

## Troubleshooting

**MQTT Connection Failed:**
- Check mosquitto is running: `ps aux | grep mosquitto`
- Check port 1883 is open: `netstat -an | grep 1883`

**No messages on dashboard:**
- Verify MQTT topics: `mosquitto_sub -t 'robot/#' -v`
- Check bridge logs: Look for "MQTT heartbeat sent" messages

**Dashboard not updating:**
- Check WebSocket connection in browser console
- Verify FastAPI backend is running on port 8000

## Next Steps

- Add heartbeat_manager (aggregates multiple drivers)
- Add safety_logic (E-STOP triggers)
- Add diagnostics_node (health monitoring)
- Add master_controller (mission control)
