# 🤖 Main Robotics Processing and Communications Unit - Startup Guide

## Complete System Startup (Recommended)

### Step 1: Start Dashboard Services (MQTT, Backend, Frontend)
```powershell
# In PowerShell - navigate to Dashboard folder
cd C:\Project\Main-Robotics-Processing-and-Communications-Unit\Main-Robotics-Processing-and-Communications-Unit\Dashboard

# Start docker services
docker-compose up -d

# Verify services are running
docker-compose ps
```

**Services started:**
- ✅ Mosquitto MQTT Broker (ports 1883, 9001)
- ✅ FastAPI Backend (port 8000)
- ✅ Dashboard Frontend (port 8080)
- ✅ Robot Agent (simulation)

---

### Step 2: Start ROS2 Nodes (Brain + Drivers + Subsystems)
```powershell
# In a NEW PowerShell/WSL terminal
wsl bash /mnt/c/Project/Main-Robotics-Processing-and-Communications-Unit/Main-Robotics-Processing-and-Communications-Unit/scripts/launch_full_system.sh
```

**This launches 13 ROS2 nodes:**

**Layer 1 - Hardware Drivers (C++):**
- p2_driver (Perception: Camera/LiDAR)
- p3_driver (Motion: Motors)
- p4_driver (Actuation: LED/Buzzer/Gripper)

**Layer 2 - S1 Brain (Python):**
- telemetry_publisher (generates robot telemetry)
- diagnostics_node (health monitoring)
- **master_controller** (command routing & safety)
- command_bridge (MQTT → ROS2 commands)
- heartbeat_bridge (ROS2 → MQTT heartbeats)
- logs_bridge (ROS2 → MQTT logs)
- telemetry_bridge (ROS2 → MQTT telemetry)

**Layer 3 - Subsystems (Python):**
- s2_perception (vision processing simulation)
- s3_motion (movement simulation)
- s4_actuation (LED/actuator simulation)

---

### Step 3: Access Dashboard
Open browser: **http://localhost:8080**

---

## Quick Start (All-in-One Script)

```powershell
# Create this PowerShell script to launch everything
cd C:\Project\Main-Robotics-Processing-and-Communications-Unit\Main-Robotics-Processing-and-Communications-Unit

# Start Docker services
cd Dashboard
docker-compose up -d
cd ..

# Wait for MQTT to be ready
Start-Sleep -Seconds 3

# Start ROS2 in background (new terminal will open)
Start-Process powershell -ArgumentList "-NoExit", "-Command", "wsl bash /mnt/c/Project/Main-Robotics-Processing-and-Communications-Unit/Main-Robotics-Processing-and-Communications-Unit/scripts/launch_full_system.sh"

# Open dashboard
Start-Process "http://localhost:8080"

Write-Host "✅ System starting..."
Write-Host "📊 Dashboard: http://localhost:8080"
Write-Host "🔧 Backend API: http://localhost:8000"
Write-Host "📡 MQTT Broker: localhost:1883"
```

---

## Verify System is Running

### Check Docker Services
```powershell
cd Dashboard
docker-compose ps
```

### Check ROS2 Nodes
```bash
# In WSL terminal
cd /mnt/c/Project/Main-Robotics-Processing-and-Communications-Unit/Main-Robotics-Processing-and-Communications-Unit/s1_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# List all nodes (should show 13 nodes)
ros2 node list

# Check if master_controller is running
ros2 node list | grep master

# Check command topic
ros2 topic info /s1/cmd --verbose
```

Expected output:
```
Subscription count: 1    ← master_controller is subscribed ✅
```

---

## Test E-STOP Functionality

1. Open dashboard: http://localhost:8080
2. Press **"E-STOP"** button (red)
3. Check logs - you should see:
   ```
   [master_controller] 🛑 E-STOP ACTIVATED BY COMMAND
   [master_controller] ⚠️ E-STOP broadcast to ALL subsystems + drivers
   [p2_driver] 🔴 HARDWARE E-STOP ACTIVATED
   [p3_driver] 🔴 HARDWARE E-STOP ACTIVATED
   [p4_driver] 🔴 HARDWARE E-STOP ACTIVATED
   [s3_motion] 🔴 HARD E-STOP LATCHED
   ```
4. Heartbeats change from "OK" to "E-STOP"
5. Press **"Clear E-STOP"** to resume

---

## Troubleshooting

### master_controller not starting?
```bash
# Run diagnostic
wsl bash /mnt/c/Project/Main-Robotics-Processing-and-Communications-Unit/Main-Robotics-Processing-and-Communications-Unit/scripts/diagnose_ros2.sh

# If master_controller missing, launch it manually:
wsl bash /mnt/c/Project/Main-Robotics-Processing-and-Communications-Unit/Main-Robotics-Processing-and-Communications-Unit/scripts/launch_master_controller.sh
```

### Dashboard not connecting?
```powershell
# Check backend logs
cd Dashboard
docker-compose logs backend

# Check MQTT logs
docker-compose logs mosquitto

# Restart services
docker-compose restart
```

### Commands not working?
1. Verify master_controller is running: `ros2 node list | grep master`
2. Check `/s1/cmd` has 1 subscriber: `ros2 topic info /s1/cmd`
3. Monitor command flow: `ros2 topic echo /s1/cmd`

---

## Shutdown Sequence

### Stop ROS2 Nodes
Press `Ctrl+C` in the terminal running ROS2 launch

### Stop Docker Services
```powershell
cd Dashboard
docker-compose down
```

---

## Architecture Overview

```
┌─────────────────────────────────────────────────┐
│  Dashboard (Browser) - http://localhost:8080    │
└────────────────┬────────────────────────────────┘
                 │ WebSocket + REST API
                 ▼
┌─────────────────────────────────────────────────┐
│  FastAPI Backend (Docker) - :8000               │
└────────────────┬────────────────────────────────┘
                 │ MQTT
                 ▼
┌─────────────────────────────────────────────────┐
│  Mosquitto Broker (Docker) - :1883              │
└──┬────────────────────────────────────────────┬─┘
   │                                            │
   ▼                                            ▼
┌──────────────────┐                    ┌──────────────────┐
│ robot_agent      │                    │ command_bridge   │
│ (Docker)         │                    │ (ROS2 Node)      │
│ Simulation only  │                    │ MQTT → ROS2      │
└──────────────────┘                    └────────┬─────────┘
                                                 │
                                                 ▼ /s1/cmd
                                        ┌──────────────────┐
                                        │ master_controller│
                                        │ Safety & Routing │
                                        └────────┬─────────┘
                                                 │
                    ┌────────────────────────────┼────────────────────────┐
                    ▼                            ▼                        ▼
            ┌──────────────┐           ┌──────────────┐         ┌──────────────┐
            │ Subsystems   │           │ Subsystems   │         │ Subsystems   │
            │ s2/s3/s4     │           │ (Python)     │         │ (Python)     │
            └──────┬───────┘           └──────┬───────┘         └──────┬───────┘
                   │                          │                        │
                   ▼                          ▼                        ▼
            ┌──────────────┐           ┌──────────────┐         ┌──────────────┐
            │ p2_driver    │           │ p3_driver    │         │ p4_driver    │
            │ (C++)        │           │ (C++)        │         │ (C++)        │
            └──────────────┘           └──────────────┘         └──────────────┘
```

---

## Next Steps

✅ System is ready when you see all 13 ROS2 nodes running + 4 Docker containers
✅ Test E-STOP, movement commands, LED controls from dashboard
✅ Monitor real-time telemetry, logs, and heartbeats
