# 🤖 S2/S3/S4 Subsystems - Mock Robot Simulation

## ✅ Complete Closed-Loop Architecture

```
Dashboard → MQTT → S1 Brain → S2/S3/S4 Subsystems → S1 Brain → MQTT → Dashboard
```

---

## 📦 Subsystem Components

### 🔭 S2 — Perception Subsystem
**Purpose:** Simulates vision, LiDAR, pose estimation, obstacle detection

**Receives:** `/s3/perception/cmd` (S1Command from master_controller)  
**Publishes:** `/s2/perception/state` (Telemetry)

**Supported Commands:**
- `scan_area` - Perform area scan, detect obstacles
- `idle` - Return to normal mode
- `calibrate_sensors` - Calibrate cameras/LiDAR
- `track_target` - Track specific target (params: target_id)

**Features:**
- Random walk pose simulation (x, y, theta drift)
- Obstacle detection (0-5 random obstacles when scanning)
- P2 interface metrics (CANopen for camera/LiDAR)
- CPU-intensive simulation (40-70% load for vision processing)
- Updates at 2Hz

---

### 🏎️ S3 — Motion Subsystem
**Purpose:** Simulates robot movement, odometry, velocity control

**Receives:** `/s2/motion/cmd` (S1Command from master_controller)  
**Publishes:** `/s3/motion/state` (Telemetry)

**Supported Commands:**
- `move_forward` - Move in heading direction
- `move_back` - Reverse movement
- `turn_left` - Rotate counter-clockwise
- `turn_right` - Rotate clockwise
- `stop` - Halt all motion
- `set_speed` - Change speed (params: speed_m/s)

**Features:**
- Kinematic simulation with proper pose integration
- Battery drain during motion (faster drain for translation)
- Motor temperature simulation (45-55°C when moving)
- Voltage drop under load (24V → 23.5V)
- P3 interface metrics (Ethernet for motor controllers)
- Updates at 10Hz (smooth motion)

---

### 🔦 S4 — Actuation Subsystem
**Purpose:** Simulates LEDs, buzzer, gripper, servo actuators

**Receives:** `/s4/actuation/cmd` (S1Command from master_controller)  
**Publishes:** `/s4/actuation/state` (Telemetry)

**Supported Commands:**
- `led_on` - Turn LED on
- `led_off` - Turn LED off
- `beep` - Activate buzzer (auto-clears)
- `gripper_open` - Open gripper
- `gripper_close` - Close gripper
- `set_servo` - Set servo angle (params: angle_degrees)

**Features:**
- Immediate state response to commands
- P4 interface metrics (LVDS + Optical for actuators)
- Low power consumption simulation
- Buzzer auto-reset after one cycle
- Updates at 5Hz

---

## 🚀 Quick Start

### Launch All Subsystems (Manual)
```bash
# Terminal 1: S2 Perception
ros2 run s_subsystems s2_perception

# Terminal 2: S3 Motion
ros2 run s_subsystems s3_motion

# Terminal 3: S4 Actuation
ros2 run s_subsystems s4_actuation
```

### Send Commands from S1
```bash
# Test perception
ros2 run s1_brain command_test perception scan_area

# Test motion
ros2 run s1_brain command_test motion move_forward

# Test actuation
ros2 run s1_brain command_test actuation led_on
```

### Monitor Subsystem States
```bash
# Watch all subsystem telemetry
ros2 topic echo /s2/perception/state
ros2 topic echo /s3/motion/state
ros2 topic echo /s4/actuation/state
```

---

## 🔍 Topic Flow

```
External → /s1/cmd (S1Command)
              ↓
    [master_controller - safety gates]
              ↓
    ┌─────────┼─────────┐
    ↓         ↓         ↓
/s3/perception/cmd  /s2/motion/cmd  /s4/actuation/cmd
    ↓         ↓         ↓
[S2 Perception] [S3 Motion]  [S4 Actuation]
    ↓         ↓         ↓
/s2/perception/state  /s3/motion/state  /s4/actuation/state
                      ↓
           [S1 can monitor subsystem health]
```

---

## 🎯 Full System Architecture

```
Layer 1: Hardware Drivers (C++)
├─ p2_driver, p3_driver, p4_driver
└─ Publish: /s1/heartbeat/raw, /s1/logs/raw

Layer 2: S1 Brain (Python)
├─ telemetry_publisher → /s1/telemetry/raw
├─ diagnostics_node → /s1/diagnostics/report
├─ master_controller → Routes commands with safety
├─ heartbeat_bridge → MQTT
├─ logs_bridge → MQTT
└─ telemetry_bridge → MQTT

Layer 3: Subsystems (Python) ⭐ NEW
├─ S2 Perception → Vision/LiDAR simulation
├─ S3 Motion → Movement simulation
└─ S4 Actuation → LED/buzzer/gripper simulation

Layer 4: Cloud
└─ Dashboard (FastAPI + WebSocket)
```

---

## 🧪 Testing Scenarios

### Scenario 1: Motion Control
```bash
# Start motion
ros2 run s1_brain command_test motion move_forward

# Monitor position change
ros2 topic echo /s3/motion/state | grep position

# Stop motion
ros2 run s1_brain command_test motion stop
```

### Scenario 2: Perception Scan
```bash
# Trigger scan
ros2 run s1_brain command_test perception scan_area

# Watch S2 logs for obstacle count
# Check /s2/perception/state for updated pose
```

### Scenario 3: LED Control
```bash
# Turn on LED
ros2 run s1_brain command_test actuation led_on

# Verify state
ros2 topic echo /s4/actuation/state | grep led

# Turn off
ros2 run s1_brain command_test actuation led_off
```

### Scenario 4: Safety Enforcement
```bash
# Create critical condition (simulate in diagnostics_node)
# Try motion command - should be BLOCKED
ros2 run s1_brain command_test motion move_forward
# Check logs: "BLOCKED motion - System CRITICAL"

# LED still works (safe command)
ros2 run s1_brain command_test actuation led_on
```

---

## 📊 Telemetry Data Flow

Each subsystem publishes rich telemetry that includes:

**S2 Perception:**
- Pose estimate (x, y, theta)
- P2 interface status (CANopen)
- CPU/Memory load (vision processing)
- Current mode (normal/scanning/calibrating/tracking)

**S3 Motion:**
- Robot position & velocity
- Battery drain simulation
- Motor temperature
- P3 interface status (Ethernet)
- Voltage under load

**S4 Actuation:**
- LED state (on/off)
- Gripper state (open/closed)
- Servo angle
- P4 interface status (LVDS + Optical)
- Buzzer events

---

## 🎛️ Command API Reference

### S1Command Message Structure
```
builtin_interfaces/Time stamp
string target      # "motion" | "perception" | "actuation"
string action      # Command-specific action
string[] params    # Optional parameters
```

### Valid Commands

| Target | Action | Params | Description |
|--------|--------|--------|-------------|
| `perception` | `scan_area` | - | Scan for obstacles |
| `perception` | `track_target` | [target_id] | Track specific object |
| `perception` | `calibrate_sensors` | - | Calibrate cameras |
| `motion` | `move_forward` | - | Drive forward |
| `motion` | `move_back` | - | Drive backward |
| `motion` | `turn_left` | - | Rotate left |
| `motion` | `turn_right` | - | Rotate right |
| `motion` | `stop` | - | Stop all motion |
| `motion` | `set_speed` | [speed] | Set velocity |
| `actuation` | `led_on` | - | Turn LED on |
| `actuation` | `led_off` | - | Turn LED off |
| `actuation` | `beep` | - | Sound buzzer |
| `actuation` | `gripper_open` | - | Open gripper |
| `actuation` | `gripper_close` | - | Close gripper |
| `actuation` | `set_servo` | [angle] | Set servo position |

---

## 🏆 Production Features

✅ **Realistic Simulation**
- Physics-based motion (kinematic model)
- Battery drain proportional to load
- Temperature increases with activity
- Interface load varies with operation

✅ **Full Closed Loop**
- Commands flow: Dashboard → MQTT → S1 → Subsystems
- Telemetry flows: Subsystems → S1 → MQTT → Dashboard
- Real-time state monitoring

✅ **Fault Injection Ready**
- Can simulate interface failures
- Battery depletion simulation
- Temperature spikes
- Perfect for testing diagnostics

✅ **Hackathon Demo Ready**
- Visual feedback (LED on/off)
- Motion visualization (position updates)
- Multi-subsystem coordination
- Safety-enforced command routing

---

## 🛠️ Troubleshooting

**Subsystems not receiving commands?**
```bash
# Check master controller is running
ros2 node list | grep master_controller

# Verify command topics
ros2 topic list | grep cmd

# Echo routed commands
ros2 topic echo /s2/motion/cmd
```

**No telemetry from subsystems?**
```bash
# Check if subsystems are running
ros2 node list | grep -E "s2|s3|s4"

# Verify state topics exist
ros2 topic list | grep state

# Check topic data types
ros2 topic info /s3/motion/state
```

**Commands blocked by safety?**
```bash
# Check system health
ros2 topic echo /s1/diagnostics/report

# View safety logs
ros2 topic echo /s1/logs/raw | grep BLOCKED
```

---

## 🚀 Next: Full Launch System

Create unified launch file to start:
- 3 C++ drivers (P2/P3/P4)
- 6 S1 brain nodes
- 3 subsystems (S2/S3/S4)
- **Total: 12 nodes in one command**

Ready for **Component B**: Unified Launch System

---

**Built with ❤️ for full-stack robot simulation**
