# robot_agent.py
import os
import time
import json
import random
import psutil
from datetime import datetime
import paho.mqtt.client as mqtt

# -------------------------------------------------------
# Configuration
# -------------------------------------------------------
BROKER = os.getenv("MQTT_BROKER_HOST", "localhost")
ROBOT_ID = os.getenv("ROBOT_ID", "robot-alpha")

HEARTBEAT_INTERVAL = 5
TELEMETRY_INTERVAL = 3

TOPIC_BASE = f"robot/{ROBOT_ID}"

# Robot state
robot_state = {
    "moving": False,
    "direction": None,
    "led": False,
    "estop": False,
    "position": {"x": 0.0, "y": 0.0, "theta": 0.0},
    "battery_pct": 100.0,
    "temperature_c": 30.0,
    "voltage_v": 24.0,
}

# 💡 NEW GLOBAL STATE: Counter for raw heartbeat simulation
raw_beat_counter = 0

# -------------------------------------------------------
# MQTT Setup
# -------------------------------------------------------
client = mqtt.Client()

def mqtt_connect():
    print(f"Connecting to MQTT broker at {BROKER}...")
    try:
        client.connect(BROKER, 1883, 60)
        print("[MQTT] Connected.")
    except Exception as e:
        print("MQTT connection failed, retrying...", e)
        time.sleep(3)
        mqtt_connect()

mqtt_connect()
client.loop_start()

# -------------------------------------------------------
# Helpers
# -------------------------------------------------------
def publish(topic_suffix, payload):
    topic = f"{TOPIC_BASE}/{topic_suffix}"
    client.publish(topic, json.dumps(payload))

def now():
    return datetime.utcnow().isoformat() + "Z"

# 💡 NEW HELPER FUNCTION for interface resource simulation
def rand_resource_pct():
    """Simulates CPU, Memory, and Disk Free percentage for an interface."""
    return {
        # CPU Usage (0-100%) - typically low to medium
        "cpu_percent": round(random.uniform(5.0, 100.0), 1),
        # Memory Usage (0-100%) - typically medium
        "mem_percent": round(random.uniform(20.0, 70.0), 1),
        # Disk Free (0-100%) - critical if low
        # Introduce a chance for low disk space for P4
        "disk_free": round(random.uniform(50.0, 95.0), 1) if random.random() < 0.9 else round(random.uniform(5.0, 25.0), 1)
    }

# -------------------------------------------------------
# Heartbeat + Telemetry
# -------------------------------------------------------
def send_heartbeat():
    global raw_beat_counter
    raw_beat_counter += 1
    
    # 💡 MODIFIED: Include a raw_beat value that changes every interval
    # We use a simple counter + small random fluctuation
    raw_beat_value = raw_beat_counter + random.uniform(-0.5, 0.5)

    payload = {
        "type": "heartbeat",
        "robot_id": ROBOT_ID,
        "timestamp": now(),
        "uptime_s": int(time.time()),
        "status": "OK" if not robot_state["estop"] else "E_STOP",
        "cpu_percent": psutil.cpu_percent(),
        "mem_percent": psutil.virtual_memory().percent,
        # ⚠️ ADDED: The raw beat value for the dashboard to plot a wave
        "raw_beat": raw_beat_value, 
        # numeric heartbeat marker (was used to track latency, but we now use a changing value)
        "heartbeat": 1 
    }
    publish("heartbeat", payload)

def send_telemetry():
    # Update position if moving and not in E-STOP
    if robot_state["moving"] and not robot_state["estop"]:
        delta = 0.1
        if robot_state["direction"] == "forward":
            robot_state["position"]["y"] += delta
        elif robot_state["direction"] == "back":
            robot_state["position"]["y"] -= delta
        elif robot_state["direction"] == "left":
            robot_state["position"]["x"] -= delta
        elif robot_state["direction"] == "right":
            robot_state["position"]["x"] += delta

    # Simulate battery drain and temperature
    robot_state["battery_pct"] = max(0.0, robot_state["battery_pct"] - random.uniform(0.0, 0.2))
    robot_state["temperature_c"] = 30 + random.random() * 7
    robot_state["voltage_v"] = 24 + random.random() * 1.2

    # Simulate interfaces traffic / errors for P2, P3, P4 (per-protocol)
    def rand_iface():
        return {
            "canopen": round(random.uniform(0, 200), 2),  # packets/sec
            "ethernet": round(random.uniform(0, 1000), 2), # kb/s or similar
            "lvds": round(random.uniform(0, 50), 2),
            "optical": round(random.uniform(0, 10), 2)
        }

    interfaces = {
        "P2": rand_iface(),
        "P3": rand_iface(),
        "P4": rand_iface()
    }

    # Optionally include per-protocol "status" fields (UP/DOWN) - simple simulation
    for k, v in interfaces.items():
        v["status"] = "UP" if random.random() > 0.02 else "DOWN"

    # 💡 ADD Interface Resource Status
    interface_status = {
        "p2": rand_resource_pct(),
        "p3": rand_resource_pct(),
        "p4": rand_resource_pct(),
    }
    
    # Flatten interface status keys for advanced.html consumption
    # p2_cpu_percent, p2_mem_percent, p2_disk_free, etc.
    final_interface_status = {}
    for port, data in interface_status.items():
        for key, value in data.items():
            final_interface_status[f"{port}_{key}"] = value

    payload = {
        "type": "telemetry",
        "robot_id": ROBOT_ID,
        "timestamp": now(),
        "battery_pct": round(robot_state["battery_pct"], 1),
        "voltage_v": round(robot_state["voltage_v"], 2),
        "temperature_c": round(robot_state["temperature_c"], 2),
        "position": {
            "x": round(robot_state["position"]["x"], 3),
            "y": round(robot_state["position"]["y"], 3),
            "theta": round(robot_state["position"]["theta"], 3)
        },


        "load": {
            "motor1": round(random.random()*3, 3),
            "motor2": round(random.random()*3, 3)
        },
        "led": robot_state["led"],
        "estop": robot_state["estop"], # 💡 ADDED E-STOP

        # include interfaces object (P2/P3/P4) with per-protocol numbers
        "interfaces": interfaces,
        # 💡 Include the resource usage for P2, P3, P4
        "interface_status": final_interface_status,
        # include small heartbeat marker for telemetry stream (0/1)
        "heartbeat": 0
    }
    publish("telemetry", payload)

# -------------------------------------------------------
# Command Handling
# -------------------------------------------------------
def handle_robot_command(payload):
    action = payload.get("action")

    # Safety: E-STOP
    if payload.get("type") == "safety":
        if action == "E_STOP":
            robot_state["estop"] = True
            robot_state["moving"] = False
            robot_state["direction"] = None
            print("⚠️  EMERGENCY STOP ACTIVATED!")
            publish("log", {
                "type": "log",
                "robot_id": ROBOT_ID,
                "timestamp": now(),
                "level": "CRITICAL",
                "message": "Emergency STOP activated"
            })
        elif action == "CLEAR_ESTOP":
            robot_state["estop"] = False
            print("✅ E-STOP cleared")
            publish("log", {
                "type": "log",
                "robot_id": ROBOT_ID,
                "timestamp": now(),
                "level": "INFO",
                "message": "E-STOP cleared"
            })
        return

    # Move / Stop commands
    if action == "move" and not robot_state["estop"]:
        direction = payload.get("direction")
        robot_state["moving"] = True
        robot_state["direction"] = direction
        print(f"[COMMAND] Moving {direction}")
        publish("log", {
            "type": "log",
            "robot_id": ROBOT_ID,
            "timestamp": now(),
            "level": "INFO",
            "message": f"Moving {direction}"
        })
    elif action == "stop":
        robot_state["moving"] = False
        robot_state["direction"] = None
        print("[COMMAND] Stopping")
        publish("log", {
            "type": "log",
            "robot_id": ROBOT_ID,
            "timestamp": now(),
            "level": "INFO",
            "message": "Stopped"
        })
    elif action == "led/on":
        robot_state["led"] = True
        print("[COMMAND] LED ON")
        publish("log", {
            "type": "log",
            "robot_id": ROBOT_ID,
            "timestamp": now(),
            "level": "INFO",
            "message": "LED turned ON"
        })
    elif action == "led/off":
        robot_state["led"] = False
        print("[COMMAND] LED OFF")
        publish("log", {
            "type": "log",
            "robot_id": ROBOT_ID,
            "timestamp": now(),
            "level": "INFO",
            "message": "LED turned OFF"
        })

def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())
        print(f"[CMD] Received: {payload}")
        handle_robot_command(payload)
    except Exception as e:
        print("[ERROR] Bad message:", e)

client.on_message = on_message
client.subscribe(f"{TOPIC_BASE}/cmd/#")

# -------------------------------------------------------
# Main Loop
# -------------------------------------------------------
if __name__ == "__main__":
    last_hb = 0
    last_tel = 0

    print("🚀 Robot agent started.")

    try:
        while True:
            now_t = time.time()

            if now_t - last_hb >= HEARTBEAT_INTERVAL:
                send_heartbeat()
                last_hb = now_t

            if now_t - last_tel >= TELEMETRY_INTERVAL:
                send_telemetry()
                last_tel = now_t

            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\n🛑 Robot agent stopped manually.")
        client.loop_stop()
        client.disconnect()