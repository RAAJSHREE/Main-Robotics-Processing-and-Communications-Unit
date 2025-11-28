from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request, HTTPException
from fastapi.responses import FileResponse
from fastapi.middleware.cors import CORSMiddleware
from typing import Dict, Any, List
import asyncio
import json
import os
import databases
import sqlalchemy
import paho.mqtt.client as mqtt
import time
from datetime import datetime

# ------------------- Database -------------------
DATABASE_URL = "sqlite:///./robot_data.db"
database = databases.Database(DATABASE_URL)
metadata = sqlalchemy.MetaData()

logs_table = sqlalchemy.Table(
    "logs", metadata,
        sqlalchemy.Column("id", sqlalchemy.Integer, primary_key=True),
        sqlalchemy.Column("robot_id", sqlalchemy.String, index=True),
        sqlalchemy.Column("ts", sqlalchemy.String),
        sqlalchemy.Column("type", sqlalchemy.String),
        sqlalchemy.Column("payload", sqlalchemy.JSON)
)

engine = sqlalchemy.create_engine(DATABASE_URL, connect_args={"check_same_thread": False})
metadata.create_all(engine)

# ------------------- App -------------------
app = FastAPI()

# ------------------- CORS -------------------
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"]
)

# ------------------- MQTT & Watchdog Config -------------------
mqtt_host = os.getenv("MQTT_BROKER_HOST", "localhost")
mqtt_client = mqtt.Client()
clients: List[WebSocket] = []  # connected websocket clients

# Watchdog Global State
ROBOT_STATES: Dict[str, Any] = {}
ROBOT_ID = os.getenv("ROBOT_ID", "robot-alpha") # Default ID
# Safety margin in seconds (e.g., 3 * 5s HB interval)
HEARTBEAT_TIMEOUT = 18 

# Get the main asyncio event loop
loop = asyncio.get_event_loop()

# ------------------- MQTT Callbacks -------------------

def on_connect(client, userdata, flags, rc):
    print(f"✅ MQTT connected with result code {rc}")
    # Subscribes to general topics for all robots
    client.subscribe("robot/+/#") 

def on_message(client, userdata, msg):
    """Handles incoming MQTT messages: logs to DB, updates state, broadcasts to WS."""
    try:
        payload = json.loads(msg.payload.decode())
    except:
        payload = {"raw": msg.payload.decode(), "topic": msg.topic}
    
    # Execute processing on the main event loop
    asyncio.run_coroutine_threadsafe(handle_mqtt_message(payload), loop)

async def safe_send(ws: WebSocket, payload):
    """Safely send JSON data over WebSocket and remove disconnected clients."""
    try:
        await ws.send_json(payload) 
    except:
        if ws in clients:
            clients.remove(ws)

async def handle_mqtt_message(payload: Dict[str, Any]):
    """Stores data to DB and broadcasts to WebSockets."""
    robot_id = payload.get("robot_id", ROBOT_ID)
    timestamp = payload.get("timestamp", datetime.utcnow().isoformat() + "Z")
    log_type = payload.get("type", "mqtt")

    # 1. Update ROBOT_STATES on Heartbeat or Telemetry (for initialization)
    if robot_id not in ROBOT_STATES:
        # Initialize state for any new robot ID found in a message
        ROBOT_STATES[robot_id] = {
            "last_hb_ts": time.time(),
            "status": payload.get("status", "OK"),
            "offline_alert_ts": 0 # Initialize alert timestamp
        }

    if log_type == "heartbeat" and robot_id:
        # CRITICAL: Only update the last_hb_ts on a dedicated HEARTBEAT message
        ROBOT_STATES[robot_id]["last_hb_ts"] = time.time()
        # Reset the status/offline flag when a heartbeat is received
        ROBOT_STATES[robot_id].pop("offline_alert_ts", None) 

    # 2. Store to Database
    try:
        await database.execute(
            logs_table.insert().values(
                robot_id=robot_id,
                ts=timestamp,
                type=log_type,
                payload=payload
            )
        )
    except Exception as e:
        print(f"❌ DB Write Error: {e}")

    # 3. Broadcast to websocket clients
    # This forwards the 'heartbeat' message which the JS client needs to plot latency.
    for ws in clients.copy():
        await safe_send(ws, payload)

# ------------------- Watchdog Task -------------------
async def watchdog_check():
    """Periodically checks the last heartbeat time for all tracked robots."""
    print("🐕 Watchdog started.")
    while True:
        # Check frequently, but less than the timeout
        await asyncio.sleep(HEARTBEAT_TIMEOUT / 3) 
        now = time.time()
        
        for robot_id, state in list(ROBOT_STATES.items()):
            # Check if the 'last_hb_ts' key exists before using it
            last_hb_t = state.get("last_hb_ts")
            if last_hb_t is None:
                continue # Skip if state isn't fully initialized

            if now - last_hb_t > HEARTBEAT_TIMEOUT:
                # Robot is offline! Log and notify
                offline_message = {
                    "type": "watchdog",
                    "robot_id": robot_id,
                    "timestamp": datetime.utcnow().isoformat() + "Z",
                    "level": "CRITICAL",
                    "message": f"Watchdog timeout: Robot {robot_id} has missed heartbeats.",
                    "status": "OFFLINE"
                }
                
                # Check if this alert has already been logged/sent recently (to prevent spam)
                if state.get("offline_alert_ts", 0) < now - HEARTBEAT_TIMEOUT:
                    print(f"🚨 WATCHDOG: {offline_message['message']}")

                    # Store critical log to DB
                    await database.execute(
                        logs_table.insert().values(
                            robot_id=robot_id,
                            ts=offline_message["timestamp"],
                            type="watchdog",
                            payload=offline_message
                        )
                    )
                    
                    # Broadcast alert to all clients
                    for ws in clients.copy():
                        await safe_send(ws, offline_message)
                        
                    # Update state to prevent spamming logs/WS
                    ROBOT_STATES[robot_id]["offline_alert_ts"] = now
            else:
                # If robot is online, ensure alert timestamp is cleared
                ROBOT_STATES[robot_id].pop("offline_alert_ts", None)


# ------------------- Startup / Shutdown -------------------
@app.on_event("startup")
async def startup():
    await database.connect()
    
    # Initialize the state for the default robot ID immediately
    if ROBOT_ID not in ROBOT_STATES:
        ROBOT_STATES[ROBOT_ID] = {
            "last_hb_ts": time.time(),
            "status": "INIT",
            "offline_alert_ts": 0
        }
    
    try:
        mqtt_client.on_connect = on_connect
        mqtt_client.on_message = on_message
        mqtt_client.connect(mqtt_host, 1883, 60)
        mqtt_client.loop_start() # Start MQTT network loop in a background thread
    except Exception as e:
        print(f"❌ Failed to connect to MQTT broker: {e}")

    print("🚀 FastAPI server started, database connected.")
    # Start the watchdog task
    asyncio.create_task(watchdog_check())

@app.on_event("shutdown")
async def shutdown():
    await database.disconnect()
    mqtt_client.loop_stop()
    print("🛑 FastAPI server stopped, database disconnected.")

# ------------------- Dashboard HTML -------------------
# Assuming your index.html and advanced.html are in a 'static' directory
@app.get("/")
async def dashboard(request: Request):
    return FileResponse("static/index.html")

@app.get("/advanced")
async def advanced(request: Request):
    return FileResponse("static/advanced.html") 

# ------------------- API Endpoints -------------------
@app.post("/api/command")
async def command_post(req: Request):
    payload = await req.json()
    robot_id = payload.get("robot_id")
    
    if not robot_id:
        raise HTTPException(status_code=400, detail="Missing robot_id required")

    # publish to MQTT so robot reacts
    topic = f"robot/{robot_id}/cmd"
    mqtt_client.publish(topic, json.dumps(payload))

    # broadcast command to websocket clients for immediate UI feedback
    for ws in clients.copy():
        asyncio.create_task(safe_send(ws, payload))

    return {"status":"sent", "payload": payload}

# API endpoint to retrieve historical logs
@app.get("/api/logs")
async def get_logs(robot_id: str = ROBOT_ID, limit: int = 50):
    try:
        # Select logs for the specified robot, ordered by id descending (most recent first), up to the limit
        query = (
            logs_table.select()
            .where(logs_table.c.robot_id == robot_id)
            .order_by(logs_table.c.id.desc())
            .limit(limit)
        )
        
        results = await database.fetch_all(query)
        
        # Convert results to a list of dicts for JSON serialization
        return {"logs": [dict(r) for r in results]}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to fetch logs: {e}")

# ------------------- WebSocket -------------------
@app.websocket("/ws/telemetry")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    clients.append(ws)
    print(f"🌐 WebSocket client connected. Total: {len(clients)}")
    try:
        while True:
            # We listen for any incoming messages to keep the connection alive
            # Alternatively, you could receive commands from the web client here.
            await ws.receive_text()
    except WebSocketDisconnect:
        if ws in clients:
            clients.remove(ws)
        print(f"🌐 WebSocket client disconnected. Total: {len(clients)}")