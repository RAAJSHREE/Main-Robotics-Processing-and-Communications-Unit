from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request, HTTPException
from fastapi.responses import FileResponse
from fastapi.middleware.cors import CORSMiddleware
from typing import Dict, Any, List
import asyncio
import json
import os
from pathlib import Path
import databases
import sqlalchemy
from sqlalchemy import text  # <-- REQUIRED FOR WAL PATCH
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

# ------------------- WAL MODE PATCH (Fix DB Locked Errors) -------------------
try:
    with engine.connect() as conn:
        conn.execute(text("PRAGMA journal_mode=WAL;"))
        conn.execute(text("PRAGMA synchronous=NORMAL;"))
    print("📌 SQLite WAL mode enabled successfully.")
except Exception as e:
    print(f"⚠ Could not enable WAL mode: {e}")
# ------------------------------------------------------------------------------

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
mqtt_host = "localhost"
mqtt_client = mqtt.Client()
clients: List[WebSocket] = []  # connected websocket clients

ROBOT_STATES: Dict[str, Any] = {}
ROBOT_ID = os.getenv("ROBOT_ID", "robot-alpha")
HEARTBEAT_TIMEOUT = 18
loop = asyncio.get_event_loop()

# ------------------- MQTT Callbacks -------------------

def on_connect(client, userdata, flags, rc):
    print(f"✅ MQTT connected with result code {rc}")
    client.subscribe("robot/+/#") 

def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())
    except:
        payload = {"raw": msg.payload.decode(), "topic": msg.topic}
    
    asyncio.run_coroutine_threadsafe(handle_mqtt_message(payload), loop)

async def safe_send(ws: WebSocket, payload):
    try:
        await ws.send_json(payload) 
    except:
        if ws in clients:
            clients.remove(ws)

async def handle_mqtt_message(payload: Dict[str, Any]):
    robot_id = payload.get("robot_id", ROBOT_ID)
    timestamp = payload.get("timestamp", datetime.utcnow().isoformat() + "Z")
    log_type = payload.get("type", "mqtt")

    if robot_id not in ROBOT_STATES:
        ROBOT_STATES[robot_id] = {
            "last_hb_ts": time.time(),
            "status": payload.get("status", "OK"),
            "offline_alert_ts": 0
        }

    if log_type == "heartbeat":
        ROBOT_STATES[robot_id]["last_hb_ts"] = time.time()
        ROBOT_STATES[robot_id].pop("offline_alert_ts", None)

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

    for ws in clients.copy():
        await safe_send(ws, payload)

# ------------------- Watchdog -------------------
async def watchdog_check():
    print("🐕 Watchdog started.")
    while True:
        await asyncio.sleep(HEARTBEAT_TIMEOUT / 3)
        now = time.time()
        
        for robot_id, state in list(ROBOT_STATES.items()):
            last_hb_t = state.get("last_hb_ts")
            if last_hb_t and now - last_hb_t > HEARTBEAT_TIMEOUT:
                msg = {
                    "type": "watchdog",
                    "robot_id": robot_id,
                    "timestamp": datetime.utcnow().isoformat() + "Z",
                    "level": "CRITICAL",
                    "message": f"Watchdog timeout: Robot {robot_id} has missed heartbeats.",
                    "status": "OFFLINE"
                }
                
                if state.get("offline_alert_ts", 0) < now - HEARTBEAT_TIMEOUT:
                    print(f"🚨 WATCHDOG: {msg['message']}")
                    await database.execute(
                        logs_table.insert().values(
                            robot_id=robot_id,
                            ts=msg["timestamp"],
                            type="watchdog",
                            payload=msg
                        )
                    )
                    for ws in clients.copy():
                        await safe_send(ws, msg)
                    state["offline_alert_ts"] = now
            else:
                state.pop("offline_alert_ts", None)

# ------------------- Startup -------------------
@app.on_event("startup")
async def startup():
    await database.connect()
    
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
        mqtt_client.loop_start()
    except Exception as e:
        print(f"❌ MQTT connect error: {e}")

    print("🚀 FastAPI server started.")
    asyncio.create_task(watchdog_check())

@app.on_event("shutdown")
async def shutdown():
    await database.disconnect()
    mqtt_client.loop_stop()
    print("🛑 FastAPI stopped.")

# ------------------- Static Files -------------------
BASE_DIR = Path(__file__).resolve().parent.parent  
STATIC_DIR = BASE_DIR / "dashboard" / "static"

@app.get("/")
async def dashboard(request: Request):
    return FileResponse(str(STATIC_DIR / "index.html"))

@app.get("/advanced")
async def advanced(request: Request):
    return FileResponse(str(STATIC_DIR / "advanced.html"))

# ------------------- API -------------------
@app.post("/api/command")
async def command_post(req: Request):
    payload = await req.json()
    robot_id = payload.get("robot_id")

    if not robot_id:
        raise HTTPException(400, "Missing robot_id")

    mqtt_client.publish(f"robot/{robot_id}/cmd", json.dumps(payload))

    for ws in clients.copy():
        asyncio.create_task(safe_send(ws, payload))

    return {"status": "sent", "payload": payload}

@app.get("/api/logs")
async def get_logs(robot_id: str = ROBOT_ID, limit: int = 50):
    query = (
        logs_table.select()
        .where(logs_table.c.robot_id == robot_id)
        .order_by(logs_table.c.id.desc())
        .limit(limit)
    )
    rows = await database.fetch_all(query)
    return {"logs": [dict(r) for r in rows]}

@app.websocket("/ws/telemetry")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    clients.append(ws)
    print(f"🌐 WebSocket client connected. Total: {len(clients)}")

    try:
        while True:
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        if ws in clients:
            clients.remove(ws)
        print(f"🌐 WebSocket client disconnected. Total: {len(clients)}")
