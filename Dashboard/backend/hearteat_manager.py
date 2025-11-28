# heartbeat/heartbeat_manager.py (Modified)
import threading
import time
import json
import paho.mqtt.client as mqtt # Import MQTT client type for reference

class HeartbeatManager:
    def __init__(self, mqtt_client: mqtt.Client, module_name: str, topic_base: str):
        """
        mqtt_client: The connected paho.mqtt.client object.
        module_name: string id of this module (e.g., "robot-alpha").
        topic_base: The base topic to publish to (e.g., "robot/robot-alpha").
        """
        self.mqtt_client = mqtt_client
        self.module_name = module_name
        
        # 💡 Topic is hardcoded to be consistent with the existing robot/robot-alpha/heartbeat
        self.heartbeat_topic = f"{topic_base}/heartbeat" 
        
        self.running = False
        self.thread = None

    def start(self, interval: int = 5):
        """Starts the heartbeat thread. Default interval is 5s to match existing robot_agent."""
        if self.running:
            return
            
        self.running = True
        self.thread = threading.Thread(target=self._run, args=(interval,), daemon=True)
        self.thread.start()
        print(f"[HB] Heartbeat started for {self.module_name} on topic {self.heartbeat_topic} at {interval}s interval.")

    def _run(self, interval):
        while self.running:
            # We use the same payload structure as your existing robot_agent heartbeat for compatibility
            payload = {
                "type": "heartbeat",
                "robot_id": self.module_name,
                "timestamp": datetime.utcnow().isoformat() + "Z", # Need to import datetime
                "uptime_s": int(time.time()),
                "status": "OK", # Simplified status, can be expanded later
                "heartbeat": 1
            }
            
            try:
                self.mqtt_client.publish(self.heartbeat_topic, json.dumps(payload), qos=0)
            except Exception as e:
                print(f"[HB Error] Failed to publish heartbeat: {e}")
                
            time.sleep(interval)

    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join()
            print(f"[HB] Heartbeat for {self.module_name} stopped.")