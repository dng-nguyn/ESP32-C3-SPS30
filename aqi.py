import time
import json
import requests
import threading
import os
import sys
import paho.mqtt.client as mqtt
from datetime import datetime
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# ================= CONFIGURATION LOAD =================

DRY_RUN = os.getenv('DRY_RUN', 'False').lower() == 'true'

# --- Notifications ---
DISCORD_WEBHOOK_URL = os.getenv('DISCORD_WEBHOOK_URL', '')

# --- Health Check ---
UPTIME_KUMA_URL = os.getenv('UPTIME_KUMA_URL', '')
HEALTH_CHECK_INTERVAL = int(os.getenv('HEALTH_CHECK_INTERVAL', 300))

# --- AQICN / WAQI Configuration ---
AQICN_TOKEN        = os.getenv('AQICN_TOKEN')
AQICN_STATION_ID   = os.getenv('AQICN_STATION_ID')
AQICN_STATION_NAME = os.getenv('AQICN_STATION_NAME')
AQICN_LAT          = float(os.getenv('AQICN_LAT', 0.0))
AQICN_LON          = float(os.getenv('AQICN_LON', 0.0))
AQICN_ORG_NAME     = os.getenv('AQICN_ORG_NAME')
AQICN_ORG_URL      = os.getenv('AQICN_ORG_URL')

# --- MQTT Configuration ---
MQTT_BROKER     = os.getenv('MQTT_BROKER')
MQTT_PORT       = int(os.getenv('MQTT_PORT', 1883))
MQTT_USER       = os.getenv('MQTT_USER')
MQTT_PASS       = os.getenv('MQTT_PASS')
MQTT_TOPIC_BASE = os.getenv('MQTT_TOPIC_BASE', 'sensor/sps30')

# --- Home Assistant Configuration ---
HA_BASE_URL     = os.getenv('HA_BASE_URL')
HA_TOKEN        = os.getenv('HA_TOKEN')

HA_ENTITIES = {
    "temp":     os.getenv('HA_ENTITY_TEMP'),
    "humidity": os.getenv('HA_ENTITY_HUMIDITY'),
    "pressure": os.getenv('HA_ENTITY_PRESSURE')
}

# ================= STATE FLAGS =================

# We use these to track if we should notify on success (First run only)
first_ha_success = True
first_upload_success = True

# ================= HELPER FUNCTIONS =================

data_buffer = {}
current_samples = 0

def log_msg(message, level="INFO"):
    """
    Prints to console and sends notifications for specific levels.
    Levels: 
      - INFO: Console only
      - ONLINE: Console + Discord (Used for first successes)
      - WARN/ERROR/FATAL: Console + Discord
    """
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    formatted_msg = f"[{timestamp}] [{level}] {message}"
    
    # Print to console
    print(formatted_msg)

    # Levels that trigger a Discord Notification
    notify_levels = ["WARN", "ERROR", "FATAL", "ONLINE"]
    
    if level in notify_levels and DISCORD_WEBHOOK_URL:
        # Add emoji for visual clarity in Discord
        emoji = "⚠️"
        if level == "ONLINE": emoji = "✅"
        if level == "FATAL": emoji = "⛔"
        
        send_discord(f"{emoji} {level}: {message}")

def send_discord(content):
    try:
        data = {"content": content}
        requests.post(DISCORD_WEBHOOK_URL, json=data, timeout=5)
    except Exception as e:
        print(f"[LOGGING ERROR] Failed to send Discord alert: {e}")

def get_iso_time():
    return datetime.now().astimezone().replace(microsecond=0).isoformat()

# ================= CORE LOGIC =================

def get_ha_state(entity_id):
    global first_ha_success
    if not entity_id: return None
        
    url = f"{HA_BASE_URL}/api/states/{entity_id}"
    headers = {"Authorization": f"Bearer {HA_TOKEN}", "content-type": "application/json"}
    
    try:
        resp = requests.get(url, headers=headers, timeout=5)
        if resp.status_code == 200:
            val = resp.json().get("state")
            
            # Notify on FIRST successful connection to HA
            if first_ha_success:
                log_msg(f"Home Assistant connection established.", "ONLINE")
                first_ha_success = False

            try:
                return float(val)
            except (ValueError, TypeError):
                log_msg(f"Entity {entity_id} state is '{val}' (not numeric)", "WARN")
                return None
        else:
            log_msg(f"HTTP {resp.status_code} fetching {entity_id}", "WARN")
    except Exception as e:
        log_msg(f"Connection failed for {entity_id}: {e}", "WARN")
    return None

def upload_payload(payload):
    global first_upload_success
    
    if DRY_RUN:
        log_msg("Dry Run: Payload generated (not sent).", "INFO")
        print(json.dumps(payload, indent=4))
        return

    url = "https://aqicn.org/sensor/upload" 
    
    try:
        headers = {'Content-Type': 'application/json'}
        response = requests.post(url, json=payload, headers=headers, timeout=15)
        
        if response.status_code == 200:
            try:
                resp_json = response.json()
                if resp_json.get("status") == "ok":
                    # NOTIFY on FIRST success, then stay silent
                    if first_upload_success:
                        log_msg("First batch uploaded successfully to AQICN! Future successes will be silent.", "ONLINE")
                        first_upload_success = False
                    else:
                        pass # Silent success
                else:
                    log_msg(f"API Status not OK: {resp_json}", "ERROR")
            except:
                log_msg(f"Uploaded (Raw response): {response.text}", "WARN")
        else:
            log_msg(f"Upload failed HTTP {response.status_code}: {response.text}", "ERROR")
            
    except Exception as e:
        log_msg(f"Upload connection failed: {e}", "ERROR")

def process_and_upload():
    timestamp = get_iso_time()
    readings = []

    # 1. Process MQTT Data
    for specie, stats in data_buffer.items():
        r = {
            "specie": specie,
            "time": timestamp,
            "value": stats.get('value'),
            "unit": "mg/m3",
            "averaging": 60 
        }
        if 'min' in stats:    r['min'] = stats['min']
        if 'max' in stats:    r['max'] = stats['max']
        if 'std' in stats:    r['stddev'] = stats['std']
        if 'median' in stats: r['median'] = stats['median']
        readings.append(r)

    # 2. Process HA Data
    for specie, entity_id in HA_ENTITIES.items():
        if not entity_id: continue

        val = get_ha_state(entity_id)
        if val is not None:
            val = round(val, 2)
            r = { "specie": specie, "time": timestamp, "value": val }
            if specie == "temp": r["unit"] = "C"
            elif specie == "pressure": r["unit"] = "hPa"
            elif specie == "humidity": r["unit"] = "%"
            readings.append(r)
        
    if not readings:
        log_msg("No readings collected to send.", "WARN")
        return

    # 3. Construct Payload
    payload = {
        "token": AQICN_TOKEN,
        "station": {
            "id": AQICN_STATION_ID,
            "name": AQICN_STATION_NAME,
            "latitude": AQICN_LAT,
            "longitude": AQICN_LON
        },
        "org": {
            "website": AQICN_ORG_URL,
            "name": AQICN_ORG_NAME
        },
        "readings": readings
    }
    
    upload_payload(payload)
    data_buffer.clear()

# --- MQTT Logic with Retry ---

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        # Always notify on connect/reconnect so we know if the service restarted
        log_msg("Connected to MQTT Broker!", "ONLINE")
        client.subscribe(f"{MQTT_TOPIC_BASE}/#")
    else:
        log_msg(f"MQTT Connect failed with code {rc}", "ERROR")

def on_disconnect(client, userdata, rc):
    if rc != 0:
        log_msg("Unexpected MQTT disconnection. Auto-reconnecting...", "WARN")

def on_message(client, userdata, msg):
    global current_samples
    topic = msg.topic
    try:
        val = float(msg.payload.decode())
    except ValueError:
        return

    subtopic = topic.replace(f"{MQTT_TOPIC_BASE}/", "")
    parts = subtopic.split('/')
    
    # TRIGGER
    if subtopic == "samples":
        current_samples = int(val)
        process_and_upload()
        return

    # Parse Specie
    raw_specie = parts[0]
    metric = "value"
    if len(parts) > 1:
        metric = parts[1]

    specie_map = { "mc1p0": "pm1.0", "mc2p5": "pm2.5", "mc10p0": "pm10" }

    if raw_specie not in specie_map: return

    aqicn_specie = specie_map[raw_specie]
    if aqicn_specie not in data_buffer:
        data_buffer[aqicn_specie] = {}

    data_buffer[aqicn_specie][metric] = val

def connect_mqtt(client):
    """
    Loops indefinitely until a connection is established.
    Uses incremental backoff: 5s, 10s, 15s... max 60s.
    """
    wait_time = 5
    while True:
        try:
            if not MQTT_BROKER:
                raise ValueError("MQTT_BROKER env var is missing")
                
            log_msg(f"Attempting connection to MQTT {MQTT_BROKER}:{MQTT_PORT}...", "INFO")
            client.connect(MQTT_BROKER, MQTT_PORT, 60)
            break # Success, exit loop
        except Exception as e:
            log_msg(f"MQTT Connection failed: {e}", "WARN")
            log_msg(f"Retrying in {wait_time} seconds...", "INFO")
            time.sleep(wait_time)
            
            # Incremental backoff, cap at 60s
            if wait_time < 60:
                wait_time += 5

# --- Health Check ---
def run_health_check():
    log_msg(f"Background health check started ({HEALTH_CHECK_INTERVAL}s)", "INFO")
    while True:
        try:
            requests.get(UPTIME_KUMA_URL, timeout=10)
        except Exception as e:
            print(f"[HEALTH ERROR] Ping failed: {e}") 
        time.sleep(HEALTH_CHECK_INTERVAL)

# ================= MAIN =================

if __name__ == "__main__":
    # 1. Start Health Check
    if UPTIME_KUMA_URL and "xxx" not in UPTIME_KUMA_URL:
        t = threading.Thread(target=run_health_check)
        t.daemon = True
        t.start()
    
    # 2. Setup MQTT Client
    client = mqtt.Client()
    if MQTT_USER and MQTT_PASS:
        client.username_pw_set(MQTT_USER, MQTT_PASS)

    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect

    log_msg(f"Starting AQICN Bridge (Dry Run: {DRY_RUN})", "INFO")

    # 3. Connect with Retry (Blocking until success)
    connect_mqtt(client)

    # 4. Start Loop
    try:
        client.loop_forever()
    except KeyboardInterrupt:
        log_msg("Stopping script...", "INFO")
    except Exception as e:
        log_msg(f"Fatal Crash: {e}", "FATAL")
        sys.exit(1)