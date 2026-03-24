# drone_mqtt.py
import base64, json, threading, time, uuid, logging
from typing import Any, Dict
import paho.mqtt.client as mqtt
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

from metrics_logger import init_battery_logger, log_battery

TAKE_OFF = False
LANDING = False
ALT = None

# --- Config MQTT ---
BROKER_HOST = "10.42.0.1"
BROKER_PORT = 8883
USE_TLS = True
UAV_ID = "uav1"
CLIENT_ID = f"drone-bridge-{uuid.uuid4()}"

CERT_CA = "./certs/ca.crt"
CERT_FILE = "./certs/client.crt"
KEY_FILE = "./certs/client.key"

TOPIC_CMD = f"uav/{UAV_ID}/cmd"
TOPIC_ACK = f"uav/{UAV_ID}/ack"
TOPIC_STATUS = f"uav/{UAV_ID}/status"
TOPIC_TEL_ALT = f"uav/{UAV_ID}/telemetry/altitude"
TOPIC_TEL_BAT = f"uav/{UAV_ID}/telemetry/battery"
TOPIC_TEL_SYS = f"uav/{UAV_ID}/telemetry/sys"
TOPIC_HEARTBEAT = f"uav/{UAV_ID}/heartbeat"

_last_heartbeat_time = time.time()
_emergency_landing_active = False

# opzionale pass-through grezzo
TOPIC_MAV_TX = f"uav/{UAV_ID}/mav/tx"  # GCS→DRONE (raw base64)
TOPIC_MAV_RX = f"uav/{UAV_ID}/mav/rx"  # DRONE→GCS (raw base64)

HEARTBEAT_TIMEOUT = 10.0  # secondi prima di attivare il failsafe
QOS_CMD = 1
QOS_TEL = 0


# --- Config SITL (il DRONE si connette al SITL) ---
SITL_LINK = "127.0.0.1:14550"
vehicle = None
mqtt_client = None
running = True

# ---------------------------
# Metrics
# ---------------------------
_last_batt_timestamp = None
_mah_consumed = 0.0  # mAh totali integrati


def setup_battery_logging(vehicle):
    init_battery_logger()

    @vehicle.on_message('SYS_STATUS')
    def listener(self, name, msg):
        voltage_V = msg.voltage_battery / 1000.0 if msg.voltage_battery != 0xFFFF else None
        current_A = msg.current_battery / 100.0 if msg.current_battery != 0xFFFF else None
        remaining = msg.battery_remaining if msg.battery_remaining != 0xFF else None
        log_battery(voltage_V, current_A, remaining_pct=remaining)


log_ap = logging.getLogger("autopilot")


def setup_statustext_forward(vehicle):
    """
    Inoltra i messaggi STATUSTEXT dell'autopilota via MQTT
    e li logga localmente col logger 'autopilot'.
    """

    @vehicle.on_message('STATUSTEXT')
    def listener(self, name, msg):
        global mqtt_client

        # msg.text in DroneKit è una stringa o una sequenza di char
        try:
            text = msg.text
            # se contiene padding NUL, ripulisci
            if isinstance(text, bytes):
                text = text.decode("utf-8", errors="ignore")
            text = text.strip("\x00")
        except Exception:
            text = "<invalid text>"

        # Severità in chiaro, se disponibile
        severity_map = {
            mavutil.mavlink.MAV_SEVERITY_EMERGENCY: "EMERGENCY",
            mavutil.mavlink.MAV_SEVERITY_ALERT: "ALERT",
            mavutil.mavlink.MAV_SEVERITY_CRITICAL: "CRITICAL",
            mavutil.mavlink.MAV_SEVERITY_ERROR: "ERROR",
            mavutil.mavlink.MAV_SEVERITY_WARNING: "WARNING",
            mavutil.mavlink.MAV_SEVERITY_NOTICE: "NOTICE",
            mavutil.mavlink.MAV_SEVERITY_INFO: "INFO",
            mavutil.mavlink.MAV_SEVERITY_DEBUG: "DEBUG",
        }
        sev = severity_map.get(msg.severity, f"SEV_{msg.severity}")

        # Log locale sul drone (come quelli che già vedi)
        log_ap.warning("%s: %s", sev, text)

        # Inoltra via MQTT alla GCS
        if mqtt_client is not None:
            try:
                payload = {
                    "severity": sev,
                    "text": text,
                }
                mqtt_client.publish(
                    TOPIC_TEL_SYS,
                    json.dumps(payload),
                    qos=QOS_TEL
                )
            except Exception as e:
                print("[SYS] MQTT publish error:", e)


# ---------------------------
# Helpers MAVLink / DroneKit
# ---------------------------
def _ack(command_id: str | None, ok: bool, **extra):
    payload = {"ok": ok}
    if command_id: payload["command_id"] = command_id
    payload.update(extra)
    mqtt_client.publish(TOPIC_ACK, json.dumps(payload), qos=1, retain=False)


def set_mode(mode: str):
    global LANDING

    if mode.upper() in ("LAND", "RTL"):
        LANDING = True
    else:
        LANDING = False

    vehicle.mode = VehicleMode(mode)
    t0 = time.time()
    while time.time() - t0 < 6:
        if vehicle.mode.name.upper() == mode.upper():
            return True
        time.sleep(0.1)
    return False


def arm_disarm(arm: bool):
    vehicle.armed = arm
    t0 = time.time()
    while time.time() - t0 < 8:
        if bool(vehicle.armed) == arm:
            return True
        time.sleep(0.2)
    return False


def takeoff(alt: float):
    global TAKE_OFF
    global ALT
    ALT = alt
    if vehicle.mode.name.upper() != "GUIDED":
        if not set_mode("GUIDED"):
            return False
    if not vehicle.armed:
        if not arm_disarm(True):
            return False
    vehicle.simple_takeoff(alt)
    TAKE_OFF = True
    return True


def send_velocity_ned(vx: float, vy: float, vz: float, duration: float):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )
    t0 = time.time()
    while time.time() - t0 < duration:
        vehicle.send_mavlink(msg)
        time.sleep(0.2)
    return True


def move_direction_by_distance(direction: str, speed: float, distance: float):
    if speed <= 0: raise ValueError("speed must be > 0")
    dur = distance / speed
    d = direction.lower()
    vx = vy = vz = 0.0
    if d == "north":
        vx = speed
    elif d == "south":
        vx = -speed
    elif d == "east":
        vy = speed
    elif d == "west":
        vy = -speed
    elif d == "up":
        vz = -speed  # NED
    elif d == "down":
        vz = speed
    else:
        raise ValueError("direction must be north/south/east/west/up/down")
    return send_velocity_ned(vx, vy, vz, dur)


def set_yaw(heading: float, rate: float = 30.0, relative: bool = False, cw: bool = True):
    is_relative = 1 if relative else 0
    direction = 1 if cw else -1
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        heading, rate, direction, is_relative,
        0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()
    return True


def set_speed(spd: float):
    vehicle.airspeed = spd
    vehicle.groundspeed = spd
    return True


def goto_absolute(lat: float, lon: float, alt: float):
    tgt = LocationGlobalRelative(lat, lon, alt)
    vehicle.simple_goto(tgt)
    return True


def set_param(name: str, value: Any):
    vehicle.parameters[name] = value
    return True


def get_param(name: str):
    return vehicle.parameters.get(name, None)


# ---------------------------
# MQTT callbacks
# ---------------------------
def _on_heartbeat(client, userdata, msg):
    """Aggiorna il timestamp dell'ultimo heartbeat ricevuto dalla GCS."""
    global _last_heartbeat_time, _emergency_landing_active
    _last_heartbeat_time = time.time()
    _emergency_landing_active = False  # se la GCS torna online, resetta il flag


def heartbeat_watchdog():
    """
    Monitora il heartbeat della GCS.
    Se non viene ricevuto alcun messaggio entro HEARTBEAT_TIMEOUT secondi
    e il drone è armato, attiva l'atterraggio di emergenza.
    """
    global _last_heartbeat_time, _emergency_landing_active
    # Attendi qualche secondo all'avvio per dare tempo alla GCS di connettersi
    time.sleep(15)
    while running:
        elapsed = time.time() - _last_heartbeat_time
        if elapsed > HEARTBEAT_TIMEOUT and not _emergency_landing_active:
            if vehicle is not None and vehicle.armed:
                print(f"[WARNING] GCS heartbeat lost ({elapsed:.1f}s). "
                      f"Activating emergency landing.")
                try:
                    set_mode("LAND")
                    _emergency_landing_active = True
                except Exception as e:
                    print(f"[WARNING] Failed to set LAND mode: {e}")
                # Resetta il timer per evitare chiamate ripetute
                _last_heartbeat_time = time.time()
        time.sleep(1.0)


def on_connect(client, userdata, flags, rc):
    print(f"[MQTT] Connected rc={rc} host={BROKER_HOST} port={BROKER_PORT}")
    client.publish(TOPIC_STATUS, "online", qos=1, retain=True)
    client.subscribe(TOPIC_CMD, qos=QOS_CMD)
    client.subscribe(TOPIC_HEARTBEAT, qos=0)
    #client.message_callback_add(TOPIC_HEARTBEAT, _on_heartbeat) # Attivarlo al passaggio al drone reale
    client.subscribe(TOPIC_MAV_TX, qos=0)


def on_disconnect(client, userdata, rc):
    print(f"[MQTT] Disconnected rc={rc}")


def on_message(client, userdata, msg):
    print(f"[MQTT] RX topic={msg.topic} payload={msg.payload[:120]!r}")

    if msg.topic == TOPIC_MAV_TX:
        try:
            raw = base64.b64decode(msg.payload)
            vehicle._master.write(raw)
        except Exception as e:
            print("[RAW] write failed:", e)
        return

    # comandi JSON
    try:
        p = json.loads(msg.payload.decode("utf-8"))
        cid = p.get("command_id")
        ok, detail = handle_command(p)
        _ack(cid, ok, **detail)
    except Exception as e:
        print("[CMD] error:", e)


def handle_command(p: Dict[str, Any]):
    t = str(p.get("type", "")).lower()
    try:
        if t == "mode":
            ok = set_mode(str(p["mode"]))
            mode = p['mode']
            return ok, {"mode": mode}
        if t == "arm":
            ok = arm_disarm(bool(p["arm"]))
            return ok, {"detail": "armed" if p["arm"] else "disarmed"}
        if t == "takeoff":
            ok = takeoff(float(p["alt"]))
            return ok, {"detail": f"takeoff alt={p['alt']}"}
        if t == "move":
            ok = move_direction_by_distance(str(p["direction"]), float(p["speed"]), float(p["distance"]))
            return ok, {"detail": "move completed"}
        if t == "velocity":
            ok = send_velocity_ned(float(p["vx"]), float(p["vy"]), float(p["vz"]), float(p.get("duration", 1.0)))
            return ok, {"detail": "velocity sent"}
        if t == "yaw":
            ok = set_yaw(float(p["heading"]), float(p.get("rate", 30.0)), bool(p.get("relative", False)),
                         bool(p.get("cw", True)))
            return ok, {"detail": "yaw set"}
        if t == "setspeed":
            ok = set_speed(float(p["speed"]))
            speed = float(p["speed"])
            return ok, {"speed": speed}
        if t == "goto":
            ok = goto_absolute(float(p["lat"]), float(p["lon"]), float(p["alt"]))
            return ok, {"detail": "goto sent"}
        if t == "param_set":
            ok = set_param(str(p["name"]), p["value"])
            return ok, {"detail": f"param_set {p['name']}={p['value']}"}
        if t == "get":
            val = get_param(str(p["name"]))
            return True, {"name": p["name"], "value": val}
        if t == "batt":
            bat = getattr(vehicle, "battery")
            volt = bat.voltage
            curr = bat.current
            lev = bat.level
            print(volt, curr, lev)
            return True, {"voltage": volt, "current": curr, "level": lev}
        return False, {"error": "unknown command"}
    except Exception as e:
        return False, {"error": str(e)}


# ---------------------------
# Telemetria
# ---------------------------
def telemetry_loop():
    global TAKE_OFF, LANDING
    last_bat_log = 0.0  # snapshot batteria ogni 15s
    BAT_LOG_EVERY = 15.0

    while running and vehicle is not None:
        try:
            bat = vehicle.battery
            now = time.time()

            if TAKE_OFF:
                alt = vehicle.location.global_relative_frame.alt or 0.0
                if alt <= 0.95 * ALT:
                    mqtt_client.publish(
                        TOPIC_TEL_ALT,
                        json.dumps({"altitude": alt}),
                        qos=QOS_TEL
                    )
                else:
                    TAKE_OFF = False

            if LANDING:
                alt = vehicle.location.global_relative_frame.alt or 0.0
                if alt >= 1.0:
                    mqtt_client.publish(
                        TOPIC_TEL_ALT,
                        json.dumps({"altitude": alt}),
                        qos=QOS_TEL
                    )
                else:
                    LANDING = False

            # Batteria ogni 10s
            if bat and (now - last_bat_log >= BAT_LOG_EVERY):
                mqtt_client.publish(
                    TOPIC_TEL_BAT,
                    json.dumps({"voltage": bat.voltage, "current": bat.current, "level": bat.level}),
                    qos=QOS_TEL,
                )
                last_bat_log = now

            time.sleep(0.5)
        except Exception as e:
            print("[TEL] error:", e)
            time.sleep(1.0)


# ---------------------------
# Main
# ---------------------------
def main():
    global vehicle, mqtt_client, running
    print(f"[UAV] Connecting SITL {SITL_LINK} ...")
    vehicle = connect(SITL_LINK, wait_ready=True, timeout=60)
    print("[UAV] Vehicle connected.")
    setup_battery_logging(vehicle)
    setup_statustext_forward(vehicle)

    mqtt_client = mqtt.Client(client_id=CLIENT_ID, clean_session=True)
    mqtt_client.will_set(TOPIC_STATUS, "offline", qos=1, retain=True)
    mqtt_client.on_connect = on_connect
    mqtt_client.on_disconnect = on_disconnect
    mqtt_client.on_message = on_message

    if USE_TLS:
        mqtt_client.tls_set(
            ca_certs=CERT_CA,
            certfile=CERT_FILE,
            keyfile=KEY_FILE,
        )
        pass

    mqtt_client.connect(BROKER_HOST, BROKER_PORT, keepalive=30)
    mqtt_client.loop_start()

    threading.Thread(target=telemetry_loop, daemon=True).start()
    #threading.Thread(target=heartbeat_watchdog, daemon=True).start() # Attivarlo al passaggio al drone reale

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        try:
            mqtt_client.publish(TOPIC_STATUS, "offline", qos=1, retain=True)
        except:
            pass
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        try:
            vehicle.close()
        except:
            pass


if __name__ == "__main__":
    main()