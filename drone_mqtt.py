# drone_mqtt_bridge.py
from __future__ import annotations
import base64, json, threading, time, uuid, os, logging, logging.handlers
from typing import Any, Dict
import paho.mqtt.client as mqtt
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from datetime import datetime, timezone

# --- Config MQTT ---
BROKER_HOST = "10.42.0.1"
BROKER_PORT = 8883
USE_TLS = True
UAV_ID = "uav1"
CLIENT_ID = f"drone-bridge-{uuid.uuid4()}"

CERT_CA = "/etc/mosquitto/ca_certificates/ca.crt"
CERT_FILE = "/etc/mosquitto/certs/drone.crt"
KEY_FILE = "/etc/mosquitto/certs/drone.key"

TOPIC_CMD = f"uav/{UAV_ID}/cmd"
TOPIC_ACK = f"uav/{UAV_ID}/ack"
TOPIC_STATUS = f"uav/{UAV_ID}/status"
TOPIC_TEL_HB = f"uav/{UAV_ID}/telemetry/heartbeat"
TOPIC_TEL_POS = f"uav/{UAV_ID}/telemetry/global_position"
TOPIC_TEL_ATT = f"uav/{UAV_ID}/telemetry/attitude"
TOPIC_TEL_BAT = f"uav/{UAV_ID}/telemetry/battery"

# opzionale pass-through grezzo
TOPIC_MAV_TX = f"uav/{UAV_ID}/mav/tx"  # GCS→DRONE (raw base64)
TOPIC_MAV_RX = f"uav/{UAV_ID}/mav/rx"  # DRONE→GCS (raw base64)

QOS_CMD = 1
QOS_TEL = 0

# --- Config SITL (il DRONE si connette al SITL) ---
SITL_LINK = "127.0.0.1:14550"  # o "udp:127.0.0.1:14550"
vehicle = None
mqtt_client = None
running = True

# --- Config logging ---
LOG_DIR = os.environ.get("ML2MQTT_LOG_DIR", "./logs")
os.makedirs(LOG_DIR, exist_ok=True)
NODE = "drone"


# ---------------------------
# Logging
# ---------------------------
def _now_utc_iso():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")

class JsonFormatter(logging.Formatter):
    def format(self, record: logging.LogRecord) -> str:
        payload = {
            "ts": _now_utc_iso(),
            "lvl": record.levelname,
            "logger": record.name,
            "msg": record.getMessage(),
        }
        # porta dentro i campi passati via extra=
        for k, v in getattr(record, "__dict__", {}).items():
            if k in ("msg","args","levelname","levelno","pathname","filename","module",
                     "exc_info","exc_text","stack_info","lineno","funcName","created",
                     "msecs","relativeCreated","thread","threadName","processName",
                     "process","name"):
                continue
            try:
                json.dumps(v)
                payload[k] = v
            except Exception:
                payload[k] = str(v)
        return json.dumps(payload, ensure_ascii=False)

def make_logger(name: str, filename: str) -> logging.Logger:
    logger = logging.getLogger(name)
    logger.setLevel(logging.INFO)
    fh = logging.handlers.TimedRotatingFileHandler(
        os.path.join(LOG_DIR, filename), when="midnight", backupCount=7, utc=True, encoding="utf-8"
    )
    fmt = JsonFormatter()
    fh.setFormatter(fmt)
    logger.handlers.clear()
    logger.addHandler(fh)
    logger.propagate = False
    return logger

def log_json(logger: logging.Logger, event: str, **fields):
    logger.info(event, extra=fields)

LOGGER = make_logger("drone", "drone.log")



# ---------------------------
# Helpers MAVLink / DroneKit
# ---------------------------
def _ack(command_id: str | None, ok: bool, **extra):
    payload = {"ok": ok}
    if command_id: payload["command_id"] = command_id
    payload.update(extra)
    mqtt_client.publish(TOPIC_ACK, json.dumps(payload), qos=1, retain=False)


def set_mode(mode: str):
    log_json(LOGGER, "action.set_mode", node=NODE, mode=mode)
    vehicle.mode = VehicleMode(mode)
    t0 = time.time()
    while time.time() - t0 < 6:
        if vehicle.mode.name.upper() == mode.upper():
            log_json(LOGGER, "action.set_mode_ok", node=NODE, mode=mode)
            return True
        time.sleep(0.1)
    log_json(LOGGER, "action.set_mode_timeout", node=NODE, mode=mode)
    return False


def arm_disarm(arm: bool):
    log_json(LOGGER, "action.arm_disarm", node=NODE, arm=arm)
    vehicle.armed = arm
    t0 = time.time()
    while time.time() - t0 < 8:
        if bool(vehicle.armed) == arm:
            log_json(LOGGER, "action.arm_disarm_ok", node=NODE, arm=arm)
            return True
        time.sleep(0.2)
    log_json(LOGGER, "action.arm_disarm_timeout", node=NODE, arm=arm)
    return False


def takeoff(alt: float):
    log_json(LOGGER, "action.takeoff", node=NODE, alt=alt)
    if vehicle.mode.name.upper() != "GUIDED":
        if not set_mode("GUIDED"):
            log_json(LOGGER, "action.takeoff_fail_mode", node=NODE)
            return False
    if not vehicle.armed:
        if not arm_disarm(True):
            log_json(LOGGER, "action.takeoff_fail_arm", node=NODE)
            return False
    vehicle.simple_takeoff(alt)
    while True:
        a = vehicle.location.global_relative_frame.alt or 0.0
        if a >= 0.95 * alt: break
        time.sleep(0.5)
    log_json(LOGGER, "action.takeoff_ok", node=NODE)
    return True


def send_velocity_ned(vx: float, vy: float, vz: float, duration: float):
    log_json(LOGGER, "action.velocity", node=NODE)
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
    log_json(LOGGER, "action.velocity_ok", node=NODE)
    return True


def move_direction_by_distance(direction: str, speed: float, distance: float):
    log_json(LOGGER, "action.move_direction", node=NODE)
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
        log_json(LOGGER, "action.move_direction_fail_comm", node=NODE)
        raise ValueError("direction must be north/south/east/west/up/down")
    log_json(LOGGER, "action.move_direction_ok", node=NODE)
    return send_velocity_ned(vx, vy, vz, dur)


def set_yaw(heading: float, rate: float = 30.0, relative: bool = False, cw: bool = True):
    log_json(LOGGER, "action.set_yaw", node=NODE)
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
    log_json(LOGGER, "action.set_yaw_ok", node=NODE)
    return True


def set_speed(spd: float):
    log_json(LOGGER, "action.set_speed", node=NODE)
    vehicle.airspeed = spd
    vehicle.groundspeed = spd
    log_json(LOGGER, "action.set_speed_ok", node=NODE)
    return True


def goto_absolute(lat: float, lon: float, alt: float):
    log_json(LOGGER, "action.goto", node=NODE)
    tgt = LocationGlobalRelative(lat, lon, alt)
    vehicle.simple_goto(tgt)
    log_json(LOGGER, "action.goto_ok", node=NODE)
    return True


def set_param(name: str, value: Any):
    vehicle.parameters[name] = value
    return True


def get_param(name: str):
    return vehicle.parameters.get(name, None)


# ---------------------------
# MQTT callbacks
# ---------------------------
def on_connect(client, userdata, flags, rc):
    print(f"[MQTT] Connected rc={rc} host={BROKER_HOST} port={BROKER_PORT}")
    log_json(LOGGER, "mqtt.connect", node=NODE, rc=rc, host=BROKER_HOST, port=BROKER_PORT, tls=USE_TLS)
    client.publish(TOPIC_STATUS, "online", qos=1, retain=True)
    r1 = client.subscribe(TOPIC_CMD, qos=QOS_CMD)
    log_json(LOGGER, "mqtt.subscribe", node=NODE, topic=TOPIC_CMD, result=getattr(r1[0], 'name', r1[0]), mid=r1[1])
    r2 = client.subscribe(TOPIC_MAV_TX, qos=0)
    log_json(LOGGER, "mqtt.subscribe", node=NODE, topic=TOPIC_MAV_TX, result=getattr(r2[0], 'name', r2[0]), mid=r2[1])


def on_disconnect(client, userdata, rc):
    print(f"[MQTT] Disconnected rc={rc}")
    log_json(LOGGER, "mqtt.disconnect", node=NODE, rc=rc)


def on_message(client, userdata, msg):
    print(f"[MQTT] RX topic={msg.topic} payload={msg.payload[:120]!r}")
    log_json(LOGGER, "mqtt.rx", node=NODE, topic=msg.topic,
             payload=msg.payload[:500].decode("utf-8", "ignore"))
    if msg.topic == TOPIC_MAV_TX:
        try:
            raw = base64.b64decode(msg.payload)
            vehicle._master.write(raw)
            log_json(LOGGER, "mav.raw_applied", node=NODE, size=len(raw))
        except Exception as e:
            print("[RAW] write failed:", e)
            log_json(LOGGER, "mav.raw_error", node=NODE, error=str(e))
        return
    # comandi JSON
    try:
        p = json.loads(msg.payload.decode("utf-8"))
        cid = p.get("command_id")
        log_json(LOGGER, "cmd.receive", node=NODE, command_id=cid, body=p)
        ok, detail = handle_command(p)
        _ack(cid, ok, **detail)
        log_json(LOGGER, "cmd.done", node=NODE, command_id=cid, ok=ok, **detail)
    except Exception as e:
        print("[CMD] error:", e)
        log_json(LOGGER, "cmd.error", node=NODE, error=str(e))


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
    last_hb_log = 0.0          # snapshot stato (mode/armed) ogni 15s
    last_bat_log = 0.0         # snapshot batteria ogni 15s
    HB_LOG_EVERY  = 15.0
    BAT_LOG_EVERY = 15.0

    while running and vehicle is not None:
        try:
            # Heartbeat
            hb = {
                "armed": bool(vehicle.armed),
                "mode": vehicle.mode.name if vehicle.mode else None,
                "sys_status": getattr(vehicle.system_status, "state", None),
            }
            # Pubblicazione MQTT (frequente)
            mqtt_client.publish(TOPIC_TEL_HB, json.dumps(hb), qos=QOS_TEL)

            # Posizione, assetto, batteria su MQTT
            loc = vehicle.location.global_relative_frame
            if loc:
                mqtt_client.publish(
                    TOPIC_TEL_POS,
                    json.dumps({"lat": loc.lat, "lon": loc.lon, "alt": loc.alt}),
                    qos=QOS_TEL,
                )

            att = vehicle.attitude
            if att:
                mqtt_client.publish(
                    TOPIC_TEL_ATT,
                    json.dumps({"roll": att.roll, "pitch": att.pitch, "yaw": att.yaw}),
                    qos=QOS_TEL,
                )

            bat = vehicle.battery

            # --- Logging su file ---
            now = time.time()

            # Stato base ogni 5s
            if now - last_hb_log >= HB_LOG_EVERY:
                log_json(LOGGER, "tel.hb", node=NODE, **hb)
                last_hb_log = now

            # **Batteria ogni 10s**
            if bat and (now - last_bat_log >= BAT_LOG_EVERY):
                mqtt_client.publish(
                    TOPIC_TEL_BAT,
                    json.dumps({"voltage": bat.voltage, "current": bat.current, "level": bat.level}),
                    qos=QOS_TEL,
                )

                # Valori possono essere None: normalizza a tipi serializzabili
                v = None if getattr(bat, "voltage", None) is None else float(bat.voltage)
                c = None if getattr(bat, "current", None) is None else float(bat.current)
                l = None if getattr(bat, "level",   None) is None else int(bat.level)
                log_json(LOGGER, "tel.battery", node=NODE, voltage=v, current=c, level=l)
                last_bat_log = now

            time.sleep(0.5)  # frequenza pubblicazione MQTT

        except Exception as e:
            print("[TEL] error:", e)
            log_json(LOGGER, "tel.error", node=NODE, error=str(e))
            time.sleep(1.0)



# ---------------------------
# Main
# ---------------------------
def main():
    global vehicle, mqtt_client, running
    print(f"[DRONE] Connecting SITL {SITL_LINK} ...")
    vehicle = connect(SITL_LINK, wait_ready=True, timeout=60)
    print("[DRONE] Vehicle connected.")
    log_json(LOGGER, "sitl.connect", node=NODE, link=SITL_LINK)
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
    try:
        while True: time.sleep(0.5)
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