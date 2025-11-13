# gcs_mqtt_client.py
import time, uuid, threading, os, json, logging, logging.handlers, sys, queue
from typing import Dict, Any, Optional
import paho.mqtt.client as mqtt
from datetime import datetime, timezone

try:
    import readline  # su Linux/Unix è disponibile; su Windows potrebbe mancare
except Exception:
    readline = None

BROKER_HOST = "10.42.0.1"
BROKER_PORT = 8883
USE_TLS = True
UAV_ID = "uav1"
TOPIC_CMD = f"uav/{UAV_ID}/cmd"
TOPIC_BATT = f"uav/{UAV_ID}/telemetry/battery"  # QUA VENGONO PUBBLICATE LE INFO SULLA BATTERIA DAL DRONE
TOPIC_ACK = f"uav/{UAV_ID}/ack"
QOS_CMD = 1

CERT_CA = "./certs/ca.crt"
CERT_FILE = "./certs/client.crt"
KEY_FILE = "./certs/client.key"

_pending = {}  # command_id -> event/result

batt_q: "queue.Queue[str]" = queue.Queue()
last_batt = {"voltage": None, "current": None, "level": None}

LOG_DIR = os.environ.get("ML2MQTT_LOG_DIR", "./logs")
os.makedirs(LOG_DIR, exist_ok=True)

# ---------------------------
# Utils
# ---------------------------
def _now_utc_iso():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")

def safe_print(line: str, prompt: str = "sitl> "):
    """
    Stampa una riga senza spezzare ciò che l'utente sta digitando.
    Funziona meglio se è presente 'readline'.
    """
    if readline:
        buf = readline.get_line_buffer()
        # Vai a inizio riga e cancellala
        sys.stdout.write("\r")
        sys.stdout.write(" " * (len(prompt) + len(buf)))
        sys.stdout.write("\r")
        # Stampa il messaggio
        print(line)
        # Ripristina il prompt + il buffer corrente
        sys.stdout.write(prompt + buf)
        sys.stdout.flush()
    else:
        # Fallback: stampa normale (potrebbe spezzare la riga)
        print(line)

def batt_printer(prompt: str = "sitl> "):
    while True:
        msg = batt_q.get()
        if msg is None:
            break
        safe_print(msg, prompt=prompt)


# ---------------------------
# Logging
# ---------------------------
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
            if k in ("msg", "args", "levelname", "levelno", "pathname", "filename", "module",
                     "exc_info", "exc_text", "stack_info", "lineno", "funcName", "created",
                     "msecs", "relativeCreated", "thread", "threadName", "processName",
                     "process", "name"):
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


LOGGER = make_logger("gcs", "gcs.log")
NODE = "gcs"

# ---------------------------
# MQTT callbacks
# ---------------------------
def _on_connect(c, u, f, rc):
    print(f"[MQTT] connected rc={rc}")
    log_json(LOGGER, "mqtt.connect", node=NODE, rc=rc, host=BROKER_HOST, port=BROKER_PORT, tls=USE_TLS)
    r = c.subscribe(TOPIC_ACK, qos=1)
    log_json(LOGGER, "mqtt.subscribe", node=NODE, topic=TOPIC_ACK, result=getattr(r[0], 'name', r[0]), mid=r[1])

    # Sottoscrizione batteria con callback dedicato
    c.message_callback_add(TOPIC_BATT, _on_batt)
    r2 = c.subscribe(TOPIC_BATT, qos=1)
    log_json(LOGGER, "mqtt.subscribe", node=NODE, topic=TOPIC_BATT, result=getattr(r2[0], 'name', r2[0]), mid=r2[1])

def _on_batt(c, u, msg):
    global last_batt
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        # normalizza
        v = payload.get("voltage")
        crr = payload.get("current")
        lvl = payload.get("level")
        last_batt = {"voltage": v, "current": crr, "level": lvl}
        # metti in coda per stampa non distruttiva
        batt_q.put(f"[BATT] voltage={v}V  current={crr}A  level={lvl}%")
        # log su file (no stampa diretta qui)
        log_json(LOGGER, "tel.battery.rx", node=NODE, voltage=v, current=crr, level=lvl)
    except Exception as e:
        log_json(LOGGER, "tel.battery.parse_error", node=NODE, error=str(e))


def _on_message(c, u, msg):
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        cid = payload.get("command_id")
        log_json(LOGGER, "ack.receive", node=NODE, topic=msg.topic, command_id=cid,
                 ok=payload.get("ok"), detail=payload.get("detail"),
                 error=payload.get("error"), mode=payload.get("mode"),
                 value=payload.get("value"))
        # name=payload.get("name") Per ora l'ho tolto
        if cid and cid in _pending:
            _pending[cid] = payload
    except Exception as e:
        print("[ACK] parse error:", e)
        log_json(LOGGER, "ack.parse_error", node=NODE, error=str(e),
                 raw=msg.payload[:300].decode("utf-8", "ignore"))


def make_client() -> mqtt.Client:
    mqttc = mqtt.Client(client_id=f"gcs-{uuid.uuid4()}", clean_session=True)
    mqttc.on_connect = _on_connect
    mqttc.on_message = _on_message
    if USE_TLS:
        mqttc.tls_set(
            ca_certs=CERT_CA,
            certfile=CERT_FILE,
            keyfile=KEY_FILE,
        )
    mqttc.connect(BROKER_HOST, BROKER_PORT, keepalive=30)
    log_json(LOGGER, "mqtt.connecting", node=NODE, host=BROKER_HOST, port=BROKER_PORT, tls=USE_TLS)
    mqttc.loop_start()
    # Thread che stampa la batteria senza rompere l'input
    t = threading.Thread(target=batt_printer, args=("sitl> ",), daemon=True)
    t.start()
    return mqttc


def publish_cmd(client: mqtt.Client, payload: Dict[str, Any], await_ack: bool = True, timeout: float = 5.0,
                qos=QOS_CMD) -> Optional[
    Dict[str, Any]]:
    cid = payload.setdefault("command_id", f"cmd-{uuid.uuid4().hex[:8]}")
    data = json.dumps(payload, separators=(",", ":"))

    if qos == None:
        qos = QOS_CMD

    log_json(LOGGER, "cmd.publish", node=NODE, topic=TOPIC_CMD, command_id=cid, body=payload)
    info = client.publish(TOPIC_CMD, data, qos=qos, retain=False)
    info.wait_for_publish()
    log_json(LOGGER, "cmd.published", node=NODE, topic=TOPIC_CMD, command_id=cid, mid=getattr(info, "mid", None))
    print(f"[GCS→MQTT] {TOPIC_CMD} {data}")
    if not await_ack:
        return None
    _pending[cid] = None
    t0 = time.time()
    while time.time() - t0 < timeout:
        if _pending[cid] is not None:
            res = _pending.pop(cid)
            log_json(LOGGER, "cmd.ack_received", node=NODE, command_id=cid, ack=res)
            return res
        time.sleep(0.05)
    res = _pending.pop(cid)
    log_json(LOGGER, "cmd.ack_timeout", node=NODE, command_id=cid)
    return res  # None se timeout


HELP = """
Comandi disponibili:

SYSTEM / MODE
  guided                      # mode GUIDED
  arm                         # arma (in GUIDED)
  disarm
  takeoff <alt_m>
  land                        # mode LAND
  rtl                         # mode RTL

MOVE (NED)
  move <dir> <speed_mps> <distance_m>    # dir: north|south|east|west|up|down
  vel <vx> <vy> <vz> <duration_s>
  yaw <heading_deg> [rate_deg_s=30] [relative=0|1] [cw=1|0]
  setspeed <mps>
  goto <lat> <lon> <alt_m>

PARAM / STATUS
  batt
"""

# ---------------------------
# CLI
# ---------------------------
def cli(c):
    print("=== ArduPilot SITL Interactive CLI (MQTT raw enabled) ===")
    print(HELP)

    while True:
        try:
            raw = input("sitl> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n[EXIT]")
            break

        if not raw:
            continue
        parts = raw.split()
        cmd = parts[0].lower()
        qos_level = None

        try:
            if cmd == "help":
                print(HELP)
            elif cmd == "guided":
                qos = parts[1] if len(parts) > 2 else None
                if qos and int(parts[2]) != QOS_CMD:
                    qos_level = int(parts[2])
                publish_cmd(c, {"type": "mode", "mode": "GUIDED"}, qos=qos_level)
                print("[UAV] Mode GUIDED")
            elif cmd == "arm":
                qos = parts[1] if len(parts) > 2 else None
                if qos and int(parts[2]) != QOS_CMD:
                    qos_level = int(parts[2])
                publish_cmd(c, {"type": "arm", "arm": True}, qos=qos_level)
                print("[UAV] ARMED")
            elif cmd == "disarm":
                qos = parts[1] if len(parts) > 2 else None
                if qos and int(parts[2]) != QOS_CMD:
                    qos_level = int(parts[2])
                publish_cmd(c, {"type": "arm", "arm": False}, qos=qos_level)
                print("[UAV] DISARMED")
            elif cmd == "takeoff":
                qos = parts[2] if len(parts) > 3 else None
                if qos and int(parts[3]) != QOS_CMD:
                    qos_level = int(parts[3])
                alt = float(parts[1])
                info = publish_cmd(c, {"type": "takeoff", "alt": alt}, timeout=25, qos=qos_level)
                print("[UAV] Starting takeoff")
            elif cmd == "move":
                qos = parts[4] if len(parts) > 5 else None
                if qos and int(parts[5]) != QOS_CMD:
                    qos_level = int(parts[5])
                dir = str(parts[1])
                speed = float(parts[2])
                dist = float(parts[3])
                publish_cmd(c, {"type": "move", "direction": dir, "speed": speed, "distance": dist}, qos=qos_level)
                print("[UAV] Move set")
            elif cmd == "vel":
                qos = parts[5] if len(parts) > 6 else None
                if qos and int(parts[6]) != QOS_CMD:
                    qos_level = int(parts[6])
                vx, vy, vz, dur = map(float, parts[1:5])
                publish_cmd(c, {"type": "velocity", "vx": vx, "vy": vy, "vz": vz, "duration": dur}, qos=qos_level)
                print("[UAV] Velocity set")
            elif cmd == "yaw":
                # COMANDO PROBLEMATICO PER LA PARTE DEL QOS
                heading = float(parts[1])
                rate = float(parts[2]) if len(parts) > 2 else 30.0
                relative = bool(int(parts[3])) if len(parts) > 3 else False
                cw = bool(int(parts[4])) if len(parts) > 4 else True
                publish_cmd(c, {"type": "yaw", "heading": heading, "rate": rate, "relative": relative, "cw": cw})
                print("[UAV] Yaw set")
            elif cmd == "setspeed":
                qos = parts[2] if len(parts) > 3 else None
                if qos and int(parts[3]) != QOS_CMD:
                    qos_level = int(parts[3])
                speed = float(parts[1])
                publish_cmd(c, {"type": "setspeed", "speed": speed}, qos=qos_level)
                print(f"[UAV] Set speed: {speed}")
            elif cmd == "goto":
                qos = parts[4] if len(parts) > 5 else None
                if qos and int(parts[5]) != QOS_CMD:
                    qos_level = int(parts[5])
                lat = float(parts[1])
                lon = float(parts[2])
                alt = float(parts[3])
                publish_cmd(c, {"type": "goto", "lat": lat, "lon": lon, "alt": alt}, qos=qos_level)
            elif cmd == "rtl":
                qos = parts[1] if len(parts) > 2 else None
                if qos and int(parts[2]) != QOS_CMD:
                    qos_level = int(parts[2])
                info = publish_cmd(c, {"type": "mode", "mode": "RTL"}, qos=qos_level)
                mode = info.get("mode")
                print(f"[UAV] Mode {mode}")
            elif cmd == "land":
                qos = parts[1] if len(parts) > 2 else None
                if qos and int(parts[2]) != QOS_CMD:
                    qos_level = int(parts[2])
                info = publish_cmd(c, {"type": "mode", "mode": "LAND"}, qos=qos_level)
                mode = info.get("mode")
                print(f"[UAV] Mode {mode}")
            elif cmd == "batt":
                qos = parts[1] if len(parts) > 2 else None
                print(qos)
                if qos and int(parts[2]) != QOS_CMD:
                    qos_level = int(parts[2])
                info = publish_cmd(c, {"type": "batt"}, qos=qos_level)
                volt = info.get("voltage")
                curr = info.get("current")
                lev = info.get("level")
                print(f"[UAV] Battery voltage: {volt}, current: {curr}, level: {lev}")
            elif cmd == "qos":
                level = parts[1]
                if QOS_CMD == level:
                    print(f"[UAV] QoS is actually set to {level}")
                else:
                    qos_level = level

        except Exception as e:
            print(f"[ERROR] {e}")


if __name__ == "__main__":
    c = make_client()
    cli(c)