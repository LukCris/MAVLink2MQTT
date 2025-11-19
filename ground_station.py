# gcs_station.py
import time, uuid, threading, os, json, sys, queue, itertools
from typing import Dict, Any, Optional
import paho.mqtt.client as mqtt
from datetime import datetime, timezone

from metrics_logger import (
    init_link_logger, log_link_event,
    init_latency_logger, record_latency_request,
    record_latency_response, record_latency_timeout
)

try:
    import readline  # su Linux/Unix è disponibile; su Windows potrebbe mancare
except Exception:
    readline = None

BROKER_HOST = "10.42.0.1"
BROKER_PORT = 8883
USE_TLS = True
UAV_ID = "uav1"
TOPIC_CMD = f"uav/{UAV_ID}/cmd"
TOPIC_STATUS = f"uav/{UAV_ID}/status"
TOPIC_BATT = f"uav/{UAV_ID}/telemetry/battery"
TOPIC_ACK = f"uav/{UAV_ID}/ack"
TOPIC_LAT_REQ = f"uav/{UAV_ID}/latency/request"
TOPIC_LAT_RES = f"uav/{UAV_ID}/latency/response"
QOS_CMD = 1

CERT_CA = "./certs/ca.crt"
CERT_FILE = "./certs/client.crt"
KEY_FILE = "./certs/client.key"

_pending = {}  # command_id -> event/result

q: "queue.Queue[str]" = queue.Queue()
last_batt = {"voltage": None, "current": None, "level": None}

LOG_DIR = os.environ.get("ML2MQTT_LOG_DIR", "./logs")
os.makedirs(LOG_DIR, exist_ok=True)

# ---------------------------
# Metrics
# ---------------------------
PING_INTERVAL = 1.0  # secondi
PING_TIMEOUT = 2.0  # quanto aspetti la risposta

latency_counter = itertools.count(1)
_pending_pings = {}  # msg_id -> t_send_ns

init_link_logger()
init_latency_logger()


def latency_ping_loop(client: mqtt.Client, stop_event: threading.Event):
    while not stop_event.is_set():
        msg_id = next(latency_counter)
        t_send_ns = time.time_ns()
        payload = json.dumps({"id": msg_id, "t_send_ns": t_send_ns})

        # registra la richiesta in sospeso
        _pending_pings[msg_id] = t_send_ns

        # log lato "request"
        record_latency_request(msg_id, t_send_ns)

        client.publish(TOPIC_LAT_REQ, payload, qos=0, retain=False)
        time.sleep(PING_INTERVAL)


def on_latency_response(client, userdata, msg):
    data = json.loads(msg.payload.decode())
    msg_id = data["id"]
    t_recv_ns = time.time_ns()

    t_send_ns = _pending_pings.pop(msg_id, None)
    if t_send_ns is None:
        # risposta di un ping non registrato o già scaduto
        return

    # log RTT
    record_latency_response(msg_id, t_recv_ns, t_send_ns)


def latency_timeout_loop(stop_event: threading.Event):
    while not stop_event.is_set():
        now_ns = time.time_ns()
        to_delete = []
        for msg_id, t_send_ns in list(_pending_pings.items()):
            dt_s = (now_ns - t_send_ns) / 1e9
            if dt_s > PING_TIMEOUT:
                # logga come perso
                record_latency_timeout(msg_id, t_send_ns)
                to_delete.append(msg_id)
        for msg_id in to_delete:
            _pending_pings.pop(msg_id, None)
        time.sleep(0.5)


# ---------------------------
# Utils
# ---------------------------
def _now_utc_iso():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def safe_print(line: str, prompt: str = "sitl> "):
    """
    Stampa una riga senza spezzare ciò che l'utente sta digitando.
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


def safe_printer(prompt: str = "sitl> "):
    while True:
        msg = q.get()
        if msg is None:
            break
        safe_print(msg, prompt=prompt)


def qos_retriever(args):
    global QOS_CMD
    qos = None
    if "-q" in args:
        idx = args.index("-q")
        try:
            level = int(args[idx + 1])
        except (IndexError, ValueError):
            print("[ERROR] Parameter -q require an integer in 0, 1 or 2")
            return None

        if level not in (0, 1, 2):
            print("[ERROR] QoS must be 0, 1 o 2")
            return None

        if level != QOS_CMD:
            qos = level
        else:
            qos = None
            print(f"[UAV] QoS is actually set to {QOS_CMD}")
    return qos


# ---------------------------
# MQTT callbacks
# ---------------------------
def _on_connect(c, u, f, rc):
    print(f"[MQTT] connected rc={rc}")
    c.subscribe(TOPIC_ACK, qos=1)
    c.subscribe(TOPIC_STATUS, qos=1)

    c.message_callback_add(TOPIC_STATUS, _on_status)

    c.message_callback_add(TOPIC_BATT, _on_batt)
    c.subscribe(TOPIC_BATT, qos=1)
    c.subscribe(TOPIC_LAT_RES, qos=1)


def _on_status(c, u, msg):
    global last_status
    try:
        payload = msg.payload.decode("utf-8")

        # metti in coda per stampa non distruttiva
        q.put(f"[UAV] UAV Status: {payload}")
    except Exception as e:
        print(f"[ERROR] {e}")


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
        q.put(f"[BATT] voltage={v}V  current={crr}A  level={lvl}%")
    except Exception as e:
        print(f"[ERROR] {e}")


def _on_message(c, u, msg):
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        cid = payload.get("command_id")
        if cid and cid in _pending:
            _pending[cid] = payload

        topic = TOPIC_CMD
        payload_bytes = len(payload)
        topic_bytes = len(topic.encode())
        log_link_event("rx", topic, payload_bytes + topic_bytes)
    except Exception as e:
        print("[ACK] parse error:", e)


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
    mqttc.message_callback_add(TOPIC_LAT_RES, on_latency_response)
    mqttc.connect(BROKER_HOST, BROKER_PORT, keepalive=30)
    mqttc.loop_start()
    # Thread che stampa la batteria senza rompere l'input
    t = threading.Thread(target=safe_printer, args=("sitl> ",), daemon=True)
    t.start()
    return mqttc


def publish_cmd(client: mqtt.Client, payload: Dict[str, Any], await_ack: bool = True, timeout: float = 5.0,
                qos=QOS_CMD) -> Optional[
    Dict[str, Any]]:
    cid = payload.setdefault("command_id", f"cmd-{uuid.uuid4().hex[:8]}")
    data = json.dumps(payload, separators=(",", ":"))
    print(qos)
    if qos is None:
        qos = QOS_CMD

    # normalizza e valida QoS
    try:
        qos = int(qos)
    except (TypeError, ValueError):
        print(f"[ERROR] Invalid QoS value: {qos}")
        return None

    if qos not in (0, 1, 2):
        print(f"[ERROR] QoS must be 0, 1 or 2, got {qos}")
        return None

    payload_bytes = len(payload) if isinstance(payload, (bytes, bytearray)) else len(str(payload).encode())
    topic_bytes = len(TOPIC_CMD.encode())
    log_link_event("tx", TOPIC_CMD, payload_bytes + topic_bytes)

    info = client.publish(TOPIC_CMD, data, qos=qos, retain=False)
    info.wait_for_publish()
    print(f"[GCS→MQTT] {TOPIC_CMD} {data}")

    if not await_ack:
        return None

    _pending[cid] = None
    t0 = time.time()
    while time.time() - t0 < timeout:
        if _pending[cid] is not None:
            res = _pending.pop(cid)
            return res
        time.sleep(0.05)
    res = _pending.pop(cid)
    return res  # None se timeout


HELP = """
Comandi disponibili:

SYSTEM / MODE
  guided [-q 0|1|2]                  # mode GUIDED
  arm [-q 0|1|2]                     # arma (in GUIDED)
  disarm [-q 0|1|2]
  takeoff <alt_m> [-q 0|1|2]
  land [-q 0|1|2]                    # mode LAND
  rtl  [-q 0|1|2]                    # mode RTL

MOVE (NED)
  move <dir> <speed_mps> <distance_m> [-q 0|1|2]   # dir: north|south|east|west|up|down
  vel <vx> <vy> <vz> <duration_s> [-q 0|1|2]
  yaw <heading_deg> [rate_deg_s=30] [relative=0|1] [cw=1|0] [-q 0|1|2]
  setspeed <mps> [-q 0|1|2]
  goto <lat> <lon> <alt_m> [-q 0|1|2]

PARAM / STATUS
  batt [-q 0|1|2]
"""


# ---------------------------
# CLI
# ---------------------------
def cli(c):
    global QOS_CMD
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
                print(qos_level)
                qos_level = qos_retriever(parts[1:])
                publish_cmd(c, {"type": "mode", "mode": "GUIDED"}, qos=qos_level)
                print("[UAV] Mode GUIDED")
            elif cmd == "arm":
                qos_level = qos_retriever(parts[1:])
                publish_cmd(c, {"type": "arm", "arm": True}, qos=qos_level)
                print("[UAV] ARMED")
            elif cmd == "disarm":
                qos_level = qos_retriever(parts[1:])
                publish_cmd(c, {"type": "arm", "arm": False}, qos=qos_level)
                print("[UAV] DISARMED")
            elif cmd == "takeoff":
                qos_level = qos_retriever(parts[1:])
                alt = float(parts[1])
                info = publish_cmd(c, {"type": "takeoff", "alt": alt}, timeout=25, qos=qos_level)
                print("[UAV] Takeoff")
            elif cmd == "move":
                qos_level = qos_retriever(parts[1:])
                dir = str(parts[1])
                speed = float(parts[2])
                dist = float(parts[3])
                publish_cmd(c, {"type": "move", "direction": dir, "speed": speed, "distance": dist}, qos=qos_level)
                print("[UAV] Move set")
            elif cmd == "vel":
                qos_level = qos_retriever(parts[1:])
                vx, vy, vz, dur = map(float, parts[1:5])
                publish_cmd(c, {"type": "velocity", "vx": vx, "vy": vy, "vz": vz, "duration": dur}, qos=qos_level)
                print("[UAV] Velocity set")
            elif cmd == "yaw":
                args = parts[1:]

                # Parse opzionale -q <n> ovunque dopo il comando
                if "-q" in args:
                    idx = args.index("-q")
                    try:
                        qos_level = int(args[idx + 1])
                    except (IndexError, ValueError):
                        print("[ERROR] yaw: missing or invalid QoS after -q")
                        continue
                    # rimuovi '-q' e il valore corrispondente dalla lista argomenti
                    del args[idx:idx + 2]

                if not args:
                    print("[ERROR] yaw: heading required")
                    continue

                try:
                    heading = float(args[0])
                    rate = float(args[1]) if len(args) > 1 else 30.0
                    relative = bool(int(args[2])) if len(args) > 2 else False
                    cw = bool(int(args[3])) if len(args) > 3 else True
                except ValueError as e:
                    print(f"[ERROR] yaw: invalid argument: {e}")
                    continue

                publish_cmd(
                    c,
                    {
                        "type": "yaw",
                        "heading": heading,
                        "rate": rate,
                        "relative": relative,
                        "cw": cw,
                    },
                    qos=qos_level,
                )
                print("[UAV] Yaw set")
            elif cmd == "setspeed":
                qos_level = qos_retriever(parts[1:])
                speed = float(parts[1])
                publish_cmd(c, {"type": "setspeed", "speed": speed}, qos=qos_level)
                print(f"[UAV] Set speed: {speed}")
            elif cmd == "goto":
                qos_level = qos_retriever(parts[1:])
                lat = float(parts[1])
                lon = float(parts[2])
                alt = float(parts[3])
                publish_cmd(c, {"type": "goto", "lat": lat, "lon": lon, "alt": alt}, qos=qos_level)
            elif cmd == "rtl":
                qos_level = qos_retriever(parts[1:])
                info = publish_cmd(c, {"type": "mode", "mode": "RTL"}, qos=qos_level)
                mode = info.get("mode")
                print(f"[UAV] Mode {mode}")
            elif cmd == "land":
                qos_level = qos_retriever(parts[1:])
                info = publish_cmd(c, {"type": "mode", "mode": "LAND"}, qos=qos_level)
                mode = info.get("mode")
                print(f"[UAV] Mode {mode}")
            elif cmd == "batt":
                qos_level = qos_retriever(parts[1:])
                info = publish_cmd(c, {"type": "batt"}, qos=qos_level)

                if not info:
                    print("[UAV] No battery info received (timeout or error)")
                    continue

                volt = info.get("voltage")
                curr = info.get("current")
                lev = info.get("level")
                print(f"[UAV] Battery voltage: {volt}, current: {curr}, level: {lev}")
            elif cmd == "qos":
                try:
                    level = int(parts[1])
                except (IndexError, ValueError):
                    print("[ERROR] A QoS value must be specified (0, 1 or 2)")
                    continue

                if level not in (0, 1, 2):
                    print("[ERROR] QOS must be 0, 1 or 2")
                    continue

                if QOS_CMD == level:
                    print(f"[UAV] QoS is actually set to {level}")
                else:
                    QOS_CMD = level
                    print(f"[UAV] QoS is now set to {QOS_CMD}")

        except Exception as e:
            print(f"[ERROR] {e}")


if __name__ == "__main__":
    c = make_client()
    stop_event = threading.Event()
    t_ping = threading.Thread(target=latency_ping_loop, args=(c, stop_event), daemon=True)
    t_ping.start()
    try:
        cli(c)
    finally:
        stop_event.set()