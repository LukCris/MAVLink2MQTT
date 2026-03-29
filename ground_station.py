# gcs_station.py
import time, uuid, threading, os, json, sys, queue, logging
from typing import Dict, Any, Optional
import paho.mqtt.client as mqtt
from datetime import datetime, timezone

from metrics_logger import (
    init_latency_logger, record_latency_request,
    record_latency_response, record_latency_timeout,
    init_completion_logger, record_completion_request, record_completion_response  # ← nuovo
)

try:
    import readline
except Exception:
    readline = None

BROKER_HOST = "10.42.0.1"
BROKER_PORT = 8883
USE_TLS = True
UAV_ID = "uav1"

CERT_CA = "./certs/ca.crt"
CERT_FILE = "./certs/client.crt"
KEY_FILE = "./certs/client.key"

TOPIC_CMD = f"uav/{UAV_ID}/cmd"
TOPIC_STATUS = f"uav/{UAV_ID}/status"
TOPIC_BATT = f"uav/{UAV_ID}/telemetry/battery"
TOPIC_ALT = f"uav/{UAV_ID}/telemetry/altitude"
TOPIC_ACK = f"uav/{UAV_ID}/ack"
TOPIC_COMPLETED = f"uav/{UAV_ID}/completed"
TOPIC_TEL_SYS = f"uav/{UAV_ID}/telemetry/sys"
TOPIC_HEARTBEAT = f"uav/{UAV_ID}/heartbeat"
HEARTBEAT_INTERVAL = 3.0  # secondi
QOS_CMD = 1

_pending_completion: Dict[str, int] = {}
_pending: Dict[str, Optional[Dict[str, Any]]] = {}  # command_id -> risposta (o None in attesa)
_pending_lat: Dict[str, int] = {}  # command_id -> t_send_ns (per latenza)

q: "queue.Queue[str]" = queue.Queue()
last_batt = {"voltage": None, "current": None, "level": None}

LOG_DIR = os.environ.get("ML2MQTT_LOG_DIR", "./logs")
os.makedirs(LOG_DIR, exist_ok=True)

log_ap = logging.getLogger("autopilot")
log_ap.setLevel(logging.WARNING)

init_latency_logger()
init_completion_logger()


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
    c.subscribe(TOPIC_ACK, qos=QOS_CMD)
    c.subscribe(TOPIC_STATUS, qos=1)

    c.message_callback_add(TOPIC_STATUS, _on_status)

    c.subscribe(TOPIC_ALT, qos=1)
    c.message_callback_add(TOPIC_ALT, _on_alt)

    c.message_callback_add(TOPIC_BATT, _on_batt)
    c.subscribe(TOPIC_BATT, qos=1)

    c.message_callback_add(TOPIC_TEL_SYS, _on_warn)
    c.subscribe(TOPIC_TEL_SYS, qos=1)

    c.subscribe(TOPIC_COMPLETED, qos=QOS_CMD)
    c.message_callback_add(TOPIC_COMPLETED, _on_completed)


def _on_status(c, u, msg):
    global last_status
    try:
        payload = msg.payload.decode("utf-8")

        # metti in coda per stampa non distruttiva
        q.put(f"[UAV] UAV Status: {payload}")
    except Exception as e:
        print(f"[ERROR] {e}")


def _on_alt(c, u, msg):
    global last_status
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        alt = payload.get("altitude")

        # metti in coda per stampa non distruttiva
        q.put(f"[UAV] UAV Altitude: {alt}")
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


def _on_warn(c, u, msg):
    try:
        data = json.loads(msg.payload.decode("utf-8"))
        sev = data.get("severity", "INFO")
        text = data.get("text", "")
        q.put(f"[UAV] {sev}: {text}")
    except Exception as e:
        print("[ERROR] parse error:", e)


def _on_message(c, u, msg):
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        cid = payload.get("command_id")
        if cid:
            # Se stiamo tracciando la latenza di questo comando, logga RTT
            t_send_ns = _pending_lat.pop(cid, None)
            if t_send_ns is not None:
                t_recv_ns = time.time_ns()
                record_latency_response(cid, t_recv_ns, t_send_ns)

            # Sblocca publish_cmd se è in attesa di questa risposta
            if cid in _pending:
                _pending[cid] = payload
    except Exception as e:
        print("[ACK] parse error:", e)


def _on_completed(c, u, msg):
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        cid = payload.get("command_id")
        if cid:
            t_send_ns = _pending_completion.pop(cid, None)
            if t_send_ns is not None:
                t_recv_ns = time.time_ns()
                record_completion_response(cid, t_recv_ns, t_send_ns)
    except Exception as e:
        print("[COMPLETED] parse error:", e)


def heartbeat_loop(client: mqtt.Client):
    """Pubblica periodicamente un heartbeat per segnalare che la GCS è attiva."""
    while True:
        try:
            payload = json.dumps({"ts": time.time()})
            client.publish(TOPIC_HEARTBEAT, payload, qos=0, retain=False)
        except Exception as e:
            print(f"[HB] publish error: {e}")
        time.sleep(HEARTBEAT_INTERVAL)


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
    mqttc.loop_start()

    # Thread che stampa messaggi dall'UAV senza rompere l'input
    t = threading.Thread(target=safe_printer, args=("sitl> ",), daemon=True)
    # threading.Thread(target=heartbeat_loop, args=(mqttc,), daemon=True).start()
    t.start()
    return mqttc


def publish_cmd(
        client: mqtt.Client,
        payload: Dict[str, Any],
        await_ack: bool = True,
        timeout: float = 5.0,
        qos=QOS_CMD
) -> Optional[Dict[str, Any]]:
    cid = payload.setdefault("command_id", f"cmd-{uuid.uuid4().hex[:8]}")
    data = json.dumps(payload, separators=(",", ":"))

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

    if qos != QOS_CMD:
        payload["qos"] = qos
        data = json.dumps(payload, separators=(",", ":"))

    t_send_ns: Optional[int] = None
    if await_ack:
        t_send_ns = time.time_ns()
        _pending_lat[cid] = t_send_ns
        record_latency_request(cid, t_send_ns)
        _pending_completion[cid] = t_send_ns
        record_completion_request(cid, t_send_ns)

    info = client.publish(TOPIC_CMD, data, qos=qos, retain=False)
    info.wait_for_publish()
    print(f"[GCS→MQTT] {TOPIC_CMD} {data}")

    if not await_ack:
        return None

    # meccanismo di attesa ACK
    _pending[cid] = None
    t0 = time.time()
    while time.time() - t0 < timeout:
        if _pending[cid] is not None:
            res = _pending.pop(cid)
            return res
        time.sleep(0.05)

    res = _pending.pop(cid, None)

    if t_send_ns is not None:
        # se non abbiamo ricevuto ACK, logghiamo timeout di latenza
        t_send_ns = _pending_lat.pop(cid, None)
        if t_send_ns is not None and res is None:
            record_latency_timeout(cid, t_send_ns)
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
    print("=== ArduPilot SITL Interactive CLI ===")
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
                print(info)
                print("[UAV] Takeoff starting...")
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
    try:
        cli(c)
    finally:
        c.loop_stop()
        c.disconnect()