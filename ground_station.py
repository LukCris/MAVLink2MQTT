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

# readline enables line-editing and history in the interactive prompt on Linux/macOS.
# It is imported optionally because it is unavailable on Windows; the CLI degrades
# gracefully to a plain input() call in that case
try:
    import readline
except Exception:
    readline = None

# ========================
# MQTT / TLS configuration
# ========================
BROKER_HOST = "10.42.0.1"
BROKER_PORT = 8883          # TLS port (plain MQTT would use 1883)
USE_TLS = True
UAV_ID = "uav1"

# Mutual-TLS certificates: the GCS presents its own cert/key and validates
# the broker against the shared CA certificate.
CERT_CA = "./certs/ca.crt"
CERT_FILE = "./certs/client.crt"
KEY_FILE = "./certs/client.key"


# ========================
# MQTT topic definitions
# ========================
TOPIC_CMD       = f"uav/{UAV_ID}/cmd"                   # GCS → drone: JSON commands
TOPIC_STATUS    = f"uav/{UAV_ID}/status"                # drone → GCS: online/offline LWT
TOPIC_BATT      = f"uav/{UAV_ID}/telemetry/battery"     # drone → GCS: periodic battery data
TOPIC_ALT       = f"uav/{UAV_ID}/telemetry/altitude"    # drone → GCS: altitude during take-off/landing
TOPIC_ACK       = f"uav/{UAV_ID}/ack"                   # drone → GCS: immediate command acknowledgement
TOPIC_COMPLETED = f"uav/{UAV_ID}/completed"             # drone → GCS: command execution finished
TOPIC_TEL_SYS   = f"uav/{UAV_ID}/telemetry/sys"         # drone → GCS: autopilot STATUSTEXT messages
TOPIC_HEARTBEAT = f"uav/{UAV_ID}/heartbeat"             # GCS → broker: periodic liveness signal


HEARTBEAT_INTERVAL = 3.0   # seconds between heartbeat publishes
QOS_CMD = 1                # default QoS for command/ack traffic


# ========================
# Pending-command tracking
# ========================

# Maps command_id → t_send_ns for the *immediate* ACK latency measurement.
# Entries are inserted just before publish and removed when the ACK arrives
# (or on timeout).
_pending_completion: Dict[str, int] = {}

# Maps command_id → response payload (or None while waiting).
# publish_cmd() busy-polls this dict until the ACK callback fills it in.
_pending: Dict[str, Optional[Dict[str, Any]]] = {}

# Maps command_id → t_send_ns for the *completion* latency measurement.
# Filled at publish time; removed when the TOPIC_COMPLETED notification arrives.
_pending_lat: Dict[str, int] = {}

# Thread-safe queue used to decouple incoming MQTT messages from the CLI prompt.
# Background callbacks push strings here; safe_printer() drains it on a
# dedicated thread so that async output never garbles the user's input line.
q: "queue.Queue[str]" = queue.Queue()


last_batt = {"voltage": None, "current": None, "level": None}

LOG_DIR = os.environ.get("ML2MQTT_LOG_DIR", "./logs")
os.makedirs(LOG_DIR, exist_ok=True)

log_ap = logging.getLogger("autopilot")
log_ap.setLevel(logging.WARNING)

# Initialize CSV log files for latency and completion metrics.
init_latency_logger()
init_completion_logger()


# ========================
# Utils
# ========================
def _now_utc_iso():
    """Return the current UTC time as an ISO-8601 string with millisecond precision."""
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def safe_print(line: str, prompt: str = "sitl> "):
    """
        Print line to stdout without corrupting the readline input buffer.

        When readline is active the user may have partially typed a command.
        A naive print() would leave the cursor mid-line and interleave the
        new text with the user's input.  This function works around that by:
          1. Overwriting the current line with spaces to erase the prompt + buffer.
          2. Printing the new message on a clean line.
          3. Re-writing the prompt and whatever the user had typed so far,
             restoring the visual state of the terminal.
        Without readline (e.g. on Windows) it falls back to a plain print().
    """
    if readline:
        buf = readline.get_line_buffer()
        sys.stdout.write("\r")
        sys.stdout.write(" " * (len(prompt) + len(buf)))
        sys.stdout.write("\r")
        print(line)
        sys.stdout.write(prompt + buf)
        sys.stdout.flush()
    else:
        print(line)


def safe_printer(prompt: str = "sitl> "):
    """
        Dedicated consumer thread for the print queue q.

        Runs in an infinite loop, blocking on q.get().  A None sentinel value
        signals the thread to exit (used for clean shutdown).
        All MQTT callbacks route their output through this thread so that
        concurrent prints never race with the interactive prompt.
    """
    while True:
        msg = q.get()
        if msg is None:
            break
        safe_print(msg, prompt=prompt)


def qos_retriever(args):
    """
        Parse the optional '-q <level>' flag from a CLI argument list.

        Returns the requested QoS level (0, 1, or 2) if the flag is present
        and the value differs from the current global default QOS_CMD.
        Returns None if the flag is absent or the requested level already
        matches the default (publish_cmd() then uses QOS_CMD unchanged).
        Prints an error and returns None on invalid input.
    """
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
            # Requested level is already the default; no override needed.
            qos = None
            print(f"[UAV] QoS is actually set to {QOS_CMD}")
    return qos


# ========================
# MQTT callbacks
# ========================
def _on_connect(c, u, f, rc):
    """
        Called by the Paho network thread once the broker connection is established.

        Subscribes to all relevant topics and registers per-topic callbacks so
        that each message type is handled by a dedicated function rather than
        a single monolithic on_message handler.
        The QoS for ACK/COMPLETED subscriptions mirrors QOS_CMD so that the
        delivery guarantee is symmetric between publish and subscribe.
    """
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

    # Subscribe to completion notifications (second ACK, carrying execution time).
    c.subscribe(TOPIC_COMPLETED, qos=QOS_CMD)
    c.message_callback_add(TOPIC_COMPLETED, _on_completed)


def _on_status(c, u, msg):
    """Handle LWT / online-offline status messages from the drone."""
    global last_status
    try:
        payload = msg.payload.decode("utf-8")
        q.put(f"[UAV] UAV Status: {payload}")
    except Exception as e:
        print(f"[ERROR] {e}")


def _on_alt(c, u, msg):
    """Handle real-time altitude updates published during take-off and landing."""
    global last_status
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        alt = payload.get("altitude")
        q.put(f"[UAV] UAV Altitude: {alt}")
    except Exception as e:
        print(f"[ERROR] {e}")


def _on_batt(c, u, msg):
    """
       Handle periodic battery telemetry from the drone.

       Updates the module-level *last_batt* snapshot so other parts of the
       CLI can query the latest known battery state without re-requesting it.
    """
    global last_batt
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        v = payload.get("voltage")
        crr = payload.get("current")
        lvl = payload.get("level")
        last_batt = {"voltage": v, "current": crr, "level": lvl}
        q.put(f"[BATT] voltage={v}V  current={crr}A  level={lvl}%")
    except Exception as e:
        print(f"[ERROR] {e}")


def _on_warn(c, u, msg):
    """Forward autopilot STATUSTEXT messages (severity + text) to the print queue."""
    try:
        data = json.loads(msg.payload.decode("utf-8"))
        sev = data.get("severity", "INFO")
        text = data.get("text", "")
        q.put(f"[UAV] {sev}: {text}")
    except Exception as e:
        print("[ERROR] parse error:", e)


def _on_message(c, u, msg):
    """
        Default callback for TOPIC_ACK: records the immediate-ACK RTT and
        unblocks publish_cmd() by storing the response in _pending.

        This is the first of two ACK stages:
          - Stage 1 (here): the drone has *received and accepted* the command.
            Latency = network round-trip only.
          - Stage 2 (_on_completed): the drone has *finished executing* the
            command.  Latency = network + execution time.
    """
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        cid = payload.get("command_id")
        if cid:
            # Pop the send timestamp and log the RTT for the immediate ACK.
            t_send_ns = _pending_lat.pop(cid, None)
            if t_send_ns is not None:
                t_recv_ns = time.time_ns()
                record_latency_response(cid, t_recv_ns, t_send_ns)

            # Signal publish_cmd() that the response is ready.
            if cid in _pending:
                _pending[cid] = payload
    except Exception as e:
        print("[ACK] parse error:", e)


def _on_completed(c, u, msg):
    """
        Callback for TOPIC_COMPLETED: records the end-to-end command completion
        latency (network round-trip + drone-side execution time).

        The drone publishes to this topic only after the command has fully
        executed (e.g. after a move maneuver finishes), giving a second,
        independent timing measurement stored in a separate CSV file.
    """
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
    """
        Publish a lightweight JSON heartbeat to TOPIC_HEARTBEAT every
        HEARTBEAT_INTERVAL seconds at QoS 0 (fire-and-forget).

        The heartbeat lets the broker and any monitoring tools detect that
        the GCS process is still running, independently of command traffic.

        Note: this loop is currently disabled in make_client() (thread start
        is commented out) and is retained for future use.
    """
    while True:
        try:
            payload = json.dumps({"ts": time.time()})
            client.publish(TOPIC_HEARTBEAT, payload, qos=0, retain=False)
        except Exception as e:
            print(f"[HB] publish error: {e}")
        time.sleep(HEARTBEAT_INTERVAL)


def make_client() -> mqtt.Client:
    """
        Build, configure, and connect the Paho MQTT client.

        Steps:
          1. Create the client with a unique ID (prevents session collisions
             when the GCS is restarted quickly).
          2. Attach TLS mutual-authentication credentials if USE_TLS is True.
          3. Connect to the broker and start the Paho background network thread
             (loop_start), which handles reconnections and dispatches callbacks.
          4. Spawn the safe_printer thread that drains the async print queue.

        Returns the connected client ready for use by publish_cmd() and cli().
    """
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
    """
        Serialize and publish a command to TOPIC_CMD, then optionally wait for
        the immediate ACK from the drone.

        Parameters
        ----------
        client    : connected Paho MQTT client.
        payload   : command dict (must contain at least "type").
        await_ack : if True, block until the drone's ACK arrives or *timeout*
                    expires.
        timeout   : maximum seconds to wait for the ACK (default 5 s).
        qos       : MQTT QoS level for this publish (0, 1, or 2).  Defaults to
                    the global QOS_CMD; can be overridden per-command via -q.

        Returns
        -------
        The ACK payload dict on success, or None on timeout / QoS error.

        Latency measurement
        -------------------
        When await_ack is True the function records t_send_ns (nanoseconds) in
        both _pending_lat and _pending_completion before publishing:
          - _pending_lat   → consumed by _on_message()  → immediate-ACK RTT CSV
          - _pending_completion → consumed by _on_completed() → completion RTT CSV
        This gives two independent timing series from a single publish call.

        ACK wait mechanism
        ------------------
        The function pre-inserts '_pending[cid] = None' before publishing, so
        that if the ACK arrives before the polling loop starts (possible at very
        low latency / QoS 0) it is not lost.  The loop then busy-polls with a
        50 ms sleep until the callback fills in the response or the timeout fires.
    """

    # Assign a unique command ID if the caller has not supplied one.
    cid = payload.setdefault("command_id", f"cmd-{uuid.uuid4().hex[:8]}")
    data = json.dumps(payload, separators=(",", ":"))

    if qos is None:
        qos = QOS_CMD

    try:
        qos = int(qos)
    except (TypeError, ValueError):
        print(f"[ERROR] Invalid QoS value: {qos}")
        return None

    if qos not in (0, 1, 2):
        print(f"[ERROR] QoS must be 0, 1 or 2, got {qos}")
        return None

    # Embed the per-command QoS override in the payload so the drone side
    # can log which QoS level was actually used for this exchange.
    if qos != QOS_CMD:
        payload["qos"] = qos
        data = json.dumps(payload, separators=(",", ":"))

    t_send_ns: Optional[int] = None
    if await_ack:
        # Capture the send timestamp in nanoseconds for high-resolution RTT.
        t_send_ns = time.time_ns()
        _pending_lat[cid] = t_send_ns
        record_latency_request(cid, t_send_ns)
        _pending_completion[cid] = t_send_ns
        record_completion_request(cid, t_send_ns)

    info = client.publish(TOPIC_CMD, data, qos=qos, retain=False)
    # wait_for_publish() blocks until the broker has acknowledged the publish
    # at the negotiated QoS level (no-op for QoS 0).
    info.wait_for_publish()
    print(f"[GCS→MQTT] {TOPIC_CMD} {data}")

    if not await_ack:
        return None

    # Pre-register the pending slot *before* the poll loop to avoid a race
    # where the ACK callback fires between publish and the first dict lookup.
    _pending[cid] = None
    t0 = time.time()
    while time.time() - t0 < timeout:
        if _pending[cid] is not None:
            res = _pending.pop(cid)
            return res
        time.sleep(0.05)

    # Timeout path: clean up and log the missing ACK.
    res = _pending.pop(cid, None)
    if t_send_ns is not None:
        # se non abbiamo ricevuto ACK, logghiamo timeout di latenza
        t_send_ns = _pending_lat.pop(cid, None)
        if t_send_ns is not None and res is None:
            record_latency_timeout(cid, t_send_ns)
    return res


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


# ========================
# Interactive CLI
# ========================
def cli(c):
    """
        Read-eval-print loop for the Ground Control Station.

        Each iteration reads one line from stdin, splits it into tokens, and
        dispatches to the appropriate publish_cmd() call.  The optional '-q'
        flag on any command is extracted by qos_retriever() before the
        positional arguments are parsed, so argument indices remain stable
        regardless of where the flag appears.

        The global QOS_CMD can be changed at runtime with the 'qos' command,
        affecting all subsequent commands that do not supply an explicit '-q'.
    """
    global QOS_CMD
    print("=== ArduPilot Interactive CLI ===")
    print(HELP)

    while True:
        try:
            raw = input("UAV> ").strip()
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
                # Longer timeout because the drone must arm and reach altitude
                # before it sends the ACK; 25 s covers typical SITL take-off.
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

                # yaw has a non-standard argument layout so -q is parsed
                # manually here instead of via qos_retriever().
                if "-q" in args:
                    idx = args.index("-q")
                    try:
                        qos_level = int(args[idx + 1])
                    except (IndexError, ValueError):
                        print("[ERROR] yaw: missing or invalid QoS after -q")
                        continue
                    del args[idx:idx + 2]   # remove flag + value in-place

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
                # Permanently change the default QoS level for this session.
                # Individual commands can still override it with -q.
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