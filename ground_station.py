# gcs_mqtt_client.py
import json, time, uuid, threading
from typing import Dict, Any, Optional
import paho.mqtt.client as mqtt

BROKER_HOST = "10.42.0.1"
BROKER_PORT = 8883
USE_TLS = True
UAV_ID = "uav1"
TOPIC_CMD = f"uav/{UAV_ID}/cmd"
TOPIC_ACK = f"uav/{UAV_ID}/ack"
QOS_CMD = 1

CERT_CA = "/etc/mosquitto/ca_certificates/ca.crt"
CERT_FILE = "/etc/mosquitto/certs/gcs.crt"
KEY_FILE = "/etc/mosquitto/certs/gcs.key"

_pending = {}  # command_id -> event/result


def _on_connect(c, u, f, rc):
    print(f"[MQTT] connected rc={rc}")
    c.subscribe(TOPIC_ACK, qos=1)


def _on_message(c, u, msg):
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        cid = payload.get("command_id")
        if cid and cid in _pending:
            _pending[cid] = payload
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
    mqttc.connect(BROKER_HOST, BROKER_PORT, keepalive=30)
    mqttc.loop_start()
    return mqttc


def publish_cmd(client: mqtt.Client, payload: Dict[str, Any], await_ack: bool = True, timeout: float = 5.0) -> Optional[
    Dict[str, Any]]:
    cid = payload.setdefault("command_id", f"cmd-{uuid.uuid4().hex[:8]}")
    data = json.dumps(payload, separators=(",", ":"))
    info = client.publish(TOPIC_CMD, data, qos=QOS_CMD, retain=False)
    info.wait_for_publish()
    print(f"[GCS→MQTT] {TOPIC_CMD} {data}")
    if not await_ack:
        return None
    _pending[cid] = None
    t0 = time.time()
    while time.time() - t0 < timeout:
        if _pending[cid] is not None:
            return _pending.pop(cid)
        time.sleep(0.05)
    return _pending.pop(cid)  # None se timeout


HELP = """
Comandi disponibili:

SYSTEM / MODE
  connect [<link>]            # predef: 127.0.0.1:14550
  guided                      # mode GUIDED
  arm                         # arma (in GUIDED)
  disarm
  takeoff <alt_m>
  land                        # mode LAND
  rtl                         # mode RTL
  reboot                      # riavvia l'autopilota

MOVE (NED)
  move <dir> <speed_mps> <distance_m>    # dir: north|south|east|west|up|down
  vel <vx> <vy> <vz> <duration_s>
  yaw <heading_deg> [rate_deg_s=30] [relative=0|1] [cw=1|0]
  setspeed <mps>
  goto <lat> <lon> <alt_m>

PARAM / STATUS
  pshow <NAME>
  pset <NAME> <VALUE>
  status
  alt
  batt

BATTERY (SITL)
  batt_full             # SIM_BATT_CAPACITY=100, SIM_BATT_VOLTAGE=12.6 (es. 3S)
  batt_set <pct> <volt> # imposta percentuale e voltaggio

ALTRO
  help
  quit / exit
"""


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

        try:
            if cmd == "help":
                print(HELP)
            elif cmd == "guided":
                publish_cmd(c, {"type": "mode", "mode": "GUIDED"})
                print("[UAV] Mode GUIDED")
            elif cmd == "arm":
                publish_cmd(c, {"type": "arm", "arm": True})
                print("[UAV] ARMED")
            elif cmd == "disarm":
                publish_cmd(c, {"type": "arm", "arm": False})
                print("[UAV] DISARMED")
            elif cmd == "takeoff":
                alt = float(parts[1])
                info = publish_cmd(c, {"type": "takeoff", "alt": alt}, timeout=25)
                print("[UAV] Takeoff completed")
            elif cmd == "move":
                dir = str(parts[1])
                speed = float(parts[2])
                dist = float(parts[3])
                publish_cmd(c, {"type": "move", "direction": dir, "speed": speed, "distance": dist})
                print("[UAV] Move set")
            elif cmd == "vel":
                vx, vy, vz, dur = map(float, parts[1:5])
                publish_cmd(c, {"type": "velocity", "vx": vx, "vy": vy, "vz": vz, "duration": dur})
                print("[UAV] Velocity set")
            elif cmd == "yaw":
                heading = float(parts[1])
                rate = float(parts[2]) if len(parts) > 2 else 30.0
                relative = bool(int(parts[3])) if len(parts) > 3 else False
                cw = bool(int(parts[4])) if len(parts) > 4 else True
                publish_cmd(c, {"type": "yaw", "heading": heading, "rate": rate, "relative": relative, "cw": cw})
                print("[UAV] Yaw set")
            elif cmd == "setspeed":
                speed = float(parts[1])
                publish_cmd(c, {"type": "setspeed", "speed": speed})
                print(f"[UAV] Set speed: {speed}")
            elif cmd == "goto":
                lat = float(parts[1]);
                lon = float(parts[2]);
                alt = float(parts[3])
                publish_cmd(c, {"type": "goto", "lat": lat, "lon": lon, "alt": alt})
            elif cmd == "rtl":
                info = publish_cmd(c, {"type": "mode", "mode": "RTL"})
                mode = info.get("mode")
                print(f"[UAV] Mode {mode}")
            elif cmd == "land":
                info = publish_cmd(c, {"type": "mode", "mode": "LAND"})
                mode = info.get("mode")
                print(f"[UAV] Mode {mode}")
            elif cmd == "get":
                param = str(parts[1])
                info = publish_cmd(c, {"type": "get", "name": param})
                val = info.get("value")
                print(f"[UAV] Param {param}: {val}")
            elif cmd == "batt":
                info = publish_cmd(c, {"type": "batt"})
                volt = info.get("voltage")
                curr = info.get("current")
                lev = info.get("level")
                print(f"[UAV] Battery voltage: {volt}, current: {curr}, level: {lev}")


        except Exception as e:
            print(f"[ERROR] {e}")


if __name__ == "__main__":
    c = make_client()
    cli(c)
