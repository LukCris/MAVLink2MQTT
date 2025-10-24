# sitl_cli.py — Controller interattivo per ArduPilot SITL (DroneKit + pymavlink + MQTT raw)
# - GUIDED / arm / takeoff
# - Movimento per distanza (direzione + velocità) o per tempo (vx,vy,vz + duration)
# - Yaw (assoluto/relativo)
# - Goto (lat, lon, alt)
# - RTL, LAND, DISARM
# - Param show/set (+ publish MAVLink raw)
# - Battery sim quick set (opzionale)
# - MQTT: pubblica frame MAVLink raw "alla Balzano" su topic dedicati

from __future__ import annotations

import time
import os
import ssl
from typing import Optional
from dataclasses import dataclass

import paho.mqtt.client as mqtt
from dronekit import connect, collections, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# =========================
# Config rete / MQTT
# =========================
USE_TLS = False                   # True se vuoi TLS
BROKER  = "127.0.0.1"            # IP broker (es. 10.42.0.1 sulla GCS hotspot)
PORT_TLS = 8883
PORT_NO_TLS = 1883
PORT = PORT_TLS if USE_TLS else PORT_NO_TLS

# Identità / topic
UAV_ID      = "alpha"
TARGET_SYS  = 1   # system id del drone a cui invii
TARGET_COMP = 1   # component id (generalmente 1)
SRC_SYS     = 254 # system id della GCS che pubblica
SRC_COMP    = 190 # component id della GCS

TOPIC_MAV_RAW_OUT = f"uav/{UAV_ID}/{TARGET_SYS}/{TARGET_COMP}/mav/out/raw"
TOPIC_STATUS      = f"uav/{UAV_ID}/{TARGET_SYS}/{TARGET_COMP}/status"

# Cert (se TLS)
CERT_CA   = "/etc/mosquitto/ca_certificates/ca.crt"
CERT_FILE = "/etc/mosquitto/certs/client.crt"
KEY_FILE  = "/etc/mosquitto/certs/client.key"

# =========================
# MQTT setup (client globale)
# =========================
mqttc = mqtt.Client(client_id=f"gcs-{os.getpid()}", clean_session=True)
mqttc.will_set(TOPIC_STATUS, payload=b"offline", qos=1, retain=True)

if USE_TLS:
    mqttc.tls_set(
        ca_certs=CERT_CA,
        certfile=CERT_FILE,
        keyfile=KEY_FILE,
        tls_version=ssl.PROTOCOL_TLSv1_2
    )

mqttc.connect(BROKER, PORT, 60)
mqttc.loop_start()
mqttc.publish(TOPIC_STATUS, b"online", qos=1, retain=True)

# =========================
# MAVLink context per impacchettare msg con src sys/comp della GCS
# =========================
mav_ctx = mavutil.mavlink.MAVLink(None)
mav_ctx.srcSystem = SRC_SYS
mav_ctx.srcComponent = SRC_COMP

def mqtt_publish_mav(msg_obj, qos: int = 0):
    """
    Impacchetta un MAVLink_*_message e lo pubblica come bytes su TOPIC_MAV_RAW_OUT.
    """
    try:
        msg_obj.pack(mav_ctx)           # importante: aggiunge header con SRC_SYS/SRC_COMP
        payload = msg_obj.get_msgbuf()  # bytes serializzati
        mqttc.publish(TOPIC_MAV_RAW_OUT, payload=payload, qos=qos)
    except Exception as e:
        print(f"[MQTT/MAV] Publish error: {e}")

# =========================
# Helpers DroneKit
# =========================
def wait_for_mode(vehicle, mode: str, timeout: float = 10):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if vehicle.mode.name.upper() == mode.upper():
            return True
        time.sleep(0.1)
    raise TimeoutError(f"Mode '{mode}' not set in time (current: {vehicle.mode.name}).")

# ---- SET_MODE: invia sia a DroneKit che come frame raw (SET_MODE) ----
COPTER_MODES = {
    "STABILIZE": 0, "ACRO": 1, "ALT_HOLD": 2, "AUTO": 3,
    "GUIDED": 4, "LOITER": 5, "RTL": 6, "CIRCLE": 7, "LAND": 9
}

def publish_set_mode_raw(mode: str):
    mname = mode.upper()
    cmode = COPTER_MODES.get(mname)
    if cmode is None:
        return
    msg = mavutil.mavlink.MAVLink_set_mode_message(
        target_system=TARGET_SYS,
        base_mode=0,          # lasciamo 0: usa custom_mode
        custom_mode=cmode
    )
    mqtt_publish_mav(msg, qos=1)       # comandi critici → QoS 1

def set_mode(vehicle, mode: str):
    vehicle.mode = VehicleMode(mode.upper())
    wait_for_mode(vehicle, mode)
    publish_set_mode_raw(mode)

def arm_and_takeoff(vehicle, target_alt: float):
    print("[ACTION] Set mode GUIDED")
    set_mode(vehicle, "GUIDED")

    print("[ACTION] Arming...")
    vehicle.armed = True
    while not vehicle.armed:
        print("  ... waiting for armable/arming")
        time.sleep(0.5)

    # Pubblica anche ARM (opzionale) come command_long raw
    arm_cmd = mavutil.mavlink.MAVLink_command_long_message(
        target_system=TARGET_SYS, target_component=TARGET_COMP,
        command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        confirmation=0, param1=1, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0
    )
    mqtt_publish_mav(arm_cmd, qos=1)

    print(f"[ACTION] Takeoff to {target_alt} m")
    vehicle.simple_takeoff(target_alt)

    # Pubblica anche il TAKEOFF raw (per il PC-drone)
    to_cmd = mavutil.mavlink.MAVLink_command_long_message(
        target_system=TARGET_SYS, target_component=TARGET_COMP,
        command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        confirmation=0,
        param1=0, param2=0, param3=0, param4=0,
        param5=0, param6=0, param7=target_alt
    )
    mqtt_publish_mav(to_cmd, qos=1)

    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"  Alt: {alt:.1f} m")
        if alt >= 0.95 * target_alt:
            print("[OK] Reached target altitude")
            break
        time.sleep(0.5)

def send_velocity_ned(vehicle, vx: float, vy: float, vz: float, duration: float):
    """
    Invia velocità NED per 'duration' secondi.
    vx, vy, vz in m/s (x=Nord +, y=Est +, z=Giù +)
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  # abilita solo i campi velocity
        0,0,0,               # position
        vx,vy,vz,            # velocity
        0,0,0,               # accel
        0,0                  # yaw, yaw_rate
    )
    t0 = time.time()
    while time.time() - t0 < duration:
        vehicle.send_mavlink(msg)
        vehicle.flush()
        # pubblica anche il frame raw su MQTT (stream → QoS 0)
        mqtt_publish_mav(msg, qos=0)
        time.sleep(0.1)

def set_yaw(vehicle, heading_deg: float, rate_deg_s: float = 30.0, relative: bool = False, cw: bool = True):
    rel = 1 if relative else 0
    direction = 1 if cw else -1
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
        heading_deg, rate_deg_s, direction, rel,
        0,0,0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()
    mqtt_publish_mav(msg, qos=1)  # yaw target → QoS 1

def set_speed(vehicle, speed_m_s: float):
    vehicle.airspeed = speed_m_s
    vehicle.groundspeed = speed_m_s
    # (opzionale) potresti anche pubblicare DO_CHANGE_SPEED come raw se vuoi
    # msg = mavutil.mavlink.MAVLink_command_long_message(...)

def goto_absolute(vehicle, lat: float, lon: float, alt: float):
    target = LocationGlobalRelative(lat, lon, alt)
    vehicle.simple_goto(target)

    # Pubblica anche il target posizione come GLOBAL_INT (critico → QoS 1)
    type_mask = 0b0000111111111000  # abilita x,y,z; disabilita il resto
    msg = mavutil.mavlink.MAVLink_set_position_target_global_int_message(
        time_boot_ms=0,
        target_system=TARGET_SYS,
        target_component=TARGET_COMP,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        type_mask=type_mask,
        lat_int=int(lat * 1e7),
        lon_int=int(lon * 1e7),
        alt=alt,
        vx=0, vy=0, vz=0,
        afx=0, afy=0, afz=0,
        yaw=0, yaw_rate=0
    )
    mqtt_publish_mav(msg, qos=1)

def do_rtl(vehicle):
    set_mode(vehicle, "RTL")     # set_mode pubblica anche il SET_MODE raw

def do_land(vehicle):
    set_mode(vehicle, "LAND")    # idem

def disarm(vehicle):
    vehicle.armed = False
    # Pubblica anche DISARM come command long
    dis_cmd = mavutil.mavlink.MAVLink_command_long_message(
        target_system=TARGET_SYS, target_component=TARGET_COMP,
        command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        confirmation=0, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0
    )
    mqtt_publish_mav(dis_cmd, qos=1)

    t0 = time.time()
    while vehicle.armed and time.time() - t0 < 10:
        time.sleep(0.2)

def set_param(vehicle, name: str, value):
    print(f"[PARAM] {name} = {value}")
    vehicle.parameters[name] = value
    # Pubblica anche PARAM_SET raw (best-effort tipo float)
    try:
        msg = mavutil.mavlink.MAVLink_param_set_message(
            target_system=TARGET_SYS,
            target_component=TARGET_COMP,
            param_id=name.encode("ascii")[:16],   # max 16 bytes
            param_value=float(value),
            param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        mqtt_publish_mav(msg, qos=1)
    except Exception as e:
        print(f"[PARAM raw publish] {e}")

def show_param(vehicle, name: str):
    val = vehicle.parameters.get(name, None)
    print(f"[PARAM] {name} => {val}")
    return val

# =========================
# Movimenti “ad alto livello”
# =========================
def move_direction_by_distance(vehicle, direction: str, speed: float, distance: float):
    """
    Muove nel frame NED per 'distance' metri alla 'speed' (m/s).
    direction ∈ {north, south, east, west, up, down}
    Calcola duration = distance / speed e invia velocity corrispondente.
    """
    if speed <= 0:
        raise ValueError("La velocità deve essere > 0")
    duration = distance / speed
    direction = direction.lower()
    vx=vy=vz=0.0

    if direction == "north":
        vx = speed
    elif direction == "south":
        vx = -speed
    elif direction == "east":
        vy = speed
    elif direction == "west":
        vy = -speed
    elif direction == "up":
        vz = -speed  # in NED z- è verso l'alto
    elif direction == "down":
        vz = speed
    else:
        raise ValueError("Direzione non valida. Usa: north|south|east|west|up|down")

    print(f"[MOVE] {direction} — speed={speed} m/s, distance={distance} m, duration≈{duration:.2f}s")
    send_velocity_ned(vehicle, vx, vy, vz, duration)

# =========================
# CLI
# =========================
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

def print_status(vehicle):
    loc = vehicle.location.global_relative_frame
    hdg = getattr(vehicle, "heading", None)
    print(f"[STATUS] mode={vehicle.mode.name} armed={vehicle.armed} "
          f"alt={loc.alt:.1f}m lat={loc.lat:.6f} lon={loc.lon:.6f} hdg={hdg}")

def cli():
    print("=== ArduPilot SITL Interactive CLI (MQTT raw enabled) ===")
    print(HELP)
    vehicle = None
    #vehicle = connect_vehicle(DEFAULT_LINK)

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

            elif cmd in ("quit", "exit"):
                break

            elif cmd == "guided":
                set_mode(vehicle, "GUIDED")

            elif cmd == "arm":
                vehicle.armed = True
                # pubblichiamo anche ARM raw
                arm_cmd = mavutil.mavlink.MAVLink_command_long_message(
                    target_system=TARGET_SYS, target_component=TARGET_COMP,
                    command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    confirmation=0, param1=1, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0
                )
                mqtt_publish_mav(arm_cmd, qos=1)
                while not vehicle.armed:
                    print("  ... arming")
                    time.sleep(0.3)

            elif cmd == "disarm":
                disarm(vehicle)

            elif cmd == "takeoff":
                alt = float(parts[1])
                arm_and_takeoff(vehicle, alt)

            elif cmd == "land":
                do_land(vehicle)

            elif cmd == "rtl":
                do_rtl(vehicle)

            elif cmd == "reboot":
                vehicle.reboot()

            elif cmd == "move":
                # move <dir> <speed> <distance>
                direction = parts[1]
                speed = float(parts[2])
                distance = float(parts[3])
                move_direction_by_distance(vehicle, direction, speed, distance)

            elif cmd == "vel":
                vx, vy, vz, dur = map(float, parts[1:5])
                send_velocity_ned(vehicle, vx, vy, vz, dur)

            elif cmd == "yaw":
                heading = float(parts[1])
                rate = float(parts[2]) if len(parts) > 2 else 30.0
                relative = bool(int(parts[3])) if len(parts) > 3 else False
                cw = bool(int(parts[4])) if len(parts) > 4 else True
                set_yaw(vehicle, heading, rate, relative, cw)

            elif cmd == "setspeed":
                spd = float(parts[1])
                set_speed(vehicle, spd)

            elif cmd == "goto":
                lat = float(parts[1]); lon = float(parts[2]); alt = float(parts[3])
                goto_absolute(vehicle, lat, lon, alt)

            elif cmd == "pshow":
                name = parts[1]
                show_param(vehicle, name)

            elif cmd == "pset":
                name = parts[1]
                val_str = parts[2]
                try:
                    val = int(val_str)
                except ValueError:
                    val = float(val_str)
                set_param(vehicle, name, val)

            elif cmd == "status":
                print_status(vehicle)

            elif cmd == "alt":
                alt = vehicle.location.global_relative_frame.alt
                print(f"[ALT] {alt:.2f} m")

            elif cmd == "batt":
                bat = getattr(vehicle, "battery", None)
                print(f"[BATT] {bat}")

            elif cmd == "batt_full":
                set_param(vehicle, "SIM_BATT_CAPACITY", 100)
                set_param(vehicle, "SIM_BATT_VOLTAGE", 12.6)

            elif cmd == "batt_set":
                pct = float(parts[1]); volt = float(parts[2])
                set_param(vehicle, "SIM_BATT_CAPACITY", pct)
                set_param(vehicle, "SIM_BATT_VOLTAGE", volt)

            else:
                print("[ERR] Comando sconosciuto. Digita 'help'.")

        except Exception as e:
            print(f"[ERROR] {e}")

    if vehicle:
        try:
            vehicle.close()
        except Exception:
            pass

    # stato offline MQTT
    mqttc.publish(TOPIC_STATUS, b"offline", qos=1, retain=True)
    mqttc.loop_stop()

if __name__ == "__main__":
    cli()
