
# sitl_cli.py — Controller interattivo per ArduPilot SITL (DroneKit + pymavlink)
# - GUIDED / arm / takeoff
# - Movimento per distanza (direzione + velocità) o per tempo (vx,vy,vz + duration)
# - Yaw (assoluto/relativo)
# - Goto (lat, lon, alt)
# - RTL, LAND, DISARM
# - Param show/set
# - Battery sim quick set (opzionale)

from __future__ import annotations


import time
from typing import Optional
from dataclasses import dataclass

from dronekit import connect, collections, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

DEFAULT_LINK = "127.0.0.1:14550"

# ---------------------------
# Helpers DroneKit
# ---------------------------
def connect_vehicle(connection: str = DEFAULT_LINK):
    print(f"[INFO] Connecting to {connection} ...")
    vehicle = connect(connection, wait_ready=True, timeout=60)
    print("[OK] Connected.")
    return vehicle

def wait_for_mode(vehicle, mode: str, timeout: float = 10):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if vehicle.mode.name.upper() == mode.upper():
            return True
        time.sleep(0.1)
    raise TimeoutError(f"Mode '{mode}' not set in time (current: {vehicle.mode.name}).")

def set_mode(vehicle, mode: str):
    vehicle.mode = VehicleMode(mode.upper())
    wait_for_mode(vehicle, mode)

def arm_and_takeoff(vehicle, target_alt: float):
    print("[ACTION] Set mode GUIDED")
    set_mode(vehicle, "GUIDED")

    print("[ACTION] Arming...")
    vehicle.armed = True
    while not vehicle.armed:
        print("  ... waiting for armable/arming")
        time.sleep(0.5)

    print(f"[ACTION] Takeoff to {target_alt} m")
    vehicle.simple_takeoff(target_alt)
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
        0b0000111111000111,  # abilitati solo i campi velocity
        0,0,0,  # position
        vx,vy,vz,
        0,0,0,  # accel
        0,0     # yaw, yaw_rate
    )
    t0 = time.time()
    while time.time() - t0 < duration:
        vehicle.send_mavlink(msg)
        vehicle.flush()
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

def set_speed(vehicle, speed_m_s: float):
    vehicle.airspeed = speed_m_s
    vehicle.groundspeed = speed_m_s

def goto_absolute(vehicle, lat: float, lon: float, alt: float):
    target = LocationGlobalRelative(lat, lon, alt)
    vehicle.simple_goto(target)

def do_rtl(vehicle): set_mode(vehicle, "RTL")
def do_land(vehicle): set_mode(vehicle, "LAND")

def disarm(vehicle):
    vehicle.armed = False
    t0 = time.time()
    while vehicle.armed and time.time() - t0 < 10:
        time.sleep(0.2)

def set_param(vehicle, name: str, value):
    print(f"[PARAM] {name} = {value}")
    vehicle.parameters[name] = value

def show_param(vehicle, name: str):
    val = vehicle.parameters.get(name, None)
    print(f"[PARAM] {name} => {val}")
    return val

# ---------------------------
# Movimenti “ad alto livello”
# ---------------------------
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

# ---------------------------
# CLI
# ---------------------------
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
  reboot                      # riavvia autopilota

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
    print("=== ArduPilot SITL Interactive CLI ===")
    print(HELP)
    vehicle = None
    vehicle = connect_vehicle(DEFAULT_LINK)

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

            elif cmd == "connect":
                link = parts[1] if len(parts) > 1 else DEFAULT_LINK
                if vehicle:
                    vehicle.close()
                vehicle = connect_vehicle(link)

            elif cmd == "guided":
                set_mode(vehicle, "GUIDED")

            elif cmd == "arm":
                vehicle.armed = True
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
                # prova int poi float
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
                # non tutti i firmwares espongono battery coerente in SITL
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

if __name__ == "__main__":
    cli()
    