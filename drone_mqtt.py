# drone_mqtt.py
import base64, json, threading, time, uuid, logging
from typing import Any, Dict
import paho.mqtt.client as mqtt
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

from metrics_logger import init_battery_logger, log_battery

# ========================
# Flight-state flags
# ========================
# These globals are set by takeoff() / set_mode() and read by telemetry_loop()
# to decide whether altitude streaming is currently needed.
TAKE_OFF = False   # True while the drone is climbing to the target altitude
LANDING  = False   # True while the drone is descending to the ground
ALT      = None    # Target altitude requested by the last takeoff() call [m]

# ========================
# MQTT / TLS configuration
# ========================
BROKER_HOST = "10.42.0.1"
BROKER_PORT = 8883          # TLS port (plain MQTT would use 1883)
USE_TLS     = True
UAV_ID      = "uav1"
# Unique client ID prevents session conflicts on broker restart or reconnection.
CLIENT_ID   = f"drone-bridge-{uuid.uuid4()}"

# Mutual-TLS certificates used by the drone to authenticate against the broker
# and to validate the broker's identity.
CERT_CA = "./certs/ca.crt"
CERT_FILE = "./certs/client.crt"
KEY_FILE = "./certs/client.key"

# ========================
# MQTT topic definitions
# ========================
TOPIC_CMD       = f"uav/{UAV_ID}/cmd"               # GCS → drone: JSON commands
TOPIC_ACK       = f"uav/{UAV_ID}/ack"               # drone → GCS: immediate acceptance ACK
TOPIC_COMPLETED = f"uav/{UAV_ID}/completed"         # drone → GCS: execution-finished notification
TOPIC_STATUS    = f"uav/{UAV_ID}/status"            # drone → GCS: online/offline LWT
TOPIC_TEL_ALT   = f"uav/{UAV_ID}/telemetry/altitude"# drone → GCS: altitude during take-off/landing
TOPIC_TEL_BAT   = f"uav/{UAV_ID}/telemetry/battery" # drone → GCS: periodic battery snapshot
TOPIC_TEL_SYS   = f"uav/{UAV_ID}/telemetry/sys"     # drone → GCS: autopilot STATUSTEXT messages
TOPIC_HEARTBEAT = f"uav/{UAV_ID}/heartbeat"         # GCS → drone: periodic liveness signal

# Raw MAVLink pass-through topics (base64-encoded binary frames).
# These allow a GCS to inject or sniff raw MAVLink traffic over MQTT
# without going through the JSON command layer.
TOPIC_MAV_TX = f"uav/{UAV_ID}/mav/tx"  # GCS → drone
TOPIC_MAV_RX = f"uav/{UAV_ID}/mav/rx"  # drone → GCS


# ========================
# Heartbeat / failsafe state
# ========================
_last_heartbeat_time = time.time()  # epoch of the most recent GCS heartbeat
_emergency_landing_active = False  # prevents repeated LAND commands

# Maximum silence from the GCS before the drone triggers an emergency landing.
HEARTBEAT_TIMEOUT = 10.0  # seconds


# ========================
# QoS runtime state
# ========================
QOS_CMD = 1   # default QoS for command/ACK traffic; may be updated per-command
QOS_TEL = 0   # telemetry is best-effort (high-rate, loss-tolerant)

# Lock that protects QOS_CMD against concurrent reads/writes: _handle_and_ack()
# may update it from a worker thread while _ack() reads it from the same thread.
_qos_lock = threading.Lock()


# ========================
# DroneKit / SITL link
# ========================
# SITL exposes a MAVLink endpoint on UDP port 14550 by default.
SITL_LINK   = "127.0.0.1:14550"
vehicle     = None   # DroneKit Vehicle object; populated in main()
mqtt_client = None   # Paho client; populated in main()
running     = True   # cleared in the finally block of main() to stop loops

# ========================
# Metrics
# ========================
_last_batt_timestamp = None


# ========================
# Battery logging
# ========================
def setup_battery_logging(vehicle):
    """
        Register a DroneKit message listener for MAVLink SYS_STATUS frames.

        SYS_STATUS carries voltage, current and remaining-capacity fields encoded
        as unsigned integers with sentinel values (0xFFFF / 0xFF) that indicate
        "not available".  These are converted to SI units before logging:
          - voltage_battery: uint16, unit = mV  → divide by 1000 to get V
          - current_battery: int16,  unit = cA  → divide by 100 to get A
          - battery_remaining: int8, unit = %   → sentinel 0xFF means unknown

        The inner function is defined inside setup_battery_logging so that it
        closes over vehicle and is automatically bound to its message bus.
    """
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
        Register a DroneKit listener for MAVLink STATUSTEXT messages and forward
        them to the GCS via MQTT on TOPIC_TEL_SYS.

        STATUSTEXT is the autopilot's human-readable log channel (arming checks,
        mode changes, error alerts, etc.).  DroneKit may deliver msg.text as either
        a str or a bytes object depending on the pymavlink version, so both cases
        are handled.  NUL padding (common in fixed-length MAVLink char arrays) is
        stripped before forwarding.

        The severity integer is mapped to a human-readable label using the standard
        MAV_SEVERITY enum so the GCS can display or filter by level without needing
        the pymavlink library itself.
    """
    @vehicle.on_message('STATUSTEXT')
    def listener(self, name, msg):
        global mqtt_client

        try:
            text = msg.text
            if isinstance(text, bytes):
                text = text.decode("utf-8", errors="ignore")
            text = text.strip("\x00")
        except Exception:
            text = "<invalid text>"

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

        log_ap.warning("%s: %s", sev, text)

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


# ========================
# MAVLink / DroneKit helpers
# ========================
def _ack(command_id: str | None, ok: bool, **extra):
    """
        Publish an immediate acknowledgement to TOPIC_ACK.

        This is Stage 1 of the two-ACK architecture: it signals that the command
        has been received and accepted by the drone bridge, before execution
        begins.  The RTT of this message represents pure MQTT network latency.

        The QoS level is read under _qos_lock because _handle_and_ack() may have
        just updated QOS_CMD from the incoming payload on the same thread.
        Extra keyword arguments (e.g. detail="accepted") are merged into the
        payload dict so the GCS gets context alongside the ok flag.
    """
    payload = {"ok": ok}
    if command_id: payload["command_id"] = command_id
    payload.update(extra)
    with _qos_lock:
        qos = QOS_CMD
    mqtt_client.publish(TOPIC_ACK, json.dumps(payload), qos=qos, retain=False)


def set_mode(mode: str):
    """
        Request a flight-mode change and wait up to 6 seconds for confirmation.

        DroneKit's vehicle.mode setter sends a MAVLink SET_MODE command but
        returns immediately; the actual mode switch happens asynchronously on the
        autopilot.  The polling loop verifies that vehicle.mode.name reflects the
        new mode before returning True, which prevents the caller from assuming
        success before the autopilot has acknowledged the change.

        The LANDING flag is set here (rather than inside telemetry_loop) so that
        altitude streaming starts as soon as LAND or RTL is requested, even if
        the autopilot takes a moment to accept the mode.
    """
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
    """
        Arm or disarm the vehicle and poll for up to 8 seconds for confirmation.

        Arming may be rejected by the autopilot if pre-arm checks fail (e.g. no
        GPS fix, RC calibration missing).  The longer timeout (8 s vs 6 s for
        mode changes) accommodates the SITL pre-arm check sequence.
    """
    vehicle.armed = arm
    t0 = time.time()
    while time.time() - t0 < 8:
        if bool(vehicle.armed) == arm:
            return True
        time.sleep(0.2)
    return False


def takeoff(alt: float):
    """
        Execute the full take-off sequence: switch to GUIDED → arm → climb.

        simple_takeoff() sends a MAVLink TAKEOFF command targeted at *alt* metres
        above the home position (relative altitude).  The function returns True as
        soon as the command is accepted; the drone then climbs autonomously.
        telemetry_loop() monitors the climb and streams altitude updates until
        the target is reached (95 % threshold to account for overshoot).

        The TAKE_OFF flag and ALT are stored globally so telemetry_loop() can
        reference the target without additional function arguments.
    """
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
    """
        Command a constant velocity in the NED (North-East-Down) body frame for
        *duration* seconds by repeatedly sending SET_POSITION_TARGET_LOCAL_NED.

        The message is re-sent every 0.2 s because ArduPilot treats velocity
        setpoints as perishable: if no new setpoint arrives within ~3 s it stops
        the maneuver.  Repeating at 5 Hz keeps the drone moving for exactly
        *duration* seconds without requiring a separate stop command.

        The type_mask field (0b0000111111000111) tells the autopilot to use only
        the velocity components (vx, vy, vz) and ignore position and acceleration
        fields in the same message.

        Note: in NED convention, positive vz points downward; callers that want
        upward motion must pass a negative vz (see move_direction_by_distance).
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,  # time_boot_ms, target_system, target_component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # reference frame
        0b0000111111000111,  # type_mask: enable vx, vy, vz only
        0, 0, 0,  # x, y, z position (ignored)
        vx, vy, vz,  # velocity setpoint [m/s]
        0, 0, 0,  # acceleration (ignored)
        0, 0  # yaw, yaw_rate (ignored)
    )
    t0 = time.time()
    while time.time() - t0 < duration:
        vehicle.send_mavlink(msg)
        time.sleep(0.2)
    return True


def move_direction_by_distance(direction: str, speed: float, distance: float):
    """
        Move the drone in one of six cardinal directions by a given distance.

        Translates the high-level (direction, speed, distance) triple into a NED
        velocity command: duration = distance / speed, then delegates to
        send_velocity_ned().

        NED sign conventions applied here:
          - North → +vx,  South → −vx
          - East  → +vy,  West  → −vy
          - Up    → −vz,  Down  → +vz  (NED z-axis points downward)
    """
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
        vz = -speed  # NED: negative vz = upward
    elif d == "down":
        vz = speed
    else:
        raise ValueError("direction must be north/south/east/west/up/down")
    return send_velocity_ned(vx, vy, vz, dur)


def set_yaw(heading: float, rate: float = 30.0, relative: bool = False, cw: bool = True):
    """
        Command a yaw rotation via MAV_CMD_CONDITION_YAW.

        Parameters
        ----------
        heading  : target heading in degrees (absolute or relative to current).
        rate     : rotation rate in deg/s (default 30 deg/s).
        relative : if True, heading is an offset from the current yaw;
                   if False, it is an absolute compass bearing (0 = North).
        cw       : rotation direction — True for clockwise, False for counter-clockwise.

        command_long_encode packages the parameters into a MAVLink COMMAND_LONG
        message targeting the autopilot (system 0, component 0).
        vehicle.flush() ensures the message is dispatched immediately rather than
        being buffered by pymavlink.
    """
    is_relative = 1 if relative else 0
    direction = 1 if cw else -1
    msg = vehicle.message_factory.command_long_encode(
        0, 0,                                  # target_system, target_component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, # command
        0,                                     # confirmation
        heading, rate, direction, is_relative, # param1-4
        0, 0, 0                                # param5-7 (unused)
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()
    return True


def set_speed(spd: float):
    """
        Set both airspeed and groundspeed targets to spd m/s.

        Setting both properties ensures consistent behavior in SITL regardless
        of whether the simulated environment has wind enabled.
    """
    vehicle.airspeed = spd
    vehicle.groundspeed = spd
    return True


def goto_absolute(lat: float, lon: float, alt: float):
    """
        Command the drone to fly to an absolute WGS-84 position.

        LocationGlobalRelative interprets alt as metres above the home point
        (relative altitude), which is the standard reference for ArduPilot
        missions and avoids dependence on a barometric sea-level calibration.
        simple_goto() issues a MAVLink SET_POSITION_TARGET_GLOBAL_INT command.
    """
    tgt = LocationGlobalRelative(lat, lon, alt)
    vehicle.simple_goto(tgt)
    return True


def set_param(name: str, value: Any):
    """Write an ArduPilot parameter by name via DroneKit's parameter dict."""
    vehicle.parameters[name] = value
    return True


def get_param(name: str):
    """Read an ArduPilot parameter by name; returns None if not found."""
    return vehicle.parameters.get(name, None)


# ========================
# MQTT callbacks
# ========================
def _on_heartbeat(client, userdata, msg):
    """
        Update the last-heartbeat timestamp whenever the GCS publishes to
        TOPIC_HEARTBEAT and clear the emergency-landing flag if it was set.

        Called from the Paho network thread; only updates primitives so no
        locking is required (GIL protects float and bool assignments in CPython).
    """
    global _last_heartbeat_time, _emergency_landing_active
    _last_heartbeat_time = time.time()
    _emergency_landing_active = False   # GCS is back online; reset failsafe


def heartbeat_watchdog():
    """
        Background thread that monitors GCS liveness via the heartbeat topic.

        If no heartbeat is received within HEARTBEAT_TIMEOUT seconds *and* the
        drone is currently armed, the watchdog triggers an emergency LAND mode
        to prevent the drone from flying unattended.

        After activating the failsafe, _last_heartbeat_time is reset so the
        watchdog does not keep calling set_mode("LAND") every second.
        The _emergency_landing_active flag additionally suppresses repeated
        activations in the same disconnection event.

        An initial 15-second grace period at startup prevents a false trigger
        before the GCS has had time to connect and start sending heartbeats.

        Note: this watchdog is currently disabled (thread start is commented out
        in main()) and is intended for activation when transitioning from SITL
        to real-hardware flights.
    """
    global _last_heartbeat_time, _emergency_landing_active
    time.sleep(15)  # grace period: allow GCS to connect before monitoring starts
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
                # Reset timer to suppress repeated LAND commands
                _last_heartbeat_time = time.time()
        time.sleep(1.0)


def on_connect(client, userdata, flags, rc):
    """
        Called by Paho when the broker connection is established.

        Publishes the retained "online" status so any GCS that subscribes later
        immediately learns the drone is reachable.  The Will (LWT) set in main()
        will overwrite this with "offline" if the drone disconnects ungracefully.

        The heartbeat callback registration is commented out because the watchdog
        is disabled for SITL; it should be enabled for real-hardware flights.
    """
    print(f"[MQTT] Connected rc={rc} host={BROKER_HOST} port={BROKER_PORT}")
    client.publish(TOPIC_STATUS, "online", qos=1, retain=True)
    client.subscribe(TOPIC_CMD, qos=QOS_CMD)
    client.subscribe(TOPIC_HEARTBEAT, qos=0)
    # enable for real hardware
    #client.message_callback_add(TOPIC_HEARTBEAT, _on_heartbeat)
    client.subscribe(TOPIC_MAV_TX, qos=0)


def on_disconnect(client, userdata, rc):
    print(f"[MQTT] Disconnected rc={rc}")


def on_message(client, userdata, msg):
    """
        Entry point for all incoming MQTT messages not handled by a per-topic
        callback.

        Two message types are handled:
          1. TOPIC_MAV_TX: raw MAVLink frames encoded as base64.  These are
             decoded and written directly to the autopilot's serial/UDP link,
             bypassing the JSON command layer entirely.  This allows a GCS to
             send any MAVLink message that DroneKit does not expose via its API.

          2. All other topics (primarily TOPIC_CMD): JSON command payloads.
             Each command is dispatched to a new daemon thread via
             _handle_and_ack().  This is critical: long-running commands such as
             "move" block for several seconds while the drone travels.  Executing
             them on the Paho network thread would prevent the client from
             processing any other MQTT messages (including further commands or
             keep-alive PINGs) for the entire duration, which could cause the
             broker to drop the connection.
    """
    print(f"[MQTT] RX topic={msg.topic} payload={msg.payload[:120]!r}")

    if msg.topic == TOPIC_MAV_TX:
        try:
            raw = base64.b64decode(msg.payload)
            vehicle._master.write(raw)
        except Exception as e:
            print("[RAW] write failed:", e)
        return

    try:
        p = json.loads(msg.payload.decode("utf-8"))
        # Spawn a worker thread so the Paho network loop is never blocked
        # by command execution time.
        threading.Thread(target=_handle_and_ack, args=(p,), daemon=True).start()
    except Exception as e:
        print("[CMD] error:", e)


def _handle_and_ack(p: Dict[str, Any]):
    """
        Execute a command and publish both ACK stages from a worker thread.

        This function implements the two-ACK architecture:
          1. Immediate ACK ("accepted") — sent before handle_command() is called.
             Its RTT measures pure MQTT latency, independent of execution time.
          2. Completion notification — sent after handle_command() returns.
             Its RTT measures MQTT latency + drone-side execution time.

        The QoS override embedded in the payload (if present) is applied here
        under _qos_lock before either publish, so both ACKs use the level that
        the GCS requested for this specific exchange.

        Running in a daemon thread means that if main() exits, this thread is
        killed automatically without needing explicit cleanup.
    """
    global QOS_CMD
    cid = p.get("command_id")

    # Apply per-command QoS override if the GCS included one in the payload.
    if "qos" in p:
        with _qos_lock:
            QOS_CMD = int(p["qos"])

    try:
        # Stage 1: immediate acceptance ACK (pure network latency).
        _ack(cid, True, detail="accepted")

        # Execute the command (may block for seconds on long maneuvers).
        ok, detail = handle_command(p)

        # Stage 2: completion notification (network + execution latency).
        mqtt_client.publish(
            TOPIC_COMPLETED,
            json.dumps({"command_id": cid, "ok": ok, **detail}),
            qos=QOS_CMD,
            retain=False
        )
    except Exception as e:
        _ack(cid, False, error=str(e))


def handle_command(p: Dict[str, Any]):
    """
        Dispatch a parsed JSON command dict to the appropriate MAVLink helper.

        Returns a (ok: bool, detail: dict) tuple that is forwarded to the GCS
        in the completion notification.  All exceptions are caught by the caller
        (_handle_and_ack) and reported as a failed ACK, so individual branches
        here may raise freely on invalid arguments.
    """
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


# ========================
# Telemetry loop
# ========================
def telemetry_loop():
    """
        Background thread that publishes real-time telemetry to the GCS.

        Two telemetry streams are managed here:

        Altitude stream (TOPIC_TEL_ALT, QoS 0):
          Active only during take-off and landing transitions, not during normal
          flight, to avoid flooding the broker with redundant data.
          - During take-off: streams altitude until the drone reaches 95 % of the
            target (ALT).  The 5 % margin accounts for ArduPilot's climb
            overshoot before it settles at the setpoint.
          - During landing: streams altitude while the drone is above 1 m AGL.
            Below 1 m the ground effect and sensor noise make the reading
            unreliable, so streaming stops and LANDING is cleared.

        Battery snapshot (TOPIC_TEL_BAT, QoS 0):
          Published every BAT_LOG_EVERY second.  Coarser than the SYS_STATUS
          listener (which logs every message), this gives the GCS a periodic
          overview without requiring a query command.

        The loop sleeps 0.5 s between iterations, giving a 2 Hz update rate for
        altitude data, which is sufficient for monitoring climb/descent progress.
    """
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
                    # Still climbing: stream current altitude to the GCS.
                    mqtt_client.publish(
                        TOPIC_TEL_ALT,
                        json.dumps({"altitude": alt}),
                        qos=QOS_TEL
                    )
                else:
                    # Target altitude reached: stop streaming.
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


# ========================
# Entry point
# ========================
def main():
    """
        Initialize the drone bridge and run until interrupted.

        Start-up sequence:
          1. Connect to ArduPilot/SITL via DroneKit (blocks until the vehicle
             reports ready or the 60-second timeout expires).
          2. Register MAVLink message listeners for battery and STATUSTEXT.
          3. Build and connect the Paho MQTT client with mutual-TLS credentials.
             The Last Will Testament (LWT) is registered *before* connecting so
             the broker will publish "offline" automatically if the TCP connection
             drops without a clean DISCONNECT packet.
          4. Start the Paho network loop (loop_start) on a background thread.
          5. Start the telemetry thread.
          6. Block the main thread in a 0.5-second sleep loop until KeyboardInterrupt.

        Shutdown sequence (finally block):
          - Clear running to signal background threads to exit.
          - Publish an explicit "offline" status (belt-and-suspenders alongside LWT).
          - Stop the Paho loop and disconnect cleanly.
          - Close the DroneKit vehicle to release the MAVLink connection.
    """
    global vehicle, mqtt_client, running
    print(f"[UAV] Connecting SITL {SITL_LINK} ...")
    vehicle = connect(SITL_LINK, wait_ready=True, timeout=60)
    print("[UAV] Vehicle connected.")
    setup_battery_logging(vehicle)
    setup_statustext_forward(vehicle)

    mqtt_client = mqtt.Client(client_id=CLIENT_ID, clean_session=True)
    # LWT: if the bridge crashes or loses connectivity, the broker publishes
    # "offline" on TOPIC_STATUS so the GCS knows immediately.
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

    # enable for real hardware
    #threading.Thread(target=heartbeat_watchdog, daemon=True).start()

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