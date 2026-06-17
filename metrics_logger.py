# metric_logger.py
import csv, time
from pathlib import Path

# ========================
# Output file paths
# ========================
# Each metric category writes to its own CSV file so that post-processing
# scripts can read them independently without filtering by type.
METRICS_FILE    = Path("battery_metrics.csv")
LINK_FILE       = Path("link_metrics.csv")      # reserved for future link-layer metrics
LAT_FILE        = Path("latency_metrics.csv")   # immediate-ACK RTT (pure MQTT latency)
COMPLETION_FILE = Path("completion_metrics.csv") # end-to-end RTT (MQTT + execution time)

# ========================
# Battery logger
# ========================
def init_battery_logger():
    """
        Create the battery CSV file with its header row if it does not exist.

        The existence check prevents overwriting data from a previous run when
        the module is re-imported or the logger is re-initialized within the
        same session.
    """
    if not METRICS_FILE.exists():
        with METRICS_FILE.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "voltage_V", "current_A", "remaining_pct"])

def log_battery(voltage_V, current_A, remaining_pct=None):
    """
        Append one battery snapshot row to the CSV.

        Called from the SYS_STATUS MAVLink listener on every message, so the
        sampling rate matches the autopilot's SYS_STATUS broadcast rate.
        Fields may be None when the autopilot reports the corresponding sentinel
        value (0xFFFF / 0xFF), indicating the measurement is unavailable.
    """
    with METRICS_FILE.open("a", newline="") as f:
        w = csv.writer(f)
        w.writerow([time.time(), voltage_V, current_A, remaining_pct])


# ========================
# Latency logger (immediate ACK)
# ========================
# Tracks the RTT between the GCS publishing a command and receiving the
# drone's immediate "accepted" ACK.  This measures pure MQTT network latency,
# independent of how long the command takes to execute on the drone.

# In-memory map of msg_id → t_send_ns, used to correlate a response with its
# original request when the response arrives asynchronously on a callback thread.
_pending_latency = {}

def init_latency_logger():
    """Create the latency CSV with its header row if it does not exist."""
    if not LAT_FILE.exists():
        with LAT_FILE.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "id", "rtt_ms", "lost"])

def record_latency_request(msg_id, t_send_ns):
    """
        Store the send timestamp (nanoseconds) for a command identified by *msg_id*.

        Called in publish_cmd() just before the MQTT publish so that t_send_ns
        captures the moment the message leaves the GCS application layer.
    """
    _pending_latency[msg_id] = t_send_ns

def record_latency_response(msg_id, t_recv_ns, t_send_ns=None):
    """
        Compute the RTT and append a successful latency row to the CSV.

        RTT is computed in milliseconds from nanosecond timestamps for sufficient
        precision at the sub-millisecond scale typical of local Wi-Fi links:
            rtt_ms = (t_recv_ns - t_send_ns) / 1e6

        t_s is the wall-clock time of reception (in seconds since epoch), used
        as the time axis in the latency-vs-time plots.

        The 'lost' column is written as 0 (not lost) to distinguish successful
        exchanges from timeouts in the same CSV file.
    """
    if t_send_ns is None:
        t_s = t_recv_ns / 1e9
        rtt_ms = None  # o calcolato fuori
    else:
        rtt_ms = (t_recv_ns - t_send_ns) / 1e6
        t_s = t_recv_ns / 1e9
    with LAT_FILE.open("a", newline="") as f:
        w = csv.writer(f)
        w.writerow([t_s, msg_id, rtt_ms, 0])

def record_latency_timeout(msg_id, t_send_ns):
    """
        Record a timed-out command as a lost sample in the latency CSV.

        rtt_ms is left empty and 'lost' is set to 1 so that post-processing
        can distinguish timeouts from successful exchanges and compute the
        packet-loss ratio independently of the RTT distribution.
        t_s is the send time (not receive time) because no response arrived.
    """
    t_s = t_send_ns / 1e9
    with LAT_FILE.open("a", newline="") as f:
        w = csv.writer(f)
        w.writerow([t_s, msg_id, "", 1])


# ========================
# Completion logger (end-to-end latency)
# ========================
# Tracks the RTT between the GCS publishing a command and receiving the
# drone's completion notification on TOPIC_COMPLETED.  This measures
# MQTT latency + drone-side execution time, giving the total command
# round-trip as perceived by the GCS operator.

_pending_completion = {}

def init_completion_logger():
    """Create the completion CSV with its header row if it does not exist."""
    if not COMPLETION_FILE.exists():
        with COMPLETION_FILE.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "id", "rtt_ms", "lost"])

def record_completion_request(msg_id, t_send_ns):
    """
        Store the send timestamp for a command whose completion will be tracked.

        Uses the same t_send_ns as record_latency_request() so that the two
        RTT measurements share a common reference point and their difference
        directly equals the drone-side execution time.
    """
    _pending_completion[msg_id] = t_send_ns

def record_completion_response(msg_id, t_recv_ns, t_send_ns=None):
    """
        Compute the end-to-end RTT and append a row to the completion CSV.

        The schema is identical to the latency CSV so that the same
        post-processing scripts (plot_gcs_metrics.py) can read both files
        with the same parser.
    """
    if t_send_ns is None:
        t_s = t_recv_ns / 1e9
        rtt_ms = None
    else:
        rtt_ms = (t_recv_ns - t_send_ns) / 1e6
        t_s = t_recv_ns / 1e9
    with COMPLETION_FILE.open("a", newline="") as f:
        w = csv.writer(f)
        w.writerow([t_s, msg_id, rtt_ms, 0])

def record_completion_timeout(msg_id, t_send_ns):
    """
       Record a completion notification that never arrived as a lost sample.

       Analogous to record_latency_timeout(); kept as a separate function so
       that latency and completion timeouts can be logged independently even
       if one arrives and the other does not.
    """
    t_s = t_send_ns / 1e9
    with COMPLETION_FILE.open("a", newline="") as f:
        w = csv.writer(f)
        w.writerow([t_s, msg_id, "", 1])