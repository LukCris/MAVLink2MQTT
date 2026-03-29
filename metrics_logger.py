# metric_logger.py
import csv, time
from pathlib import Path

METRICS_FILE = Path("battery_metrics.csv")
LINK_FILE = Path("link_metrics.csv")
LAT_FILE = Path("latency_metrics.csv")
COMPLETION_FILE = Path("completion_metrics.csv")

# -----------------------------
# Battery
# -----------------------------
def init_battery_logger():
    if not METRICS_FILE.exists():
        with METRICS_FILE.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "voltage_V", "current_A", "remaining_pct", "mAh_consumed"])

def log_battery(voltage_V, current_A, remaining_pct=None, mah_consumed=None):
    with METRICS_FILE.open("a", newline="") as f:
        w = csv.writer(f)
        w.writerow([time.time(), voltage_V, current_A, remaining_pct, mah_consumed])


# -----------------------------
# Latency
# -----------------------------
_pending_latency = {}

def init_latency_logger():
    if not LAT_FILE.exists():
        with LAT_FILE.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "id", "rtt_ms", "lost"])

def record_latency_request(msg_id, t_send_ns):
    _pending_latency[msg_id] = t_send_ns

def record_latency_response(msg_id, t_recv_ns, t_send_ns=None):
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
    t_s = t_send_ns / 1e9
    with LAT_FILE.open("a", newline="") as f:
        w = csv.writer(f)
        w.writerow([t_s, msg_id, "", 1])


# -----------------------------
# Completion
# -----------------------------
_pending_completion = {}

def init_completion_logger():
    if not COMPLETION_FILE.exists():
        with COMPLETION_FILE.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "id", "rtt_ms", "lost"])

def record_completion_request(msg_id, t_send_ns):
    _pending_completion[msg_id] = t_send_ns

def record_completion_response(msg_id, t_recv_ns, t_send_ns=None):
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
    t_s = t_send_ns / 1e9
    with COMPLETION_FILE.open("a", newline="") as f:
        w = csv.writer(f)
        w.writerow([t_s, msg_id, "", 1])