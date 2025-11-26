import csv, time
from pathlib import Path

METRICS_FILE = Path("battery_metrics.csv")
LINK_FILE = Path("link_metrics.csv")
LAT_FILE = Path("latency_metrics.csv")

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


def init_latency_logger():
    if not LAT_FILE.exists():
        with LAT_FILE.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "id", "rtt_ms", "lost"])


# -----------------------------
# Latency
# -----------------------------
_pending_latency = {}

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