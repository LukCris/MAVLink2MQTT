#!/usr/bin/env python3
import os
import re
import json

import pandas as pd
import matplotlib.pyplot as plt

# ========================
# CONFIG
# ========================

LOG_DIR = "./logs/dist-5m_tls-on_qos1"          # cartella dove stanno i file di log
PLOTS_DIR = "./plots"  # cartella di output per i PNG

PING_LOG = "ping_icmp.log"
LATENCY_CSV = "latency_metrics.csv"
IPERF_TCP_JSON = "iperf_tcp.json"
IPERF_UDP_JSON = "iperf_udp_10M.json"


def ensure_plots_dir(path: str):
    os.makedirs(path, exist_ok=True)


# ========================
# PING PARSER & PLOTS
# ========================

def parse_ping_log(path: str) -> pd.DataFrame:
    """
    Parso un ping 'linux-style' con righe tipo:
    [1764151009.354366] 64 bytes from 10.42.0.20: icmp_seq=1 ttl=64 time=0.418 ms
    Restituisco un DataFrame con colonne:
    - t_s (epoch del log, float)
    - icmp_seq (int)
    - rtt_ms (float)
    - t_rel_s (tempo relativo al primo campione)
    """
    pattern = re.compile(
        r"\[(?P<t_s>\d+\.\d+)\].*icmp_seq=(?P<seq>\d+).*time=(?P<rtt>\d+\.\d+)\s*ms"
    )

    records = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            m = pattern.search(line)
            if not m:
                continue
            t_s = float(m.group("t_s"))
            seq = int(m.group("seq"))
            rtt = float(m.group("rtt"))
            records.append({"t_s": t_s, "icmp_seq": seq, "rtt_ms": rtt})

    if not records:
        return pd.DataFrame(columns=["t_s", "icmp_seq", "rtt_ms", "t_rel_s"])

    df = pd.DataFrame(records).sort_values("t_s").reset_index(drop=True)
    df["t_rel_s"] = df["t_s"] - df["t_s"].iloc[0]
    return df


def plot_ping_rtt(df: pd.DataFrame, outdir: str, prefix: str = "ping"):
    if df.empty:
        print("[PING] Nessun dato da plottare")
        return

    # RTT vs tempo relativo
    plt.figure()
    plt.plot(df["t_rel_s"], df["rtt_ms"])
    plt.xlabel("Time [s]")
    plt.ylabel("ICMP RTT [ms]")
    plt.title("ICMP RTT vs time")
    plt.grid(True)
    plt.tight_layout()
    out_path = os.path.join(outdir, f"{prefix}_rtt_vs_time.png")
    plt.savefig(out_path, dpi=300)
    plt.close()

    # RTT vs icmp_seq (scatter/line)
    plt.figure()
    plt.plot(df["icmp_seq"], df["rtt_ms"])
    plt.xlabel("ICMP sequence")
    plt.ylabel("ICMP RTT [ms]")
    plt.title("ICMP RTT vs sequence")
    plt.grid(True)
    plt.tight_layout()
    out_path = os.path.join(outdir, f"{prefix}_rtt_vs_seq.png")
    plt.savefig(out_path, dpi=300)
    plt.close()


# ========================
# MQTT LATENCY CSV
# ========================

def parse_latency_csv(path: str) -> pd.DataFrame:
    """
    latency_metrics.csv con header:
    t_s,id,rtt_ms,lost
    Restituisco anche t_rel_s.
    """
    df = pd.read_csv(path)
    if df.empty:
        return df

    df = df.sort_values("t_s").reset_index(drop=True)
    df["t_rel_s"] = df["t_s"] - df["t_s"].iloc[0]
    return df


def plot_latency(df: pd.DataFrame, outdir: str, prefix: str = "latency"):
    if df.empty:
        print("[LATENCY] Nessun dato da plottare")
        return

    # RTT per comando (scatter con id)
    plt.figure()
    plt.scatter(df["t_rel_s"], df["rtt_ms"])
    plt.xlabel("Time [s]")
    plt.ylabel("Command RTT [ms]")
    plt.title("MQTT command latency vs time")
    plt.grid(True)
    plt.tight_layout()
    out_path = os.path.join(outdir, f"{prefix}_rtt_vs_time.png")
    plt.savefig(out_path, dpi=300)
    plt.close()

    # Eventuale istogramma RTT
    plt.figure()
    plt.hist(df["rtt_ms"], bins=20)
    plt.xlabel("Command RTT [ms]")
    plt.ylabel("Count")
    plt.title("MQTT command latency distribution")
    plt.grid(True)
    plt.tight_layout()
    out_path = os.path.join(outdir, f"{prefix}_rtt_hist.png")
    plt.savefig(out_path, dpi=300)
    plt.close()


# ========================
# IPERF TCP
# ========================

def parse_iperf_tcp(path: str):
    """
    Parso iperf_tcp.json (iperf3 --json).
    Ritorno:
      df_intervals: per intervallo con colonne:
        start_s, end_s, seconds, bits_per_second, mbps,
        rtt_ms, retransmits
      summary: dict con throughput medio, retrans totali, RTT min/max/mean ecc.
    """
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    intervals = data.get("intervals", [])
    inter_records = []
    for interval in intervals:
        s = interval["sum"]
        start = float(s["start"])
        end = float(s["end"])
        seconds = float(s["seconds"])
        bps = float(s["bits_per_second"])
        retrans = int(s.get("retransmits", 0))

        # prendo i campi rtt dal primo stream, in µs
        stream = interval["streams"][0]
        rtt_us = stream.get("rtt", None)
        if rtt_us is not None:
            rtt_ms = float(rtt_us) / 1000.0
        else:
            rtt_ms = None

        inter_records.append(
            dict(
                start_s=start,
                end_s=end,
                seconds=seconds,
                bits_per_second=bps,
                mbps=bps / 1e6,
                rtt_ms=rtt_ms,
                retransmits=retrans,
            )
        )

    df_intervals = pd.DataFrame(inter_records)

    # summary dal blocco "end"
    end = data.get("end", {})
    sender = end.get("streams", [{}])[0].get("sender", {})
    summary = {
        "seconds": float(sender.get("seconds", 0.0)),
        "bytes": int(sender.get("bytes", 0)),
        "throughput_bps": float(sender.get("bits_per_second", 0.0)),
        "throughput_mbps": float(sender.get("bits_per_second", 0.0)) / 1e6,
        "retransmits": int(sender.get("retransmits", 0)),
        "max_rtt_ms": float(sender.get("max_rtt", 0.0)) / 1000.0,
        "min_rtt_ms": float(sender.get("min_rtt", 0.0)) / 1000.0,
        "mean_rtt_ms": float(sender.get("mean_rtt", 0.0)) / 1000.0,
    }

    return df_intervals, summary


def plot_iperf_tcp(df: pd.DataFrame, summary: dict, outdir: str, prefix: str = "iperf_tcp"):
    if df.empty:
        print("[IPERF TCP] Nessun dato da plottare")
        return

    # Throughput vs time
    t_mid = (df["start_s"] + df["end_s"]) / 2.0

    plt.figure()
    plt.plot(t_mid, df["mbps"])
    plt.xlabel("Time [s]")
    plt.ylabel("Throughput [Mbps]")
    plt.title(
        f"TCP throughput vs time\nAvg ≈ {summary['throughput_mbps']:.1f} Mbps, "
        f"Retransmits = {summary['retransmits']}"
    )
    plt.grid(True)
    plt.tight_layout()
    out_path = os.path.join(outdir, f"{prefix}_throughput.png")
    plt.savefig(out_path, dpi=300)
    plt.close()

    # RTT vs time (se disponibili)
    if df["rtt_ms"].notna().any():
        plt.figure()
        plt.plot(t_mid, df["rtt_ms"])
        plt.xlabel("Time [s]")
        plt.ylabel("RTT [ms]")
        plt.title(
            "TCP RTT (iperf) vs time\n"
            f"min={summary['min_rtt_ms']:.1f} ms, mean={summary['mean_rtt_ms']:.1f} ms, max={summary['max_rtt_ms']:.1f} ms"
        )
        plt.grid(True)
        plt.tight_layout()
        out_path = os.path.join(outdir, f"{prefix}_rtt.png")
        plt.savefig(out_path, dpi=300)
        plt.close()

    # Retransmits per intervallo
    if (df["retransmits"] > 0).any():
        plt.figure()
        plt.bar(t_mid, df["retransmits"], width=0.8)
        plt.xlabel("Time [s]")
        plt.ylabel("Retransmits per interval")
        plt.title("TCP retransmissions per second (iperf)")
        plt.grid(True)
        plt.tight_layout()
        out_path = os.path.join(outdir, f"{prefix}_retransmits.png")
        plt.savefig(out_path, dpi=300)
        plt.close()


# ========================
# IPERF UDP
# ========================

def parse_iperf_udp(path: str):
    """
    Parso iperf_udp_10M.json.
    Ritorno:
      df_intervals: start_s, end_s, seconds, bits_per_second, mbps, packets
      summary: dict con throughput, jitter, loss ecc.
    """
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    intervals = data.get("intervals", [])
    inter_records = []
    for interval in intervals:
        s = interval["sum"]
        start = float(s["start"])
        end = float(s["end"])
        seconds = float(s["seconds"])
        bps = float(s["bits_per_second"])
        packets = int(s.get("packets", 0))

        inter_records.append(
            dict(
                start_s=start,
                end_s=end,
                seconds=seconds,
                bits_per_second=bps,
                mbps=bps / 1e6,
                packets=packets,
            )
        )

    df_intervals = pd.DataFrame(inter_records)

    end = data.get("end", {})
    # iperf3 UDP:  end["streams"][0]["udp"] + end["sum"]
    udp = end.get("streams", [{}])[0].get("udp", {})
    sum_ = end.get("sum", udp)

    summary = {
        "seconds": float(sum_.get("seconds", 0.0)),
        "bytes": int(sum_.get("bytes", 0)),
        "throughput_bps": float(sum_.get("bits_per_second", 0.0)),
        "throughput_mbps": float(sum_.get("bits_per_second", 0.0)) / 1e6,
        "jitter_ms": float(sum_.get("jitter_ms", 0.0)),
        "lost_packets": int(sum_.get("lost_packets", 0)),
        "packets": int(sum_.get("packets", 0)),
        "lost_percent": float(sum_.get("lost_percent", 0.0)),
    }

    return df_intervals, summary


def plot_iperf_udp(df: pd.DataFrame, summary: dict, outdir: str, prefix: str = "iperf_udp_10M"):
    if df.empty:
        print("[IPERF UDP] Nessun dato da plottare")
        return

    t_mid = (df["start_s"] + df["end_s"]) / 2.0

    plt.figure()
    plt.plot(t_mid, df["mbps"])
    plt.xlabel("Time [s]")
    plt.ylabel("Throughput [Mbps]")
    plt.title(
        "UDP throughput vs time (target 10 Mbps)\n"
        f"Avg ≈ {summary['throughput_mbps']:.2f} Mbps, "
        f"jitter ≈ {summary['jitter_ms']:.3f} ms, "
        f"loss = {summary['lost_percent']:.2f}%"
    )
    plt.grid(True)
    plt.tight_layout()
    out_path = os.path.join(outdir, f"{prefix}_throughput.png")
    plt.savefig(out_path, dpi=300)
    plt.close()

# ========================
# MAIN
# ========================

def main():
    ensure_plots_dir(PLOTS_DIR)

    # --- PING ---
    ping_path = os.path.join(LOG_DIR, PING_LOG)
    if os.path.exists(ping_path):
        df_ping = parse_ping_log(ping_path)
        print(f"[PING] Campioni: {len(df_ping)}")
        plot_ping_rtt(df_ping, PLOTS_DIR)
    else:
        print(f"[PING] File non trovato: {ping_path}")

    # --- MQTT latency ---
    lat_path = os.path.join(LOG_DIR, LATENCY_CSV)
    if os.path.exists(lat_path):
        df_lat = parse_latency_csv(lat_path)
        print(f"[LATENCY] Campioni: {len(df_lat)}")
        plot_latency(df_lat, PLOTS_DIR)
    else:
        print(f"[LATENCY] File non trovato: {lat_path}")

    # --- IPERF TCP ---
    tcp_path = os.path.join(LOG_DIR, IPERF_TCP_JSON)
    if os.path.exists(tcp_path):
        df_tcp, sum_tcp = parse_iperf_tcp(tcp_path)
        print("[IPERF TCP] Summary:", sum_tcp)
        plot_iperf_tcp(df_tcp, sum_tcp, PLOTS_DIR)
    else:
        print(f"[IPERF TCP] File non trovato: {tcp_path}")

    # --- IPERF UDP 10M ---
    udp_path = os.path.join(LOG_DIR, IPERF_UDP_JSON)
    if os.path.exists(udp_path):
        df_udp, sum_udp = parse_iperf_udp(udp_path)
        print("[IPERF UDP] Summary:", sum_udp)
        plot_iperf_udp(df_udp, sum_udp, PLOTS_DIR)
    else:
        print(f"[IPERF UDP] File non trovato: {udp_path}")

    print(f"Tutti i grafici (disponibili) sono stati salvati in: {PLOTS_DIR}")


if __name__ == "__main__":
    main()
