#!/usr/bin/env python3
import os
import re
import json

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ========================
# CONFIG
# ========================

LOG_DIR = "../logs/"          # cartella dove stanno i file di log
PLOTS_DIR = "../4m_plots_QoS1"  # cartella di output per i PNG

PING_LOG = "ping_icmp.log"
LATENCY_CSV = "latency_metrics.csv"
IPERF_TCP_JSON = "iperf_tcp.json"
IPERF_UDP_JSON = "iperf_udp_10M.json"
WIFI_SNR_LOG = "wifi_snr.log"


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

    rtt = pd.to_numeric(df["rtt_ms"], errors="coerce").astype(float)
    rtt = rtt.dropna()
    if rtt.empty:
        print("[PING] RTT vuoti/non validi")
        return

    # soglia solo per scalare l'asse (non visualizzata)
    p95 = float(np.nanquantile(rtt, 0.95))

    # limiti asse Y → grafico meno "stretto"
    y_low = 0.0
    y_high = max(1.0, p95 * 1.15)

    # =====================
    # RTT vs time
    # =====================
    plt.figure(figsize=(10, 4.5))   # meno alto → più leggibile
    plt.plot(
        df["t_rel_s"],
        df["rtt_ms"],
        linewidth=1.6,
        alpha=0.85
    )

    plt.xlabel("Time [s]")
    plt.ylabel("ICMP RTT [ms]")
    plt.title("ICMP RTT vs time")
    plt.grid(True)

    plt.ylim(y_low, y_high)

    plt.tight_layout()
    out_path = os.path.join(outdir, f"{prefix}_rtt_vs_time.png")
    plt.savefig(out_path, dpi=300)
    plt.close()

    # =====================
    # RTT vs ICMP sequence
    # =====================
    plt.figure(figsize=(10, 4.5))
    plt.plot(
        df["icmp_seq"],
        df["rtt_ms"],
        linewidth=1.6,
        alpha=0.85
    )

    plt.xlabel("ICMP sequence")
    plt.ylabel("ICMP RTT [ms]")
    plt.title("ICMP RTT vs sequence")
    plt.grid(True)

    plt.ylim(y_low, y_high)

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

    df = df.copy()
    df = df.sort_values("t_s").reset_index(drop=True)
    df["t_rel_s"] = df["t_s"] - df["t_s"].iloc[0]

    # forza tipi e normalizza: se lost=1 -> rtt_ms deve essere NaN
    df["lost"] = pd.to_numeric(df["lost"], errors="coerce").fillna(0).astype(int)
    df["rtt_ms"] = pd.to_numeric(df["rtt_ms"], errors="coerce")
    df.loc[df["lost"] == 1, "rtt_ms"] = np.nan

    t = df["t_rel_s"]
    rtt = df["rtt_ms"]
    lost_mask = df["lost"] == 1

    # Serie interpolata SOLO per disegnare la linea continua (trend)
    # (non modifica i punti raw)
    rtt_line = rtt.interpolate(method="linear", limit_area="inside")

    # =====================
    # LINEA + PUNTI + MARKER LOSS
    # =====================
    fig, ax = plt.subplots()

    # linea continua (trend)
    ax.plot(t, rtt_line, linestyle="-", linewidth=2, alpha=0.9,
            label="trend (interp)", zorder=2)

    # punti reali (solo quelli ricevuti)
    ok_mask = ~lost_mask & rtt.notna()
    ax.scatter(t[ok_mask], rtt[ok_mask], marker="o", s=35, alpha=0.9,
               label="received", zorder=3)

    # evidenzia i loss: linee verticali + X in alto
    if lost_mask.any():
        for x in t[lost_mask]:
            ax.axvline(x, linestyle="--", linewidth=1, alpha=0.5)
        # DOPO (corretto)
        ax.scatter(
            t[lost_mask],
            rtt_line[lost_mask],
            marker="x",
            s=80,
            linewidths=3,
            color="crimson",
            label="lost",
            zorder=10
        )

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Command RTT [ms]")
    ax.set_title("MQTT command latency vs time")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, f"{prefix}_rtt_vs_time_line.png"), dpi=300)
    plt.close(fig)

    # =====================
    # SMOOTH (rolling)
    # =====================
    if len(df) >= 5:
        # smooth sul trend (interp) così rimane continuo
        rtt_smooth = rtt_line.rolling(window=3, center=True, min_periods=1).mean()

        fig, ax = plt.subplots()
        ax.plot(t, rtt_line, alpha=0.35, label="trend (interp)")
        ax.scatter(t[ok_mask], rtt[ok_mask], alpha=0.6, label="received")
        ax.plot(t, rtt_smooth, linewidth=2, color="crimson", label="moving avg (3)")

        if lost_mask.any():
            for x in t[lost_mask]:
                ax.axvline(x, linestyle="--", linewidth=1, alpha=0.4)

        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Command RTT [ms]")
        ax.set_title("MQTT command latency (smoothed)")
        ax.grid(True)
        ax.legend()
        fig.tight_layout()
        fig.savefig(os.path.join(outdir, f"{prefix}_rtt_smooth.png"), dpi=300)
        plt.close(fig)

    # =====================
    # BOXPLOT
    # =====================
    rtt = df["rtt_ms"].dropna()
    n = len(rtt)

    plt.figure()
    plt.boxplot(
        rtt,
        vert=True,
        widths=0.4,
        showfliers=True,
        medianprops=dict(linewidth=2),
        boxprops=dict(linewidth=1.5),
        whiskerprops=dict(linewidth=1.5),
        capprops=dict(linewidth=1.5),
    )

    plt.ylabel("Command RTT [ms]")
    plt.title(f"MQTT command latency (boxplot, n={n})")

    # asse X inutile → lo rimuovo
    plt.xticks([])

    plt.grid(axis="y", linestyle="--", alpha=0.6)
    plt.tight_layout()

    out_path = os.path.join(outdir, f"{prefix}_boxplot.png")
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


import os
import numpy as np
import matplotlib.pyplot as plt

def plot_iperf_tcp(df, summary, outdir, prefix="iperf_tcp"):
    if df.empty:
        print("[IPERF TCP] Nessun dato da plottare")
        return

    t_mid = (df["start_s"] + df["end_s"]) / 2.0

    def _prep_ax(ax):
        ax.set_axisbelow(True)
        ax.grid(True, axis="y", linestyle="--", alpha=0.35)
        ax.grid(False, axis="x")
        ax.margins(x=0.02)

    # =====================
    # THROUGHPUT vs time
    # =====================
    thr = df["mbps"].astype(float)
    thr_mean = float(summary.get("throughput_mbps", np.nan))
    thr_p05 = float(thr.quantile(0.05))
    thr_p95 = float(thr.quantile(0.95))

    fig, ax = plt.subplots(figsize=(10, 6))

    ax.plot(t_mid, thr, linewidth=2.4, label="Throughput", color="#1f77b4")
    ax.axhline(thr_mean, linestyle="--", linewidth=1.8, color="0.3",
               label=f"Mean: {thr_mean:.1f} Mbps")
    ax.axhline(thr_p95, linestyle=":", linewidth=1.8, color="#9467bd",
               label=f"p95: {thr_p95:.1f} Mbps")
    ax.axhline(thr_p05, linestyle=":", linewidth=1.8, color="#8c564b",
               label=f"p05: {thr_p05:.1f} Mbps")

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Throughput [Mbps]")
    ax.legend(loc="upper left", frameon=True, framealpha=0.9)

    fig.suptitle("TCP throughput vs time", fontsize=15, y=0.98)
    fig.text(
        0.5, 0.945,
        f"Avg ≈ {thr_mean:.1f} Mbps, Retransmits = {summary.get('retransmits', 0)}",
        ha="center", va="top", fontsize=11
    )

    _prep_ax(ax)
    fig.tight_layout(rect=[0, 0, 1, 0.92])
    fig.savefig(os.path.join(outdir, f"{prefix}_throughput.png"), dpi=300)
    plt.close(fig)

    # =====================
    # RTT vs time
    # =====================
    if df["rtt_ms"].notna().any():
        rtt = df["rtt_ms"].astype(float)
        rtt_mean = float(summary.get("mean_rtt_ms", np.nan))
        rtt_p95 = float(rtt.quantile(0.95))
        spike_mask = rtt > rtt_p95

        fig, ax = plt.subplots(figsize=(10, 6))

        ax.plot(t_mid, rtt, linewidth=2.4, label="RTT", color="#1f77b4")
        ax.axhline(rtt_mean, linestyle="--", linewidth=1.8, color="0.3",
                   label=f"Mean: {rtt_mean:.1f} ms")
        ax.axhline(rtt_p95, linestyle=":", linewidth=1.8, color="#9467bd",
                   label=f"p95: {rtt_p95:.1f} ms")

        if spike_mask.any():
            ax.scatter(
                t_mid[spike_mask], rtt[spike_mask],
                marker="x", s=160, linewidths=3,
                color="crimson", label="Spike (>p95)", zorder=10
            )

        ax.set_xlabel("Time [s]")
        ax.set_ylabel("RTT [ms]")
        ax.legend(loc="upper left", frameon=True, framealpha=0.9)

        fig.suptitle("TCP RTT (iperf) vs time", fontsize=15, y=0.98)
        fig.text(
            0.5, 0.945,
            f"min={summary.get('min_rtt_ms', np.nan):.1f} ms, "
            f"mean={summary.get('mean_rtt_ms', np.nan):.1f} ms, "
            f"max={summary.get('max_rtt_ms', np.nan):.1f} ms",
            ha="center", va="top", fontsize=11
        )

        if np.isfinite(rtt.max()):
            ax.set_ylim(bottom=0, top=rtt.max() * 1.08)

        _prep_ax(ax)
        fig.tight_layout(rect=[0, 0, 1, 0.92])
        fig.savefig(os.path.join(outdir, f"{prefix}_rtt.png"), dpi=300)
        plt.close(fig)

        # =====================
        # SCATTER: RTT vs Throughput
        # =====================
        if df["rtt_ms"].notna().any():
            thr = df["mbps"].astype(float).to_numpy()
            rtt = df["rtt_ms"].astype(float).to_numpy()

            # spike definito in modo non arbitrario (p95 RTT)
            rtt_p95 = float(np.nanquantile(rtt, 0.95))
            spike_mask = rtt > rtt_p95

            # correlazione Pearson
            valid = np.isfinite(thr) & np.isfinite(rtt)
            pearson_r = np.corrcoef(thr[valid], rtt[valid])[0, 1]

            fig, ax = plt.subplots(figsize=(10, 6))

            # punti "normali"
            ax.scatter(thr[~spike_mask], rtt[~spike_mask], s=60, alpha=0.85, label="samples")

            # punti spike
            if spike_mask.any():
                ax.scatter(thr[spike_mask], rtt[spike_mask],
                           marker="x", s=160, linewidths=3, color="crimson",
                           label="RTT spike (>p95)", zorder=10)

            ax.set_xlabel("Throughput [Mbps]")
            ax.set_ylabel("RTT [ms]")

            fig.suptitle("RTT vs Throughput (iperf TCP)", fontsize=15, y=0.98)
            fig.text(
                0.5, 0.91,
                f"Spike threshold: p95 RTT = {rtt_p95:.1f} ms | "
                f"Pearson r = {pearson_r:.2f} (weak correlation)",
                ha="center", va="top", fontsize=11
            )

            ax.set_axisbelow(True)
            ax.grid(True, axis="both", linestyle="--", alpha=0.25)
            ax.legend(loc="upper right", frameon=True, framealpha=0.9)

            fig.tight_layout(rect=[0, 0, 1, 0.92])
            out_path = os.path.join(outdir, f"{prefix}_rtt_vs_throughput.png")
            fig.savefig(out_path, dpi=300)
            plt.close(fig)


# ========================
# IPERF UDP
# ========================

def parse_iperf_udp(path: str):
    """
    Parso iperf_udp.json.
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
# SNR
# ========================

def parse_wifi_snr_log(path: str) -> pd.DataFrame:
    """
    wifi_snr.log CSV-like:
    ts,signal_dbm,noise_db,est_snr_db
    1765988299.127822567,-38,-69,31
    ...

    Ritorna DF con:
      ts (float), signal_dbm (float), noise_db (float), est_snr_db (float), t_rel_s (float)
    """
    df = pd.read_csv(path)

    # normalizza nomi (nel caso cambi)
    expected = {"ts", "signal_dbm", "noise_db", "est_snr_db"}
    if not expected.issubset(set(df.columns)):
        raise ValueError(f"[WIFI SNR] Colonne attese: {expected}, trovate: {set(df.columns)}")

    # tipi
    for c in ["ts", "signal_dbm", "noise_db", "est_snr_db"]:
        df[c] = pd.to_numeric(df[c], errors="coerce")

    df = df.dropna(subset=["ts", "est_snr_db"]).sort_values("ts").reset_index(drop=True)
    if df.empty:
        return df

    df["t_rel_s"] = df["ts"] - df["ts"].iloc[0]
    return df


def plot_wifi_snr(
    df: pd.DataFrame,
    outdir: str,
    prefix: str = "wifi_snr",
    warn_db: float = 25.0,
    bad_db: float = 20.0
):
    if df.empty:
        print("[WIFI SNR] Nessun dato da plottare")
        return

    # --- palette soglie (DISTINTE) ---
    # warn: arancione, bad: rosso (ben distinguibili)
    WARN_C = "#ff7f0e"
    BAD_C  = "#d62728"

    snr = df["est_snr_db"].astype(float)
    t = df["t_rel_s"].astype(float)

    snr_min = float(np.nanmin(snr))
    snr_mean = float(np.nanmean(snr))
    snr_max = float(np.nanmax(snr))
    snr_p95 = float(np.nanquantile(snr, 0.95))

    def _prep_ax(ax):
        ax.set_axisbelow(True)
        ax.grid(True, axis="y", linestyle="--", alpha=0.35)
        ax.grid(False, axis="x")
        ax.margins(x=0.02)

    # =========================
    # 1) SNR vs time
    # =========================
    fig, ax = plt.subplots(figsize=(10, 6))

    ax.plot(t, snr, linewidth=2.4, label="Estimated SNR")

    # soglie con colori distinti
    #ax.axhline(warn_db, linestyle="--", linewidth=2.0, color=WARN_C, label=f"warn {warn_db:.0f} dB")
    #ax.axhline(bad_db,  linestyle="--", linewidth=2.0, color=BAD_C,  label=f"bad {bad_db:.0f} dB")

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Estimated SNR [dB]")
    ax.legend(loc="upper right", frameon=True, framealpha=0.9)

    fig.suptitle("WiFi estimated SNR vs time", fontsize=15, y=0.98)
    fig.text(
        0.5, 0.945,
        f"min={snr_min:.1f} dB | mean={snr_mean:.1f} dB | max={snr_max:.1f} dB | p95={snr_p95:.1f} dB",
        ha="center", va="top", fontsize=11
    )

    _prep_ax(ax)
    fig.tight_layout(rect=[0, 0, 1, 0.92])
    fig.savefig(os.path.join(outdir, f"{prefix}_snr_vs_time.png"), dpi=300)
    plt.close(fig)

    # =========================
    # 2) ECDF
    # =========================
    x = np.sort(snr.dropna().to_numpy())
    y = np.arange(1, len(x) + 1) / len(x)

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.plot(x, y, linewidth=2.6, label="ECDF")

    ax.axvline(warn_db, linestyle="--", linewidth=2.0, color=WARN_C, label=f"warn {warn_db:.0f} dB")
    ax.axvline(bad_db,  linestyle="--", linewidth=2.0, color=BAD_C,  label=f"bad {bad_db:.0f} dB")

    ax.set_xlabel("Estimated SNR [dB]")
    ax.set_ylabel("ECDF")
    ax.set_title("WiFi SNR ECDF (cumulative distribution)", pad=10)
    ax.legend(loc="lower right", frameon=True, framealpha=0.9)

    ax.set_axisbelow(True)
    ax.grid(True, axis="both", linestyle="--", alpha=0.25)
    ax.margins(x=0.02)

    fig.tight_layout()
    fig.savefig(os.path.join(outdir, f"{prefix}_snr_ecdf.png"), dpi=300)
    plt.close(fig)

    # =========================
    # 3) Outage timeline (SOLO SE SERVE)
    # =========================
    warn_mask = snr < warn_db
    bad_mask = snr < bad_db

    if warn_mask.any() or bad_mask.any():
        fig, ax = plt.subplots(figsize=(10, 3.2))

        # visualizzazione “timeline”: 2 righe, punti dove succede
        if warn_mask.any():
            ax.scatter(t[warn_mask], np.ones(warn_mask.sum()) * 1.0, s=25, color=WARN_C, label=f"warn < {warn_db:.0f} dB")
        if bad_mask.any():
            ax.scatter(t[bad_mask],  np.ones(bad_mask.sum())  * 0.0, s=25, color=BAD_C,  label=f"bad < {bad_db:.0f} dB")

        ax.set_yticks([1.0, 0.0])
        ax.set_yticklabels([f"warn < {warn_db:.0f} dB", f"bad < {bad_db:.0f} dB"])
        ax.set_xlabel("Time [s]")
        ax.set_title("WiFi SNR outage timeline", pad=10)

        ax.set_axisbelow(True)
        ax.grid(True, axis="x", linestyle="--", alpha=0.25)
        ax.grid(False, axis="y")
        ax.legend(loc="upper right", frameon=True, framealpha=0.9)

        fig.tight_layout()
        fig.savefig(os.path.join(outdir, f"{prefix}_snr_outage.png"), dpi=300)
        plt.close(fig)
    else:
        print("[WIFI SNR] Nessun campione sotto soglia: salto outage timeline.")

    # =========================
    # 4) Signal vs time (noise costante → non plottato)
    # =========================
    if "signal_dbm" in df.columns:
        fig, ax = plt.subplots(figsize=(10, 6))

        sig = df["signal_dbm"].astype(float)

        ax.plot(t, sig, linewidth=2.4, label="Signal [dBm]", color="#1f77b4")

        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Signal level [dBm]")
        ax.set_title("WiFi signal level vs time", pad=10)

        ymin = sig.min() - 3
        ymax = sig.max() + 3
        ax.set_ylim(ymin, ymax)

        ax.legend(loc="best", frameon=True, framealpha=0.9)

        ax.set_axisbelow(True)
        ax.grid(True, axis="y", linestyle="--", alpha=0.35)
        ax.grid(False, axis="x")
        ax.margins(x=0.02)

        fig.tight_layout()
        fig.savefig(os.path.join(outdir, f"{prefix}_signal_vs_time.png"), dpi=300)
        plt.close(fig)


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

    # --- WIFI SNR ---
    snr_path = os.path.join(LOG_DIR, WIFI_SNR_LOG)
    if os.path.exists(snr_path):
        df_snr = parse_wifi_snr_log(snr_path)
        print(f"[WIFI SNR] Campioni: {len(df_snr)}")
        plot_wifi_snr(df_snr, PLOTS_DIR, prefix="wifi_snr", warn_db=25.0, bad_db=20.0)
    else:
        print(f"[WIFI SNR] File non trovato: {snr_path}")


if __name__ == "__main__":
    main()
