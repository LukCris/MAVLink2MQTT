# plot_scenario.py
"""
Generate per-QoS comparison plots for all measurement scenarios found in
LOGS_DIR.  For each metric (ICMP RTT, MQTT RTT, completion latency, TCP
throughput, UDP throughput) a single figure is produced with one subplot per
Wi-Fi band, each curve representing a different GCS-to-drone distance.

All axes share the same scale across bands so that 2.4 GHz and 5 GHz panels
can be compared visually at a glance.
"""

import matplotlib as mpl
import json
import os
import re
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

# ========================
# Global matplotlib style
# ========================
mpl.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
    "font.size": 9,
    "axes.titlesize": 10,
    "axes.labelsize": 9,
    "legend.fontsize": 8,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,

    "axes.linewidth": 0.8,
    "grid.linewidth": 0.5,
    "grid.linestyle": "--",

    "lines.linewidth": 1.4,

    "figure.dpi": 300,
    "savefig.dpi": 600,

    "legend.frameon": True,
    "legend.framealpha": 0.9,
})

# ========================
# Configuration
# ========================

LOGS_DIR = "../logs"
OUT_DIR = "../plots/qos_0"

QOS = 0  # only folders whose name contains qos<QOS> are processed

# Mapping from folder band token → human-readable label used in plot titles.
BAND_TOKENS = {
    "2_4": "2.4 GHz",
    "5_8": "5.8 GHz",
}

# File names expected inside each scenario folder.
PING_LOG = "ping_icmp.log"
LATENCY_CSV = "latency_metrics.csv"
COMPLETION_CSV = "completion_metrics.csv"
IPERF_TCP_JSON = "iperf_tcp.json"
IPERF_UDP_JSON = "iperf_udp_10M.json"

# Optional manual upper bound for the X axis of each metric plot.
# Setting a value truncates the axis to improve readability when a few
# outliers would otherwise compress the relevant portion of the distribution.
# Set to None to use automatic scaling.
# Index mapping:
#   0 → ICMP RTT CDF       [ms]
#   1 → MQTT RTT CDF       [ms]
#   2 → Completion RTT CDF [ms]
#   3 → TCP Throughput     [s]
#   4 → UDP Throughput     [s]
XMAX_OVERRIDES = {
    0: None,   # ICMP RTT CDF
    1: None,   # MQTT RTT CDF
    2: None,   # Completion RTT CDF
    3: None,   # TCP Throughput
    4: None,   # UDP Throughput
}

# ========================
# Scenario discovery
# ========================

def _dist_to_meters(dist_str: str) -> float:
    """
        Convert a distance token from the folder name to metres for numeric sorting.

        Folder names encode distance as e.g. "42cm" or "5m".  Sorting
        lexicographically would place "16m" before "5m"; converting to a common
        unit (metres) before sorting gives the correct physical order.
        Returns inf for unrecognized formats so they sort to the end.
    """

    m = re.fullmatch(r"([0-9]+(?:\.[0-9]+)?)(cm|m)", dist_str.lower())

    if not m:
        return float("inf")

    v, u = float(m.group(1)), m.group(2)

    return v / 100.0 if u == "cm" else v


def discover_scenarios(logs_dir: str, qos: int, band_tokens: dict) -> dict:
    """
        Scan logs_dir and return all scenario folders that match the naming
        convention and the requested QoS level.

        Expected folder name format:
            dist-<distance>_tls-on_qos<level>_<band>
        Example:
            dist-5m_tls-on_qos1_2_4

        The result is a nested dict:
            { band_token: { dist_label: full_folder_path, ... }, ... }

        Distance labels within each band are sorted in ascending physical order
        (42 cm < 5 m < 16 m) using _dist_to_meters() so that legend entries and
        plot curves appear in a meaningful sequence.
    """

    pattern = re.compile(
        r"^dist-(?P<dist>[^_]+)_tls-on_qos(?P<qos>\d+)_(?P<band>.+)$"
    )

    result = {bt: {} for bt in band_tokens}

    for entry in os.listdir(logs_dir):

        full = os.path.join(logs_dir, entry)

        if not os.path.isdir(full):
            continue

        m = pattern.match(entry)

        if not m:
            continue

        if int(m.group("qos")) != qos:
            continue

        band = m.group("band")
        dist = m.group("dist")

        if band not in band_tokens:
            continue

        result[band][dist] = full

    # Sort each band's distance map in ascending physical order.
    for band in result:

        result[band] = dict(
            sorted(
                result[band].items(),
                key=lambda kv: _dist_to_meters(kv[0])
            )
        )

    return result

# ========================
# Parsers
# ========================

def parse_ping(path: str) -> pd.DataFrame:
    """
        Parse a Linux-style ping log with timestamp-prefixed lines.

        Expected line format:
            [<epoch_s>] 64 bytes from <ip>: icmp_seq=<n> ttl=64 time=<rtt> ms

        Returns a DataFrame with columns: t_s, rtt_ms, t_rel_s.
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

            records.append({
                "t_s": float(m.group("t_s")),
                "rtt_ms": float(m.group("rtt")),
            })

    if not records:
        return pd.DataFrame(columns=["t_s", "rtt_ms", "t_rel_s"])

    df = pd.DataFrame(records).sort_values("t_s").reset_index(drop=True)

    df["t_rel_s"] = df["t_s"] - df["t_s"].iloc[0]

    return df


def parse_latency(path: str) -> pd.DataFrame:
    """Parse the MQTT immediate-ACK latency CSV (latency_metrics.csv)."""
    df = pd.read_csv(path)

    if df.empty:
        return df

    df = df.sort_values("t_s").reset_index(drop=True)

    df["t_rel_s"] = df["t_s"] - df["t_s"].iloc[0]

    return df


def parse_completion(path: str) -> pd.DataFrame:
    """Parse the command completion latency CSV (completion_metrics.csv)."""
    df = pd.read_csv(path)

    if df.empty:
        return df

    df = df.sort_values("t_s").reset_index(drop=True)

    df["t_rel_s"] = df["t_s"] - df["t_s"].iloc[0]

    return df


def parse_iperf_tcp(path: str) -> tuple:
    """
        Parse an iperf3 TCP JSON report.

        Returns (df_intervals, avg_mbps) where df_intervals has columns
        start_s, end_s, mbps for each 1-second reporting interval.
        avg_mbps is taken from the sender summary in the "end" block, which
        is more accurate than averaging the per-interval values.
    """
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    records = []

    for interval in data.get("intervals", []):

        s = interval["sum"]

        records.append({
            "start_s": float(s["start"]),
            "end_s": float(s["end"]),
            "mbps": float(s["bits_per_second"]) / 1e6,
        })

    df = pd.DataFrame(records)

    end = data.get("end", {})

    sender = end.get("streams", [{}])[0].get("sender", {})

    avg_mbps = float(sender.get("bits_per_second", 0.0)) / 1e6

    return df, avg_mbps


def parse_iperf_udp(path: str) -> tuple:
    """
        Parse an iperf3 UDP JSON report.

        The summary block for UDP is nested differently from TCP: iperf3 places
        it under end["streams"][0]["udp"] rather than end["streams"][0]["sender"].
        end["sum"] is used as a fallback when the udp sub-block is absent.
    """
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    records = []

    for interval in data.get("intervals", []):

        s = interval["sum"]

        records.append({
            "start_s": float(s["start"]),
            "end_s": float(s["end"]),
            "mbps": float(s["bits_per_second"]) / 1e6,
        })

    df = pd.DataFrame(records)

    end = data.get("end", {})

    udp = end.get("streams", [{}])[0].get("udp", {})
    summ = end.get("sum", udp)  # fallback to end["sum"] if "udp" key is absent

    avg_mbps = float(summ.get("bits_per_second", 0.0)) / 1e6

    return df, avg_mbps


# ========================
# Plot helpers
# ========================

def _plot_cdf(ax, series: pd.Series, label: str, colour: str):
    """
        Plot an empirical CDF of series on ax.

        The CDF is computed by sorting the values and assigning rank / n to each
        point, giving the fraction of observations ≤ x for each x.
        drawstyle="steps-post" produces the characteristic staircase shape of an
        empirical CDF, where each step rises at the exact observed value.
    """
    sorted_vals = np.sort(series.dropna().values)

    cdf = np.arange(1, len(sorted_vals) + 1) / len(sorted_vals)

    ax.plot(
        sorted_vals,
        cdf,
        linewidth=1.5,
        label=label,
        color=colour,
        drawstyle="steps-post",
    )


def _plot_ts(ax, t, y, label, colour):
    """Plot a time-series line (throughput vs time) on ax."""
    ax.plot(
        t,
        y,
        linewidth=1.5,
        label=label,
        color=colour,
        marker=None,
    )


def _style_ax(ax, ylabel, title, use_cdf):
    """
        Apply consistent axis styling.

        For CDF plots the Y axis is formatted as a percentage (0–100 %) using
        PercentFormatter so that the axis reads "0 %, 25 %, 50 %..." rather than
        "0.0, 0.25, 0.50...".  Minor grid lines at every tick subdivision add
        visual precision without cluttering the plot.
    """
    ax.set_title(title, fontsize=10)

    ax.set_ylabel(ylabel, fontsize=9)

    if use_cdf:

        ax.set_xlabel("Latency [ms]", fontsize=9)
        # xmax=1.0 tells the formatter that 1.0 corresponds to 100 %.
        ax.yaxis.set_major_formatter(
            ticker.PercentFormatter(xmax=1.0, decimals=0)
        )

    else:

        ax.set_xlabel("Time [s]", fontsize=9)

    ax.grid(True, which="major", linestyle="--", linewidth=0.5, alpha=0.6)
    ax.grid(True, which="minor", linestyle=":", linewidth=0.4, alpha=0.3)

    ax.minorticks_on()

    ax.tick_params(labelsize=8)

# ========================
# Global axis limits
# ========================

def compute_global_limits(scenarios, metric_row):
    """
        Compute X and Y axis limits that encompass all scenarios for a given metric.

        Iterating over every scenario before plotting ensures that sharex/sharey
        axes use a common scale determined by the actual data rather than by the
        arbitrary order in which scenarios are plotted.

        A 5 % padding is added on each side of the data range so that curves do
        not touch the axis borders.  If XMAX_OVERRIDES defines a cap for this
        metric, x_max is clamped to that value after padding, which effectively
        clips outliers that would otherwise compress the bulk of the distribution.

        Returns (x_min, x_max, y_min, y_max).
    """
    x_min = float("inf")
    x_max = float("-inf")

    y_min = float("inf")
    y_max = float("-inf")

    for band_token in scenarios:

        for _, folder in scenarios[band_token].items():

            try:

                # ICMP RTT CDF
                if metric_row == 0:

                    path = os.path.join(folder, PING_LOG)

                    if os.path.exists(path):

                        df = parse_ping(path)

                        vals = df["rtt_ms"].dropna()

                        if len(vals):

                            x_min = min(x_min, vals.min())
                            x_max = max(x_max, vals.max())

                            y_min = 0
                            y_max = 1

                # MQTT RTT CDF
                elif metric_row == 1:

                    path = os.path.join(folder, LATENCY_CSV)

                    if os.path.exists(path):

                        df = parse_latency(path)

                        vals = df["rtt_ms"].dropna()

                        if len(vals):

                            x_min = min(x_min, vals.min())
                            x_max = max(x_max, vals.max())

                            y_min = 0
                            y_max = 1

                # COMPLETION RTT CDF
                elif metric_row == 2:

                    path = os.path.join(folder, COMPLETION_CSV)

                    if os.path.exists(path):

                        df = parse_latency(path)

                        vals = df["rtt_ms"].dropna()

                        if len(vals):
                            x_min = min(x_min, vals.min())
                            x_max = max(x_max, vals.max())

                            y_min = 0
                            y_max = 1


                # TCP
                elif metric_row == 3:

                    path = os.path.join(folder, IPERF_TCP_JSON)

                    if os.path.exists(path):

                        df, _ = parse_iperf_tcp(path)

                        t = (
                            df["start_s"] +
                            df["end_s"]
                        ) / 2.0

                        y = df["mbps"]

                        if len(t):

                            x_min = min(x_min, t.min())
                            x_max = max(x_max, t.max())

                            y_min = min(y_min, y.min())
                            y_max = max(y_max, y.max())

                # UDP
                elif metric_row == 4:

                    path = os.path.join(folder, IPERF_UDP_JSON)

                    if os.path.exists(path):

                        df, _ = parse_iperf_udp(path)

                        t = (
                            df["start_s"] +
                            df["end_s"]
                        ) / 2.0

                        y = df["mbps"]

                        if len(t):

                            x_min = min(x_min, t.min())
                            x_max = max(x_max, t.max())

                            y_min = min(y_min, y.min())
                            y_max = max(y_max, y.max())

            except Exception:
                pass    # skip folders with missing or malformed files

    def _pad(min_v, max_v, pad_ratio=0.05):
        if not np.isfinite(min_v) or not np.isfinite(max_v):
            return min_v, max_v

        if min_v == max_v:
            return min_v - 1, max_v + 1

        pad = (max_v - min_v) * pad_ratio
        return min_v - pad, max_v + pad

    x_min, x_max = _pad(x_min, x_max)
    y_min, y_max = _pad(y_min, y_max)

    # Clamp x_max to the manual override, if configured for this metric.
    override = XMAX_OVERRIDES.get(metric_row)
    if override is not None:
        x_max = float(override)

    return x_min, x_max, y_min, y_max


# ========================
# Figure builder
# ========================

def build_figures(scenarios, band_tokens, out_dir):
    """
        Build and save one figure per metric.

        Each figure has one subplot per Wi-Fi band (columns), with all distances
        overlaid as separate curves in each subplot.  sharex=True and sharey=True
        ensure the axes share the limits computed by compute_global_limits(), so
        the two band panels are directly comparable without manual axis alignment.

        The shared legend is placed below the figure (bbox_to_anchor=(0.5, -0.05))
        rather than inside a subplot, to avoid overlapping curves in either panel.
        rect=[0, 0.08, 1, 1.05] in tight_layout reserves vertical space at the
        bottom for the legend and at the top for the suptitle.
    """
    colours = plt.rcParams["axes.prop_cycle"].by_key()["color"]

    # Descriptor for each metric: output filename, axis labels, and row index
    # used by compute_global_limits() and the plotting dispatch below.
    metric_info = [
        {
            "filename": "icmp_rtt_cdf.png",
            "ylabel": "CDF",
            "title": "ICMP RTT CDF",
            "use_cdf": True,
            "row": 0,
        },
        {
            "filename": "mqtt_rtt_cdf.png",
            "ylabel": "CDF",
            "title": "MQTT RTT CDF",
            "use_cdf": True,
            "row": 1,
        },
        {
            "filename": "completion_latency_cdf.png",
            "ylabel": "CDF",
            "title": "Completion RTT CDF",
            "use_cdf": True,
            "row": 2,
        },
        {
            "filename": "tcp_throughput.png",
            "ylabel": "Throughput [Mbps]",
            "title": "TCP Throughput",
            "use_cdf": False,
            "row": 3,
        },
        {
            "filename": "udp_throughput.png",
            "ylabel": "Throughput [Mbps]",
            "title": "UDP Throughput",
            "use_cdf": False,
            "row": 4,
        },
    ]

    os.makedirs(out_dir, exist_ok=True)

    for metric in metric_info:

        # One column per band; sharex/sharey enforce the global scale.
        x_min, x_max, y_min, y_max = compute_global_limits(
            scenarios,
            metric["row"],
        )

        fig, axes = plt.subplots(
            nrows=1,
            ncols=len(band_tokens),
            figsize=(7.2, 2.6),
            constrained_layout=False,
            sharex=True,
            sharey=True,
        )

        # Wrap a single Axes object in a list for uniform iteration.
        if len(band_tokens) == 1:
            axes = [axes]

        for col_idx, (band_token, band_label) in enumerate(
            band_tokens.items()
        ):

            ax = axes[col_idx]

            dist_map = scenarios.get(band_token, {})

            if not dist_map:
                ax.set_visible(False)
                continue

            # Each distance gets the next colour in the cycle.
            for colour_idx, (dist_label, folder) in enumerate(
                dist_map.items()
            ):

                colour = colours[colour_idx % len(colours)]

                # ICMP RTT CDF
                if metric["row"] == 0:

                    path = os.path.join(folder, PING_LOG)

                    if os.path.exists(path):

                        df = parse_ping(path)

                        if not df.empty:

                            _plot_cdf(
                                ax,
                                df["rtt_ms"],
                                dist_label,
                                colour,
                            )

                # MQTT RTT CDF
                elif metric["row"] == 1:

                    path = os.path.join(folder, LATENCY_CSV)

                    if os.path.exists(path):

                        df = parse_latency(path)

                        if not df.empty:

                            _plot_cdf(
                                ax,
                                df["rtt_ms"],
                                dist_label,
                                colour,
                            )

                # COMPLETION CDF
                elif metric["row"] == 2:

                    path = os.path.join(folder, COMPLETION_CSV)

                    if os.path.exists(path):

                        df = parse_latency(path)

                        if not df.empty:
                            _plot_cdf(
                                ax,
                                df["rtt_ms"],
                                dist_label,
                                colour,
                            )

                # TCP
                elif metric["row"] == 3:

                    path = os.path.join(folder, IPERF_TCP_JSON)

                    if os.path.exists(path):

                        df, _ = parse_iperf_tcp(path)

                        if not df.empty:

                            t_mid = (
                                df["start_s"] +
                                df["end_s"]
                            ) / 2.0

                            _plot_ts(
                                ax,
                                t_mid,
                                df["mbps"],
                                dist_label,
                                colour,
                            )

                # UDP
                elif metric["row"] == 4:

                    path = os.path.join(folder, IPERF_UDP_JSON)

                    if os.path.exists(path):

                        df, _ = parse_iperf_udp(path)

                        if not df.empty:

                            t_mid = (
                                df["start_s"] +
                                df["end_s"]
                            ) / 2.0

                            _plot_ts(
                                ax,
                                t_mid,
                                df["mbps"],
                                dist_label,
                                colour,
                            )

            ax.set_xlim(x_min, x_max)

            if metric["use_cdf"]:
                ax.set_ylim(0, 1)
            else:
                ax.set_ylim(y_min, y_max)

            ax.margins(x=0.02, y=0.05)

            _style_ax(
                ax,
                metric["ylabel"],
                f"{band_label} - {metric['title']}",
                metric["use_cdf"],
            )

        # TCP/UDP throughput is independent of QoS, so the QoS label is
        # omitted from the title to avoid misleading the reader.
        if metric["row"] in [3, 4]:

            suptitle = (
                f"{metric['title']} Comparison Across Distances "
                f"(TLS on)"
            )
        else:

            suptitle = (
                f"{metric['title']} Comparison Across Distances "
                f"(QoS {QOS}, TLS on)"
            )

        # Collect legend handles from the first subplot only; because sharey
        # is active, all subplots have the same set of distance labels.
        handles, labels = axes[0].get_legend_handles_labels()
        fig.legend(
            handles,
            labels,
            title="Distance",
            loc="lower center",
            bbox_to_anchor=(0.5, -0.05),
            ncol=min(len(labels), 5),
            frameon=True,
            fancybox=False,
            edgecolor="black",
            fontsize=8,
            title_fontsize=8,
        )

        fig.suptitle(
            suptitle,
            fontsize=11,
            fontweight="semibold",
        )

        out_path = os.path.join(
            out_dir,
            metric["filename"],
        )

        # rect=[left, bottom, right, top]: bottom=0.08 reserves space for the
        # legend; top=1.05 allows the suptitle to sit above the subplot area.
        plt.tight_layout(rect=[0, 0.08, 1, 1.05])
        fig.savefig(out_path, dpi=600, bbox_inches="tight")
        plt.close(fig)
        print(f"[DONE] Saved: {out_path}")


# ========================
# Entry point
# ========================

def main():

    if not os.path.isdir(LOGS_DIR):

        raise SystemExit(
            f"[ERROR] LOGS_DIR not found: {LOGS_DIR}"
        )

    scenarios = discover_scenarios(
        LOGS_DIR,
        QOS,
        BAND_TOKENS,
    )

    found = sum(len(v) for v in scenarios.values())

    if found == 0:

        raise SystemExit(
            f"[ERROR] No matching scenario folders found."
        )

    for bt, dists in scenarios.items():

        label = BAND_TOKENS.get(bt, bt)

        print(f"[INFO] Band {label}: {list(dists.keys())}")

    build_figures(
        scenarios,
        BAND_TOKENS,
        OUT_DIR,
    )


if __name__ == "__main__":
    main()