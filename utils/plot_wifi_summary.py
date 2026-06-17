# plot_wifi_cdf.py
"""
Auto-discovers CoreWLAN CSV files from a single directory using the naming
convention:

    rssi_<band>_<distance>.csv
    e.g.  rssi_2.4ghz_42cm.csv
          rssi_5ghz_2m.csv
          rssi_5.8ghz_5m.csv

For each band found, one PNG is produced containing three vertically
stacked subplots showing empirical CDFs:
    1. RSSI [dBm]
    2. Noise Floor [dBm]
    3. SNR [dB]
Each subplot shows one CDF curve per test distance.

Expected CSV format (header on first line):
    t_s,rssi_dbm,noise_dbm,snr_db

Usage
-----
    python3 plot_wifi_cdf.py
"""

import os
import re
import numpy as np
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from collections import defaultdict

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

CSV_DIR = "../wifi_csv"   # directory containing all rssi_*.csv files
OUT_DIR = "../wifi_plots" # output directory for the generated PNGs

# ========================
# Discovery helpers
# ========================

# Regex to validate and parse filenames of the form rssi_<band>_<dist>.csv.
# The band group captures the frequency token (e.g. "2.4ghz", "5.8ghz") and
# the dist group captures the distance label (e.g. "42cm", "3m").
# re.IGNORECASE allows both "GHz" and "ghz" spellings.
_FILE_RE = re.compile(
    r"^rssi_(?P<band>[0-9.]+ghz)_(?P<dist>[^.]+)\.csv$",
    re.IGNORECASE,
)

# Maps raw band tokens from filenames to human-readable labels for titles.
_BAND_LABELS = {
    "2.4ghz": "2.4 GHz",
    "5.8ghz": "5.8 GHz",
}


def _dist_to_meters(dist_str: str) -> float:
    """
        Convert a distance string to metres for numeric sorting.

        Lexicographic sorting would place "16m" before "2m"; converting to a
        common unit before sorting gives the correct physical order.
        Returns inf for unrecognized formats so they sort to the end.
    """
    m = re.fullmatch(r"([0-9]+(?:\.[0-9]+)?)(cm|m)", dist_str.lower())
    if not m:
        return float("inf")
    value, unit = float(m.group(1)), m.group(2)
    return value / 100.0 if unit == "cm" else value


def discover_files(csv_dir: str) -> dict:
    """
        Scan csv_dir for files matching the naming convention and group them
        by band.

        defaultdict(list) is used so that a new empty list is created
        automatically for each band token encountered, avoiding an explicit
        existence check before appending.

        Returns:
            { band_token: [(dist_label, full_path), ...] }
        Entries within each band are sorted by ascending physical distance.
    """
    groups = defaultdict(list)

    for fname in os.listdir(csv_dir):
        m = _FILE_RE.match(fname)
        if not m:
            continue
        band = m.group("band").lower()
        dist = m.group("dist")
        groups[band].append((dist, os.path.join(csv_dir, fname)))

    for band in groups:
        groups[band].sort(key=lambda t: _dist_to_meters(t[0]))

    return dict(groups)


# ========================
# Parser
# ========================

def parse_rssi_csv(path: str) -> pd.DataFrame:
    """
        Parse a CoreWLAN CSV file produced by measure_mac_snr.py.

        A regex-based line parser is used instead of pd.read_csv() because the
        files may or may not have a header row (depending on how the script was
        invoked) and may contain malformed lines.  The regex matches only lines
        where all four fields are valid numbers, automatically skipping the header
        ("t_s,rssi_dbm,...") and any incomplete or corrupted rows without raising
        an exception.

        Returns a DataFrame with columns: t_s, rssi_dbm, noise_dbm, snr_db,
        sorted chronologically.  Returns an empty DataFrame if no valid lines
        are found.
    """
    line_re = re.compile(
        r"^(?P<t>[0-9]+(?:\.[0-9]+)?)"
        r",(?P<rssi>-?[0-9]+(?:\.[0-9]+)?)"
        r",(?P<noise>-?[0-9]+(?:\.[0-9]+)?)"
        r",(?P<snr>-?[0-9]+(?:\.[0-9]+)?)$"
    )

    records = []
    with open(path, "r", encoding="utf-8") as fh:
        for line in fh:
            m = line_re.match(line.strip())
            if not m:
                continue    # silently skip header and malformed lines
            records.append({
                "t_s":       float(m.group("t")),
                "rssi_dbm":  float(m.group("rssi")),
                "noise_dbm": float(m.group("noise")),
                "snr_db":    float(m.group("snr")),
            })

    if not records:
        return pd.DataFrame(columns=["t_s", "rssi_dbm", "noise_dbm", "snr_db"])

    return pd.DataFrame(records).sort_values("t_s").reset_index(drop=True)


# ========================
# Global axis limits
# ========================
def compute_global_axis_limits(groups: dict):
    """
        Compute X-axis limits for RSSI, noise, and SNR that span all bands and
        distances, so that the same scale is used in every panel across all
        output figures.

        np.floor(min - 2) and np.ceil(max + 2) add a 2-unit margin and snap to
        integer dBm values, giving clean axis tick positions without leaving
        data points at the very edge of the plot area.

        Returns:
            { "rssi_dbm": (vmin, vmax), "noise_dbm": (vmin, vmax), "snr_db": (vmin, vmax) }
    """

    metrics = {
        "rssi_dbm": [],
        "noise_dbm": [],
        "snr_db": [],
    }

    for _, entries in groups.items():

        for _, path in entries:

            df = parse_rssi_csv(path)

            if df.empty:
                continue

            for key in metrics:
                metrics[key].extend(df[key].dropna().values)

    limits = {}

    for key, values in metrics.items():

        arr = np.asarray(values)

        vmin = np.floor(arr.min() - 2)
        vmax = np.ceil(arr.max() + 2)

        limits[key] = (vmin, vmax)

    return limits


# ========================
# Plot helpers
# ========================

def _plot_cdf(ax: plt.Axes, series: pd.Series, label: str, colour: str):
    """
        Plot an empirical CDF of series on ax as a staircase curve.

        Each point (x, y) on the curve means: y% of the observations are ≤ x.
        drawstyle="steps-post" makes the step rise at the exact observed value,
        which is the correct visual representation of an empirical CDF.
    """

    vals = np.sort(series.dropna().values)

    cdf = np.arange(1, len(vals) + 1) / len(vals)

    ax.plot(
        vals,
        cdf,
        linewidth=1.5,
        label=label,
        color=colour,
        drawstyle="steps-post",
    )


def _style(ax: plt.Axes, xlabel: str, title: str):
    """
        Apply consistent axis formatting to a single CDF subplot.

        PercentFormatter(xmax=1.0) converts the raw [0, 1] CDF values to
        percentage labels (0 %, 25 %, ..., 100 %) on the Y axis.
        Minor grid lines (dotted, lower alpha) add visual precision between
        the major gridlines without cluttering the plot.
    """
    ax.set_title(title, fontsize=10)

    ax.set_xlabel(xlabel, fontsize=9)

    ax.set_ylabel("CDF", fontsize=9)

    ax.set_ylim(0, 1)

    ax.yaxis.set_major_formatter(
        ticker.PercentFormatter(xmax=1.0, decimals=0)
    )

    ax.grid(
        True,
        which="major",
        linestyle="--",
        linewidth=0.5,
        alpha=0.6,
    )

    ax.grid(
        True,
        which="minor",
        linestyle=":",
        linewidth=0.4,
        alpha=0.3,
    )

    ax.minorticks_on()

    ax.tick_params(labelsize=8)


# ========================
# Per-band figure
# ========================
def plot_band(
    band_token: str,
    entries: list,
    out_dir: str,
    global_limits: dict,
) -> None:
    """
        Produce and save a three-panel CDF figure for one Wi-Fi band.

        The three panels (RSSI, noise floor, SNR) share the X-axis limits
        computed by compute_global_axis_limits(), so figures for different bands
        can be placed side-by-side in the thesis and compared directly.

        sharex=False is intentional: RSSI, noise and SNR span different value
        ranges (e.g. RSSI −90 to −40 dBm, SNR 0 to 50 dB) so linking their
        X axes would force an inappropriate common scale.

        The shared legend is placed at the bottom of the figure
        (bbox_to_anchor=(0.5, 0)) with rect=[0, 0.05, 1, 1] in tight_layout
        reserving a 5 % margin at the bottom so the legend does not overlap
        the lowest subplot's X-axis label.

        Parameters
        ----------
        band_token    : raw token from the filename, e.g. '5.8ghz'.
        entries       : [(dist_label, csv_path), ...] sorted by distance.
        out_dir       : directory where the output PNG is saved.
        global_limits : dict returned by compute_global_axis_limits().
    """
    band_label = _BAND_LABELS.get(band_token, band_token.upper())

    # Load and validate all CSV files for this band.
    frames = []
    for dist_label, path in entries:
        df = parse_rssi_csv(path)
        if df.empty:
            print(f"[WARN] No valid data in {path}, skipping.")
            continue
        frames.append((dist_label, df))
        print(
            f"[INFO] {band_label} / {dist_label}: {len(df)} samples  "
            f"RSSI={df['rssi_dbm'].median():.0f} dBm  "
            f"SNR={df['snr_db'].median():.0f} dB  (medians)"
        )

    if not frames:
        print(f"[WARN] No data for band {band_label}, skipping figure.")
        return

    # Three vertically stacked subplots, one per RF metric.
    fig, (ax_rssi, ax_noise, ax_snr) = plt.subplots(
        nrows=3,
        ncols=1,
        figsize=(7.2, 6.8),
        sharex=False,    # axes have different units; do not share X
        constrained_layout=False,
    )

    fig.suptitle(
        f"Wi-Fi Signal Quality CDFs ({band_label})",
        fontsize=11,
        fontweight="semibold",
    )

    colours = plt.rcParams["axes.prop_cycle"].by_key()["color"]

    # Plot all three metrics for each distance in a single pass so that
    # the color assigned to each distance is consistent across panels.
    for idx, (dist_label, df) in enumerate(frames):
        c = colours[idx % len(colours)]
        _plot_cdf(ax_rssi, df["rssi_dbm"],  dist_label, c)
        _plot_cdf(ax_noise, df["noise_dbm"], dist_label, c)
        _plot_cdf(ax_snr,  df["snr_db"],    dist_label, c)

    _style(ax_rssi, "RSSI [dBm]",        "Received Signal Strength (RSSI)")
    _style(ax_noise, "Noise Floor [dBm]", "Noise Floor")
    _style(ax_snr,  "SNR [dB]",          "Signal-to-Noise Ratio (SNR)")

    # Apply global limits so all band figures share the same X scale.
    ax_rssi.set_xlim(global_limits["rssi_dbm"])
    ax_noise.set_xlim(global_limits["noise_dbm"])
    ax_snr.set_xlim(global_limits["snr_db"])

    # Collect legend handles from the first panel; all panels have the
    # same set of distance labels so only one set of handles is needed.
    handles, labels = ax_rssi.get_legend_handles_labels()
    fig.legend(
        handles,
        labels,
        title="Distance",
        loc="lower center",
        bbox_to_anchor=(0.5, 0),    # centred at the bottom of the figure
        ncol=min(len(labels), 5),
        frameon=True,
        fancybox=False,
        edgecolor="black",
        fontsize=8,
        title_fontsize=8,
    )

    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, f"wifi_cdf_{band_token}.png")

    # rect=[left, bottom, right, top]: bottom=0.05 reserves space for
    # the figure-level legend below the bottom subplot.
    plt.tight_layout(rect=[0, 0.05, 1, 1])
    fig.savefig(
        out_path,
        dpi=600,
        bbox_inches="tight",
    )
    plt.close(fig)
    print(f"[INFO] Saved: {out_path}")


# ========================
# Entry point
# ========================

def main():
    if not os.path.isdir(CSV_DIR):
        raise SystemExit(f"[ERROR] CSV_DIR not found: {CSV_DIR}")

    groups = discover_files(CSV_DIR)

    # Compute global axis limits before plotting so every figure uses
    # the same scale regardless of the order in which bands are processed.
    global_limits = compute_global_axis_limits(groups)

    if not groups:
        raise SystemExit(
            f"[ERROR] No files matching 'rssi_<band>_<distance>.csv' "
            f"found in {CSV_DIR}"
        )

    for band_token, entries in sorted(groups.items()):
        label = _BAND_LABELS.get(band_token, band_token.upper())
        print(f"[INFO] Band {label}: {[d for d, _ in entries]}")

    for band_token, entries in sorted(groups.items()):
        plot_band(
            band_token,
            entries,
            OUT_DIR,
            global_limits,
        )

    print(f"\n[DONE] All figures saved to: {OUT_DIR}")


if __name__ == "__main__":
    main()