# plot_heatmap.py
"""
Generate a Wi-Fi RSSI radio heatmap overlaid on the laboratory floor plan.

Workflow:
1. Run collect_rssi.py during the physical survey  →  rssi_survey.csv
2. Open the floor plan in an image editor (Preview, GIMP, etc.) and note
   the pixel coordinates (x, y) of each point where ENTER was pressed.
3. Fill in the SURVEY_POINTS list below with those coordinates.
4. Run this script.
"""

import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from scipy.interpolate import RBFInterpolator
from PIL import Image
import matplotlib as mpl

# ========================
# Global matplotlib style
# ========================
# Times New Roman / Times is standard for academic publications;
# fallback to DejaVu Serif if neither is available on the system.
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

    "figure.dpi": 300,
    "savefig.dpi": 600,
})

# ========================
# CONFIGURATION
# ========================

FLOORPLAN_PATH = "../Planimetria_Celso.jpeg"  # laboratory floor plan image
SURVEY_CSV     = "../rssi_survey_5_8.csv"     # CSV produced by collect_rssi.py
OUTPUT_PATH    = "../heatmap_radio_5_8.png"   # output image path

# Wi-Fi band label — used only in the plot title, not for any computation.
WIFI_BAND = "5.8 GHz"

# Fixed RSSI color scale — identical across all scenarios so that heatmaps
# from different bands or distances can be compared visually at a glance.
# -58 dBm ≈ good signal (green end), -90 dBm ≈ weak signal (red end).
RSSI_VMIN = -90   # dBm
RSSI_VMAX = -58   # dBm

# Pixel coordinates of the AP/GCS in the floor plan image.
# Set to None to suppress the marker.
AP_POS = (168, 303)

# Manual mapping: mark index → pixel coordinates in the floor plan.
# Order must match the order in which ENTER was pressed during the survey.
# Format: (x_pixel, y_pixel)  —  origin is the top-left corner of the image.
SURVEY_POINTS: list[tuple[int, int]] = [
    (1155,  813),   # mark #1
    (899, 691),
    (1155,  385),
    (758,  385),
    (537, 360),
    (348, 360),
    (168, 360),
    (90, 111),
    (168, 460),
    (348, 460),
    (537, 460),
    (99, 635),
    (99, 772),
]

# RBF kernel for spatial interpolation.
# "thin_plate_spline" minimizes the bending energy of a thin plate fitted
# through the data points; it is C2-smooth and well-suited to sparse,
# irregularly spaced RF measurements.
# Other options: "linear", "cubic", "gaussian".
RBF_KERNEL = "thin_plate_spline"

# Number of grid points along each axis for the interpolated heatmap.
# Higher values produce a sharper image but increase computation time.
GRID_RESOLUTION = 300

# Heatmap overlay opacity (0 = fully transparent, 1 = fully opaque).
HEATMAP_ALPHA = 0.6

# Color map: "RdYlGn" maps weak signal (red) → medium (yellow) → strong (green),
# which matches the intuitive traffic-light convention used in RF coverage maps.
COLORMAP = "RdYlGn"

# ========================
# DATA LOADING
# ========================

def load_marked_rssi(csv_path: str) -> list[float]:
    """
        Extract one representative RSSI value per survey mark from the CSV.

        Rather than using the single sample at the exact moment ENTER was pressed
        (which may be noisy), the function averages all samples within a ±2-second
        window around each mark timestamp.  This reduces the impact of short-term
        fading fluctuations while still capturing the RF conditions at that
        physical position.

        Parameters
        ----------
        csv_path : path to the CSV produced by collect_rssi.py.

        Returns
        -------
        List of mean RSSI values in dBm, one per mark, in chronological order.
    """
    df = pd.read_csv(csv_path)
    df = df.sort_values("t_s").reset_index(drop=True)

    marks = df[df["mark"] == 1]
    if marks.empty:
        sys.exit("[ERROR] Nessun mark trovato nel CSV. Hai premuto INVIO durante il survey?")

    rssi_values = []
    window = 2.0  # seconds on each side of the mark timestamp

    for _, row in marks.iterrows():
        t_mark = row["t_s"]
        nearby = df[
            (df["t_s"] >= t_mark - window) &
            (df["t_s"] <= t_mark + window)
        ]
        mean_rssi = nearby["rssi_dBm"].mean()
        rssi_values.append(mean_rssi)

    return rssi_values


# ========================
# INTERPOLATION
# ========================

def interpolate_heatmap(
    points: list[tuple[int, int]],
    values: list[float],
    img_width: int,
    img_height: int,
    resolution: int = GRID_RESOLUTION,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
        Interpolate RSSI values over a regular grid covering the entire floor plan.

        A Radial Basis Function (RBF) interpolator is used because the survey
        points are irregularly spaced and relatively sparse; RBF handles this case
        gracefully without requiring a structured grid as input.

        The smoothing parameter (0.5) allows a slight deviation from exact
        interpolation at the data points, trading off fidelity at known locations
        for a smoother overall surface — appropriate given the measurement noise
        inherent in Wi-Fi RSSI sampling.

        Coordinate convention:
          - x increases to the right (image columns).
          - y increases downward (image rows), matching PIL/NumPy image layout.
        The returned grids use the same convention, so they can be passed directly
        to pcolormesh with the image displayed via imshow.

        Parameters
        ----------
        points     : list of (x, y) pixel coordinates of the survey marks.
        values     : RSSI value [dBm] at each point.
        img_width  : floor plan image width in pixels.
        img_height : floor plan image height in pixels.
        resolution : number of grid points along each axis.

        Returns
        -------
        (grid_x, grid_y, grid_rssi) — 2-D arrays of shape (resolution, resolution).
    """
    xy  = np.array(points, dtype=float)     # shape (N, 2)
    val = np.array(values,  dtype=float)    # shape (N,)

    xs = np.linspace(0, img_width,  resolution)
    ys = np.linspace(0, img_height, resolution)
    grid_x, grid_y = np.meshgrid(xs, ys)
    grid_xy = np.column_stack([grid_x.ravel(), grid_y.ravel()])

    # Flatten the grid to (resolution², 2) for the RBF query, then reshape back.
    rbf = RBFInterpolator(xy, val, kernel=RBF_KERNEL, smoothing=0.5)
    grid_rssi = rbf(grid_xy).reshape(grid_x.shape)

    return grid_x, grid_y, grid_rssi


# ========================
# PLOTTING
# ========================

def plot_heatmap(
    floorplan_path: str,
    points: list[tuple[int, int]],
    rssi_values: list[float],
    output_path: str,
):
    """
        Compose and save the final heatmap figure.

        Layer order (zorder):
          0 — floor plan background image
          1 — semi-transparent RSSI heatmap (pcolormesh)
          2 — survey measurement points (scatter)
          3 — dBm value labels on each point
          4 — AP/GCS position marker and label
    """
    try:
        img = Image.open(floorplan_path).convert("RGB")
    except FileNotFoundError:
        sys.exit(f"[ERROR] Planimetria non trovata: {floorplan_path}")

    img_w, img_h = img.size
    img_arr = np.array(img)

    print(f"[INFO] Punti misurati: {len(points)}")
    print(f"[INFO] Interpolazione RBF ({RBF_KERNEL}) su griglia {GRID_RESOLUTION}×{GRID_RESOLUTION}...")
    gx, gy, g_rssi = interpolate_heatmap(points, rssi_values, img_w, img_h)

    vmin = RSSI_VMIN
    vmax = RSSI_VMAX

    fig, ax = plt.subplots(
        figsize=(7.2, 5.4),
        constrained_layout=False,
    )

    # Layer 0: floor plan background.
    # extent=[0, img_w, img_h, 0] maps the image to pixel coordinates with
    # the origin at the top-left, matching the (x→right, y→down) convention
    # used for SURVEY_POINTS.
    ax.imshow(img_arr, extent=[0, img_w, img_h, 0], aspect="equal", zorder=0)

    # Layer 1: interpolated RSSI heatmap.
    # pcolormesh is used instead of imshow for the heatmap because it accepts
    # explicit coordinate grids (gx, gy), making it straightforward to align
    # the interpolated surface with the floor plan pixel coordinates.
    hm = ax.pcolormesh(
        gx, gy, g_rssi,
        cmap=COLORMAP,
        vmin=vmin, vmax=vmax,
        alpha=HEATMAP_ALPHA,
        shading="auto",
        zorder=1,
    )

    # Layer 2: actual measurement points, colored by RSSI using the same
    # scale as the heatmap so they are visually consistent.
    xs_pts = [p[0] for p in points]
    ys_pts = [p[1] for p in points]
    sc = ax.scatter(
        xs_pts, ys_pts,
        c=rssi_values, cmap=COLORMAP, vmin=vmin, vmax=vmax,
        s=55, edgecolors="black", linewidths=0.6,
        zorder=2,
    )

    # Layer 3: dBm labels, placed 12 pixels above each point to avoid
    # overlapping the scatter marker.  A semi-transparent black background
    # improves legibility over the floor plan image.
    for (x, y), v in zip(points, rssi_values):
        ax.text(
            x, y - 12, f"{v:.0f} dBm",
            fontsize=6.5,
            ha="center", color="white",
            fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.2",
                      fc="black", alpha=0.45),
            zorder=3,
        )

    # Layer 4: AP/GCS position marker.
    if AP_POS is not None:
        ax.plot(*AP_POS, marker="*",
                markersize=14,
                color="white",
                markeredgecolor="black", markeredgewidth=0.8, zorder=4)
        ax.text(AP_POS[0] + 12, AP_POS[1] - 12, "AP/GCS",
                fontsize=7,
                color="white", fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.2", fc="black", alpha=0.6),
                zorder=4)

    # Color bar: fraction and pad control its width and distance from the axes.
    cbar = fig.colorbar(hm, ax=ax, fraction=0.03, pad=0.02)
    cbar.set_label("RSSI [dBm]", fontsize=9)
    cbar.ax.tick_params(labelsize=8)

    ax.set_title(
        f"Wi-Fi RSSI Heatmap ({WIFI_BAND})",
        fontsize=11,
        fontweight="semibold",
    )

    # Remove tick marks: pixel coordinates have no physical meaning to the reader.
    ax.set_xticks([])
    ax.set_yticks([])

    ax.set_xlim(0, img_w)
    # y-axis is inverted (origin top-left) to match the image coordinate system.
    ax.set_ylim(img_h, 0)
    ax.set_aspect("equal")

    # Remove the surrounding box for a cleaner academic figure style.
    for spine in ax.spines.values():
        spine.set_visible(False)

    # rect=[0, 0, 1, 0.97] leaves a small margin at the top so the title
    # is not clipped by tight_layout.
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.savefig(
        output_path,
        dpi=600,
        bbox_inches="tight",
    )
    plt.close()
    print(f"[INFO] Heatmap salvata: {output_path}")

    # Per-point summary for quick sanity-checking after the run.
    print("\n[RIEPILOGO MISURE]")
    for i, ((x, y), v) in enumerate(zip(points, rssi_values), 1):
        print(f"  Mark #{i:2d}  ({x:4d}, {y:4d})  →  {v:.1f} dBm")
    print(f"\n  Media : {np.mean(rssi_values):.1f} dBm")
    print(f"  Min   : {np.min(rssi_values):.1f} dBm")
    print(f"  Max   : {np.max(rssi_values):.1f} dBm")


# ========================
# MAIN
# ========================

def main():
    rssi_values = load_marked_rssi(SURVEY_CSV)

    # Guard against a mismatch between the number of marks in the CSV and the
    # number of entries in SURVEY_POINTS (e.g. an accidental double ENTER press
    # or a missing coordinate entry).  Rather than crashing, truncate to the
    # shorter of the two lists and warn the user.
    if len(rssi_values) != len(SURVEY_POINTS):
        print(
            f"[WARNING] Mark nel CSV: {len(rssi_values)}, "
            f"Punti configurati: {len(SURVEY_POINTS)}\n"
            f"  Assicurati che SURVEY_POINTS abbia esattamente {len(rssi_values)} voci."
        )
        n = min(len(rssi_values), len(SURVEY_POINTS))
        rssi_values   = rssi_values[:n]
        points        = SURVEY_POINTS[:n]
    else:
        points = SURVEY_POINTS

    plot_heatmap(FLOORPLAN_PATH, points, rssi_values, OUTPUT_PATH)


if __name__ == "__main__":
    main()