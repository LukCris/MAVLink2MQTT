#!/usr/bin/env python3
"""
plot_heatmap.py
---------------
Genera una radio heatmap sovrapposta alla planimetria del laboratorio.

Workflow:
1. Esegui collect_rssi.py durante il survey → rssi_survey.wifi_csv
2. Apri la planimetria in un editor (Preview, GIMP, ecc.) e annota
   le coordinate pixel (x, y) di ogni punto dove hai premuto INVIO
3. Compila la lista SURVEY_POINTS qui sotto
4. Esegui questo script
"""

import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from scipy.interpolate import RBFInterpolator
from PIL import Image

# ===========================================================================
# CONFIGURAZIONE  ← modifica qui
# ===========================================================================

FLOORPLAN_PATH = "../planimetria.jpeg"       # planimetria del laboratorio
SURVEY_CSV     = "../rssi_survey_2_4.csv"     # CSV prodotto da collect_rssi.py
OUTPUT_PATH    = "../heatmap_radio_2_4.png"  # immagine di output

# Banda Wi-Fi — cambia per ogni scenario ("2.4 GHz" o "5 GHz")
# Viene usata solo nel titolo del grafico
WIFI_BAND = "2.4 GHz"

# Scala RSSI fissa — uguale per tutti gli scenari, per confronto diretto
# -40 dBm = segnale ottimo (verde), -80 dBm = segnale debole (rosso)
RSSI_VMIN = -75   # dBm
RSSI_VMAX = -50   # dBm

# Posizione dell'AP/GCS nella planimetria (coordinate pixel)
# Metti None per non visualizzarlo
AP_POS = (127, 202)   # ← aggiorna con le coordinate reali

# Associazione manuale mark → coordinate pixel nella planimetria.
# Ordine: lo stesso ordine in cui hai premuto INVIO durante il survey.
# Formato: (x_pixel, y_pixel)
#
# Esempio (DA SOSTITUIRE con i tuoi valori reali):
SURVEY_POINTS: list[tuple[int, int]] = [
    (101,  642),   # mark #1
    (101, 475),
    (101,  352),
    (234,  352),
    (405, 352),
    (405, 238),
    (234, 238),
    (101, 238),
]

# Parametro dell'interpolazione RBF.
# Valori comuni: "thin_plate_spline", "linear", "cubic", "gaussian"
RBF_KERNEL = "thin_plate_spline"

# Risoluzione della griglia di interpolazione (pixel della planimetria).
# Valori più alti = heatmap più nitida ma più lenta.
GRID_RESOLUTION = 300

# Trasparenza della heatmap (0 = invisibile, 1 = opaco)
HEATMAP_ALPHA = 0.6

# Colormap: "jet" classico RF, oppure "RdYlGn" (rosso=debole, verde=forte)
COLORMAP = "RdYlGn"

# ===========================================================================
# CARICAMENTO DATI
# ===========================================================================

def load_marked_rssi(csv_path: str) -> list[float]:
    """
    Legge il CSV e, per ogni mark, calcola il RSSI medio
    nei 2 secondi precedenti e successivi al mark stesso.
    Restituisce una lista ordinata di valori RSSI (uno per mark).
    """
    df = pd.read_csv(csv_path)
    df = df.sort_values("t_s").reset_index(drop=True)

    marks = df[df["mark"] == 1]
    if marks.empty:
        sys.exit("[ERROR] Nessun mark trovato nel CSV. Hai premuto INVIO durante il survey?")

    rssi_values = []
    window = 2.0  # secondi attorno al mark

    for _, row in marks.iterrows():
        t_mark = row["t_s"]
        nearby = df[
            (df["t_s"] >= t_mark - window) &
            (df["t_s"] <= t_mark + window)
        ]
        mean_rssi = nearby["rssi_dBm"].mean()
        rssi_values.append(mean_rssi)

    return rssi_values


# ===========================================================================
# INTERPOLAZIONE
# ===========================================================================

def interpolate_heatmap(
    points: list[tuple[int, int]],
    values: list[float],
    img_width: int,
    img_height: int,
    resolution: int = GRID_RESOLUTION,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Interpola i valori RSSI su una griglia che copre tutta la planimetria.
    Restituisce (grid_x, grid_y, grid_rssi).
    """
    xy  = np.array(points, dtype=float)     # shape (N, 2)
    val = np.array(values,  dtype=float)    # shape (N,)

    # Griglia di output
    xs = np.linspace(0, img_width,  resolution)
    ys = np.linspace(0, img_height, resolution)
    grid_x, grid_y = np.meshgrid(xs, ys)
    grid_xy = np.column_stack([grid_x.ravel(), grid_y.ravel()])

    # RBF interpolation (scipy >= 1.7)
    rbf = RBFInterpolator(xy, val, kernel=RBF_KERNEL, smoothing=0.5)
    grid_rssi = rbf(grid_xy).reshape(grid_x.shape)

    return grid_x, grid_y, grid_rssi


# ===========================================================================
# PLOT
# ===========================================================================

def plot_heatmap(
    floorplan_path: str,
    points: list[tuple[int, int]],
    rssi_values: list[float],
    output_path: str,
):
    # Carica planimetria
    try:
        img = Image.open(floorplan_path).convert("RGB")
    except FileNotFoundError:
        sys.exit(f"[ERROR] Planimetria non trovata: {floorplan_path}")

    img_w, img_h = img.size
    img_arr = np.array(img)

    # Interpola
    print(f"[INFO] Punti misurati: {len(points)}")
    print(f"[INFO] Interpolazione RBF ({RBF_KERNEL}) su griglia {GRID_RESOLUTION}×{GRID_RESOLUTION}...")
    gx, gy, g_rssi = interpolate_heatmap(points, rssi_values, img_w, img_h)

    # Scala fissa — identica per 2.4 GHz e 5 GHz, confronto diretto in tesi
    vmin = RSSI_VMIN
    vmax = RSSI_VMAX

    # Figura
    fig, ax = plt.subplots(figsize=(10, 8))

    # 1. Planimetria di sfondo
    ax.imshow(img_arr, extent=[0, img_w, img_h, 0], aspect="equal", zorder=0)

    # 2. Heatmap sovrapposta
    hm = ax.pcolormesh(
        gx, gy, g_rssi,
        cmap=COLORMAP,
        vmin=vmin, vmax=vmax,
        alpha=HEATMAP_ALPHA,
        shading="auto",
        zorder=1,
    )

    # 3. Punti di misura reali
    xs_pts = [p[0] for p in points]
    ys_pts = [p[1] for p in points]
    sc = ax.scatter(
        xs_pts, ys_pts,
        c=rssi_values, cmap=COLORMAP, vmin=vmin, vmax=vmax,
        s=80, edgecolors="black", linewidths=0.8,
        zorder=2,
    )

    # Etichette dBm sui punti
    for (x, y), v in zip(points, rssi_values):
        ax.text(
            x, y - 12, f"{v:.0f} dBm",
            fontsize=7, ha="center", color="white",
            fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.2", fc="black", alpha=0.5),
            zorder=3,
        )

    # 4. Marcatore AP/GCS
    if AP_POS is not None:
        ax.plot(*AP_POS, marker="*", markersize=18, color="white",
                markeredgecolor="black", markeredgewidth=0.8, zorder=4)
        ax.text(AP_POS[0] + 12, AP_POS[1] - 12, "AP/GCS",
                fontsize=8, color="white", fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.2", fc="black", alpha=0.6),
                zorder=4)

    # Colorbar
    cbar = fig.colorbar(hm, ax=ax, fraction=0.03, pad=0.02)
    cbar.set_label("RSSI [dBm]", fontsize=11)

    ax.set_title(f"Radio Heatmap — Laboratorio ({WIFI_BAND})", fontsize=13)
    ax.set_xlabel("x [pixel]")
    ax.set_ylabel("y [pixel]")
    ax.set_xlim(0, img_w)
    ax.set_ylim(img_h, 0)   # origine in alto a sinistra (come le immagini)
    ax.set_aspect("equal")

    plt.tight_layout()
    plt.savefig(output_path, dpi=200, bbox_inches="tight")
    plt.close()
    print(f"[INFO] Heatmap salvata: {output_path}")

    # Stampa riepilogo
    print("\n[RIEPILOGO MISURE]")
    for i, ((x, y), v) in enumerate(zip(points, rssi_values), 1):
        print(f"  Mark #{i:2d}  ({x:4d}, {y:4d})  →  {v:.1f} dBm")
    print(f"\n  Media : {np.mean(rssi_values):.1f} dBm")
    print(f"  Min   : {np.min(rssi_values):.1f} dBm")
    print(f"  Max   : {np.max(rssi_values):.1f} dBm")


# ===========================================================================
# MAIN
# ===========================================================================

def main():
    # Carica i valori RSSI dai mark
    rssi_values = load_marked_rssi(SURVEY_CSV)

    # Sanity check
    if len(rssi_values) != len(SURVEY_POINTS):
        print(
            f"[WARNING] Mark nel CSV: {len(rssi_values)}, "
            f"Punti configurati: {len(SURVEY_POINTS)}\n"
            f"  Assicurati che SURVEY_POINTS abbia esattamente {len(rssi_values)} voci."
        )
        # Tronca al minimo per non crashare
        n = min(len(rssi_values), len(SURVEY_POINTS))
        rssi_values   = rssi_values[:n]
        points        = SURVEY_POINTS[:n]
    else:
        points = SURVEY_POINTS

    plot_heatmap(FLOORPLAN_PATH, points, rssi_values, OUTPUT_PATH)


if __name__ == "__main__":
    main()