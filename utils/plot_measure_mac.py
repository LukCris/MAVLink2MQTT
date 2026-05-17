import pandas as pd
import matplotlib.pyplot as plt
import sys, os

# ========================
# CONFIG
# ========================
LOG_DIR = "../logs/dist-42cm_tls-on_qos0_5ghz"
PLOTS_DIR = "../42cm_plots_QoS0/5ghz"

# Stessi range fissi degli altri grafici
AXIS = {
    "rssi_noise": {"y": (-100, -20)},   # dBm
    "snr":        {"y": (0, 80)},        # dB
}

def ensure_plots_dir(path):
    os.makedirs(path, exist_ok=True)

def parse_mac_rssi(path):
    df = pd.read_csv(path)
    if df.empty:
        return df
    df = df.sort_values("t_s").reset_index(drop=True)
    df["t_rel_s"] = df["t_s"] - df["t_s"].iloc[0]
    return df

def plot_rssi_noise_snr(df, outdir, prefix="mac_wifi"):
    if df.empty:
        print("[MAC] Nessun dato da plottare")
        return

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    # --- Subplot 1: RSSI e Noise ---
    ax1.plot(df["t_rel_s"], df["rssi_dbm"],   label="RSSI",       color="steelblue")
    ax1.plot(df["t_rel_s"], df["noise_dbm"],  label="Noise floor", color="tomato", linestyle="--")
    ax1.set_ylabel("Level [dBm]")
    ax1.set_title("RSSI and Noise Floor vs Time")
    ax1.set_ylim(AXIS["rssi_noise"]["y"])
    ax1.legend()
    ax1.grid(True)

    # --- Subplot 2: SNR ---
    ax2.plot(df["t_rel_s"], df["snr_db"], label="SNR", color="seagreen")
    ax2.set_ylabel("SNR [dB]")
    ax2.set_xlabel("Time [s]")
    ax2.set_title("Signal-to-Noise Ratio vs Time")
    ax2.set_ylim(AXIS["snr"]["y"])
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    out_path = os.path.join(outdir, f"{prefix}_rssi_noise_snr.png")
    plt.savefig(out_path, dpi=300)
    plt.close()
    print(f"[MAC] Salvato: {out_path}")


def main():
    ensure_plots_dir(PLOTS_DIR)
    path = os.path.join(LOG_DIR, "mac_rssi_noise.csv")
    if os.path.exists(path):
        df = parse_mac_rssi(path)
        print(f"[MAC] Campioni: {len(df)}")
        plot_rssi_noise_snr(df, PLOTS_DIR)
    else:
        print(f"[MAC] File non trovato: {path}")

if __name__ == "__main__":
    main()