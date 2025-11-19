import pandas as pd
import matplotlib.pyplot as plt

# ---------- BATTERIA ----------
df_batt = pd.read_csv("battery_metrics.csv")  # usa l'header già presente

qos = input("QoS level used:")

# Forza i tipi numerici (per sicurezza)
df_batt["t_s"] = pd.to_numeric(df_batt["t_s"], errors="coerce")
df_batt["voltage_V"] = pd.to_numeric(df_batt["voltage_V"], errors="coerce")
df_batt["current_A"] = pd.to_numeric(df_batt["current_A"], errors="coerce")
df_batt["mAh_consumed"] = pd.to_numeric(df_batt["mAh_consumed"], errors="coerce")
df_batt["remaining_pct"] = pd.to_numeric(df_batt["remaining_pct"], errors="coerce")

# Elimina eventuali righe sporche (NaN)
df_batt = df_batt.dropna(subset=["t_s"])

# Tempo relativo (secondi dall'inizio)
t0 = df_batt["t_s"].iloc[0]
df_batt["t_rel_s"] = df_batt["t_s"] - t0

# --- Plot 1: Battery voltage ---
plt.figure()
plt.plot(df_batt["t_rel_s"], df_batt["voltage_V"])
plt.xlabel("Time [s]")
plt.ylabel("Voltage [V]")
plt.title(f"Battery Voltage Over Time with QoS {qos}")
plt.grid(True)
plt.tight_layout()
plt.savefig("battery_voltage.png")

# --- Plot 2: Battery current ---
plt.figure()
plt.plot(df_batt["t_rel_s"], df_batt["current_A"])
plt.xlabel("Time [s]")
plt.ylabel("Current [A]")
plt.title(f"Battery Current Over Time with QoS {qos}")
plt.grid(True)
plt.tight_layout()
plt.savefig("battery_current.png")

# --- Plot 3: Battery mAh consumed ---
plt.figure()
plt.plot(df_batt["t_rel_s"], df_batt["mAh_consumed"])
plt.xlabel("Time [s]")
plt.ylabel("Energy Consumed [mAh]")
plt.title(f"Battery Energy Consumption Over Time with QoS {qos}")
plt.grid(True)
plt.tight_layout()
plt.savefig("battery_mAh.png")

# --- Plot 4: Battery percentage ---
plt.figure()
plt.plot(df_batt["t_rel_s"], df_batt["remaining_pct"])
plt.xlabel("Time [s]")
plt.ylabel("Battery Level [%]")
plt.title(f"Battery Percentage Over Time with QoS {qos}")
plt.ylim(0, 100)  # (optional)
plt.grid(True)
plt.tight_layout()
plt.savefig("battery_percentage.png")
