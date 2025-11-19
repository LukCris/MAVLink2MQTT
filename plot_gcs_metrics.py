# ---------- LATENZA ----------
import pandas as pd
import matplotlib.pyplot as plt

df_lat = pd.read_csv("latency_metrics.csv")
df_ok = df_lat[df_lat["lost"] == 0].copy()

qos = input("QoS level used:")

# Converti tutto in numerico (per sicurezza)
df_lat["t_s"] = pd.to_numeric(df_lat["t_s"], errors="coerce")
df_lat["id"] = pd.to_numeric(df_lat["id"], errors="coerce")
df_lat["rtt_ms"] = pd.to_numeric(df_lat["rtt_ms"], errors="coerce")

df_lat = df_lat.dropna(subset=["t_s"])

# Tempo relativo
t0_lat = df_lat["t_s"].iloc[0]
df_lat["t_rel_s"] = df_lat["t_s"] - t0_lat

# jitter = differenza assoluta tra RTT successivi
df_ok["jitter_ms"] = df_ok["rtt_ms"].diff().abs()

# --- RTT over time ---
plt.figure()
plt.plot(df_lat["t_rel_s"], df_lat["rtt_ms"], marker="o")
plt.xlabel("Time [s]")
plt.ylabel("Round-Trip Time [ms]")
plt.title(f"RTT Over Time with QoS {qos}")
plt.grid(True)
plt.tight_layout()
plt.savefig("latency_rtt.png")

# --- RTT boxplot ---
plt.figure()
plt.boxplot(df_lat["rtt_ms"], vert=True)
plt.ylabel("Round-Trip Time [ms]")
plt.title(f"RTT Distribution with QoS {qos}")
plt.tight_layout()
plt.savefig("latency_boxplot.png")

# --- Stats ---
print("=======================")
print(f"Stats with QoS {qos}:")
print("RTT mean:", df_ok["rtt_ms"].mean())
print("RTT std:", df_ok["rtt_ms"].std())
print("RTT min:", df_ok["rtt_ms"].min())
print("RTT max:", df_ok["rtt_ms"].max())
print("Packet loss [%]:", 100 * df_lat["lost"].mean())
print("Jitter mean [ms]:", df_ok["jitter_ms"].mean())
print("=======================")


# ---------- THROUGHPUT ----------
df_thr = pd.read_csv("link_metrics.csv")

df_thr["t_s"] = pd.to_numeric(df_thr["t_s"], errors="coerce")
df_thr["payload_bytes"] = pd.to_numeric(df_thr["payload_bytes"], errors="coerce")
df_thr["total_tx_bytes"] = pd.to_numeric(df_thr["total_tx_bytes"], errors="coerce")
df_thr["total_rx_bytes"] = pd.to_numeric(df_thr["total_rx_bytes"], errors="coerce")

df_thr = df_thr.dropna(subset=["t_s"])
df_thr = df_thr.sort_values("t_s")

# Tempo relativo
t0_thr = df_thr["t_s"].iloc[0]
df_thr["t_rel_s"] = df_thr["t_s"] - t0_thr

# --- Plot cumulativo TX/RX ---
plt.figure()
plt.plot(df_thr["t_rel_s"], df_thr["total_tx_bytes"], label="TX cumulative bytes")
plt.plot(df_thr["t_rel_s"], df_thr["total_rx_bytes"], label="RX cumulative bytes")
plt.xlabel("Time [s]")
plt.ylabel("Cumulative Bytes")
plt.title(f"Cumulative TX/RX Data Over Time with QoS {qos}")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig("throughput_cumulative.png")

# --- Throughput istantaneo (byte/s) ---
df_thr["dt"] = df_thr["t_s"].diff()
df_thr["d_tx_bytes"] = df_thr["total_tx_bytes"].diff()
df_thr["d_rx_bytes"] = df_thr["total_rx_bytes"].diff()

df_thr["tx_Bps"] = df_thr["d_tx_bytes"] / df_thr["dt"]
df_thr["rx_Bps"] = df_thr["d_rx_bytes"] / df_thr["dt"]

# Salta la prima riga (NaN)
df_thr_rate = df_thr.dropna(subset=["tx_Bps", "rx_Bps"])

plt.figure()
plt.plot(df_thr_rate["t_rel_s"], df_thr_rate["tx_Bps"], marker="o", label="TX [Bytes/s]")
plt.plot(df_thr_rate["t_rel_s"], df_thr_rate["rx_Bps"], marker="o", label="RX [Bytes/s]")
plt.xlabel("Time [s]")
plt.ylabel("Throughput [Bytes/s]")
plt.title(f"Instantaneous TX/RX Throughput with QoS {qos}")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig("throughput_rates.png")
