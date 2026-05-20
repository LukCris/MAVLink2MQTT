#!/usr/bin/env python3
"""
measure_mac_snr.py — Raccolta di RSSI, noise floor e SNR via CoreWLAN.

COMPATIBILITÀ:
  - Solo macOS (non funziona su Linux o Windows)
  - Richiede macOS 10.7 (Lion) o successivo, che è quando CoreWLAN
    è stato introdotto come framework pubblico Apple.
  - NON usa Apple80211.framework (deprecato e rimosso nelle versioni recenti
    di macOS); usa invece CoreWLAN, che è il framework ufficiale e supportato.

DIPENDENZE:
  pip install pyobjc-core pyobjc-framework-CoreWLAN

UTILIZZO:
  python3 measure_mac_snr.py <nome_scenario>

  Esempio:
    python3 measure_mac_snr.py dist-4m_tls-on_qos0_5ghz

  I dati vengono salvati in:
    logs/<nome_scenario>/mac_rssi_noise.wifi_csv
"""

import objc
import CoreWLAN
import time, sys, os

def get_wifi_metrics():
    iface = CoreWLAN.CWWiFiClient.sharedWiFiClient().interface()
    if iface is None:
        return None, None

    rssi = iface.rssiValue()
    noise = iface.noiseMeasurement()
    return float(rssi), float(noise)


scenario = sys.argv[1] if len(sys.argv) > 1 else "default"
os.makedirs(f"logs/{scenario}", exist_ok=True)
out_path = f"logs/{scenario}/mac_rssi_noise.wifi_csv"

with open(out_path, "w") as f:
    f.write("t_s,rssi_dbm,noise_dbm,snr_db\n")
    while True:
        rssi, noise = get_wifi_metrics()
        if rssi is not None and noise is not None:
            t = time.time()
            snr = rssi - noise
            f.write(f"{t},{rssi},{noise},{snr:.1f}\n")
            f.flush()
            print(f"RSSI={rssi} dBm  Noise={noise} dBm  SNR={snr:.1f} dB")
        time.sleep(1)