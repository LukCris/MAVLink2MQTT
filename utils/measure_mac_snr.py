# measure_mac_snr.py
"""
Continuous RSSI, noise floor, and SNR collection via CoreWLAN.

COMPATIBILITY:
  - macOS only (not available on Linux or Windows).
  - Requires macOS 10.7 (Lion) or later, when CoreWLAN was introduced as a
    public Apple framework.
  - Does NOT use Apple80211.framework (deprecated and removed in recent macOS
    versions); uses CoreWLAN, the official supported framework.

DEPENDENCIES:
  pip install pyobjc-core pyobjc-framework-CoreWLAN

USAGE:
  python3 measure_mac_snr.py <scenario_name>
  Example:
    python3 measure_mac_snr.py dist-4m_tls-on_qos0_5ghz

  Data is saved to:
    logs/<scenario_name>/mac_rssi_noise.csv

KNOWN LIMITATION:
  On the 5 GHz band, macOS returns a fixed constant for noiseMeasurement()
  due to a driver-level restriction; the SNR column is therefore not
  meaningful at 5 GHz and is declared as such in the thesis.
"""

import objc
import CoreWLAN
import time, sys, os

def get_wifi_metrics():
    """
        Read the current RSSI and noise floor from the default Wi-Fi interface.

        CoreWLAN values:
          - rssiValue()        : received signal strength in dBm (e.g. -55).
                                 Updated at the adapter's internal polling rate,
                                 typically once per second.
          - noiseMeasurement() : ambient noise floor in dBm (e.g. -95).
                                 On 5 GHz this is a driver-level constant, not a
                                 live measurement (see module docstring).

        Returns (rssi, noise) as floats, or (None, None) if no interface is
        found (e.g. Wi-Fi is disabled or no AP is associated).
    """
    iface = CoreWLAN.CWWiFiClient.sharedWiFiClient().interface()
    if iface is None:
        return None, None

    rssi = iface.rssiValue()
    noise = iface.noiseMeasurement()
    return float(rssi), float(noise)

# ========================
# Script-level entry point
# ========================
# The scenario name is taken from the first command-line argument so that
# the output path matches the directory structure used by all other
# measurement scripts (run_gcs_scenario.sh, collect_rssi.py, etc.).
scenario = sys.argv[1] if len(sys.argv) > 1 else "default"
os.makedirs(f"logs/{scenario}", exist_ok=True)
out_path = f"logs/{scenario}/mac_rssi_noise.wifi_csv"

with open(out_path, "w") as f:
    f.write("t_s,rssi_dbm,noise_dbm,snr_db\n")
    while True:
        rssi, noise = get_wifi_metrics()
        if rssi is not None and noise is not None:
            t = time.time()

            # SNR is computed as the arithmetic difference between RSSI and
            # noise floor (both in dBm), which in logarithmic scale equals
            # the ratio of signal power to noise power in dB.
            # Note: this is only meaningful at 2.4 GHz; see module docstring
            # for the 5 GHz limitation.
            snr = rssi - noise
            f.write(f"{t},{rssi},{noise},{snr:.1f}\n")
            f.flush()   # ensure data survives an unclean exit (Ctrl+C)
            print(f"RSSI={rssi} dBm  Noise={noise} dBm  SNR={snr:.1f} dB")
        time.sleep(1)   # 1 Hz sampling rate matches CoreWLAN's update cadence