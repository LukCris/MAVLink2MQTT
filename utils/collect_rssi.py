#collect_rssi.py
"""
Sample RSSI via CoreWLAN (macOS) and save the results to a CSV file.

Usage:
    python3 collect_rssi.py --out rssi_survey.wifi_csv --interval 0.5

During the survey:
    - Press ENTER to mark the current measurement point (records the current position)
    - Press Ctrl+C to terminate

The produced CSV has the following columns:
    t_s, rssi_dBm, noise_dBm, ssid, bssid, mark
where 'mark' is 1 only on the row in which ENTER was pressed
(useful for identifying the samples to associate with the physical
coordinates of the position occupied during the survey).
"""

import argparse
import csv
import sys
import threading
import time

# CoreWLAN is a macOS-only framework exposed to Python via the pyobjc bridge.
# The import guard gives a clear error on non-macOS systems rather than an
# obscure AttributeError later.
try:
    import CoreWLAN
except ImportError:
    sys.exit(
        "[ERROR] CoreWLAN non trovato.\n"
        "Installa con:  pip install pyobjc-framework-CoreWLAN"
    )


# ========================
# CoreWLAN helpers
# ========================

def get_wifi_interface() -> "CoreWLAN.CWInterface":
    """
        Return the default Wi-Fi interface via the shared CoreWLAN client.

        CWWiFiClient.sharedWiFiClient() is the singleton entry point to the
        CoreWLAN framework.  interface() returns the primary Wi-Fi adapter
        (e.g. en0 on most Macs); it returns None if no Wi-Fi hardware is present
        or if the adapter is disabled.
    """
    client = CoreWLAN.CWWiFiClient.sharedWiFiClient()
    iface = client.interface()
    if iface is None:
        sys.exit("[ERROR] Nessuna interfaccia Wi-Fi trovata.")
    return iface


def sample(iface: "CoreWLAN.CWInterface") -> dict:
    """
        Take an instantaneous RF measurement from the Wi-Fi interface.

        CoreWLAN values:
          - rssiValue()       : RSSI of the associated AP in dBm (typically −30 to −90).
          - noiseMeasurement(): ambient noise floor in dBm.
                                On 5 GHz, macOS returns a fixed driver-level constant
                                (hardware limitation); this is declared as a known
                                constraint in the thesis and SNR is therefore not
                                computed for the 5 GHz band.
          - ssid() / bssid()  : human-readable and MAC-address identifiers of the
                                associated AP; included for traceability in case
                                the adapter roams between APs during a long survey.

        The 'mark' field defaults to 0 and is set to 1 by run_survey() when the
        user presses ENTER to tag the current physical position.
    """
    return {
        "t_s":      time.time(),
        "rssi_dBm": iface.rssiValue(),
        "noise_dBm": iface.noiseMeasurement(),
        "ssid":     iface.ssid() or "",
        "bssid":    iface.bssid() or "",
        "mark":     0,
    }


# ========================
# Mark mechanism
# ========================

# A threading.Event used to signal between the input listener thread and the
# main sampling loop.  Using an Event (rather than a shared boolean) is
# thread-safe without explicit locking: set() and is_set() are atomic in
# CPython's threading implementation.
_mark_flag = threading.Event()


def _input_loop():
    """
    Dedicated thread that waits for the user to press ENTER.

    Blocking input() in a separate thread allows the main loop to continue
    sampling at the configured rate without being interrupted.  When ENTER
    is detected, the Event is set; the main loop checks it on the next
    iteration and writes mark=1 on that sample row.
    EOFError is caught to handle non-interactive stdin (e.g. piped input).
    """
    while True:
        try:
            input()           # blocks until ENTER is pressed
            _mark_flag.set()  # signal the main loop
        except EOFError:
            break


def run_survey(out_path: str, interval: float):
    """
        Main survey loop: sample RSSI at *interval* seconds and write to CSV.

        The mark workflow:
          1. The user moves to a physical position of interest.
          2. They press ENTER; _input_loop() sets _mark_flag.
          3. On the next sample iteration, the flag is detected, the current row
             gets mark=1, and the flag is cleared for the next mark.
          4. Post-processing (plot_heatmap.py) filters rows where mark=1 to
             extract the (RSSI, position) pairs used for spatial interpolation.

        The CSV is flushed after every row (f.flush()) so that data is not lost
        if the process is killed rather than terminated with Ctrl+C.
    """
    iface = get_wifi_interface()

    print(f"[INFO] Interfaccia: {iface.interfaceName()}")
    print(f"[INFO] SSID corrente: {iface.ssid()}")
    print(f"[INFO] Campionamento ogni {interval}s → {out_path}")
    print()
    print("  Premi  INVIO   per marcare il punto corrente")
    print("  Premi  Ctrl+C  per terminare il survey")
    print()

    # Daemon thread: killed automatically when the main thread exits.
    t = threading.Thread(target=_input_loop, daemon=True)
    t.start()

    with open(out_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=["t_s", "rssi_dBm", "noise_dBm", "ssid", "bssid", "mark"]
        )
        writer.writeheader()

        n_samples = 0
        n_marks   = 0

        try:
            while True:
                row = sample(iface)

                # Check atomically whether the user pressed ENTER since the
                # last iteration; if so, tag this sample and reset the flag.
                if _mark_flag.is_set():
                    row["mark"] = 1
                    _mark_flag.clear()
                    n_marks += 1
                    print(f"  ★  MARK #{n_marks}  |  RSSI={row['rssi_dBm']} dBm  |  t={row['t_s']:.3f}")

                writer.writerow(row)
                f.flush()   # ensure data survives an unclean exit
                n_samples += 1

                time.sleep(interval)

        except KeyboardInterrupt:
            print(f"\n[INFO] Survey terminato. Campioni totali: {n_samples}, Mark: {n_marks}")
            print(f"[INFO] File salvato: {out_path}")


# ========================
# Entry point
# ========================

def main():
    parser = argparse.ArgumentParser(description="Survey RSSI con CoreWLAN (macOS)")
    parser.add_argument("--out",      default="rssi_survey.wifi_csv",
                        help="File CSV di output (default: rssi_survey.wifi_csv)")
    parser.add_argument("--interval", type=float, default=0.5,
                        help="Intervallo di campionamento in secondi (default: 0.5)")
    args = parser.parse_args()

    run_survey(args.out, args.interval)


if __name__ == "__main__":
    main()