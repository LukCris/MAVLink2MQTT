#!/usr/bin/env python3
"""
collect_rssi.py
---------------
Campiona il RSSI via CoreWLAN (macOS) e salva su CSV.

Utilizzo:
    python3 collect_rssi.py --out rssi_survey.csv --interval 0.5

Durante il survey:
    - Premi INVIO per marcare un "punto di misura" (registra la posizione corrente)
    - Premi Ctrl+C per terminare

Il CSV prodotto ha colonne:
    t_s, rssi_dBm, noise_dBm, ssid, bssid, mark
dove 'mark' è 1 solo sulla riga in cui hai premuto INVIO
(utile per identificare i campioni da associare alle coordinate).
"""

import argparse
import csv
import sys
import threading
import time

try:
    import CoreWLAN
except ImportError:
    sys.exit(
        "[ERROR] CoreWLAN non trovato.\n"
        "Installa con:  pip install pyobjc-framework-CoreWLAN"
    )


# ---------------------------------------------------------------------------
# CoreWLAN helpers
# ---------------------------------------------------------------------------

def get_wifi_interface() -> "CoreWLAN.CWInterface":
    client = CoreWLAN.CWWiFiClient.sharedWiFiClient()
    iface = client.interface()
    if iface is None:
        sys.exit("[ERROR] Nessuna interfaccia Wi-Fi trovata.")
    return iface


def sample(iface: "CoreWLAN.CWInterface") -> dict:
    """Restituisce un campione istantaneo dall'interfaccia Wi-Fi."""
    return {
        "t_s":      time.time(),
        "rssi_dBm": iface.rssiValue(),       # int, es. -55
        "noise_dBm": iface.noiseMeasurement(), # int, es. -95
        "ssid":     iface.ssid() or "",
        "bssid":    iface.bssid() or "",
        "mark":     0,
    }


# ---------------------------------------------------------------------------
# Campionamento in background
# ---------------------------------------------------------------------------

_mark_flag = threading.Event()


def _input_loop():
    """Thread separato: aspetta INVIO e setta il flag di mark."""
    while True:
        try:
            input()          # blocca finché l'utente non preme INVIO
            _mark_flag.set()
        except EOFError:
            break


def run_survey(out_path: str, interval: float):
    iface = get_wifi_interface()

    print(f"[INFO] Interfaccia: {iface.interfaceName()}")
    print(f"[INFO] SSID corrente: {iface.ssid()}")
    print(f"[INFO] Campionamento ogni {interval}s → {out_path}")
    print()
    print("  Premi  INVIO   per marcare il punto corrente")
    print("  Premi  Ctrl+C  per terminare il survey")
    print()

    # Thread listener INVIO
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

                # Controlla se l'utente ha premuto INVIO
                if _mark_flag.is_set():
                    row["mark"] = 1
                    _mark_flag.clear()
                    n_marks += 1
                    print(f"  ★  MARK #{n_marks}  |  RSSI={row['rssi_dBm']} dBm  |  t={row['t_s']:.3f}")

                writer.writerow(row)
                f.flush()
                n_samples += 1

                time.sleep(interval)

        except KeyboardInterrupt:
            print(f"\n[INFO] Survey terminato. Campioni totali: {n_samples}, Mark: {n_marks}")
            print(f"[INFO] File salvato: {out_path}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Survey RSSI con CoreWLAN (macOS)")
    parser.add_argument("--out",      default="rssi_survey.csv",
                        help="File CSV di output (default: rssi_survey.csv)")
    parser.add_argument("--interval", type=float, default=0.5,
                        help="Intervallo di campionamento in secondi (default: 0.5)")
    args = parser.parse_args()

    run_survey(args.out, args.interval)


if __name__ == "__main__":
    main()