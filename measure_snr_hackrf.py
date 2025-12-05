#!/usr/bin/env python3
import argparse
import csv
import subprocess
import sys
from statistics import mean

def estimate_snr_from_row(row):
    """
    row: una riga CSV di hackrf_sweep.
    Formato tipico:
    date, time, freq_start_hz, bin_width_hz, num_bins, p0, p1, ..., pN

    Restituisce (snr_db, signal_db, noise_db) oppure (None, None, None) se riga non valida.
    """
    if not row or row[0].startswith("#"):
        return None, None, None

    if len(row) < 6:
        return None, None, None

    try:
        # Questi tre li leggiamo solo per completezza (freq di partenza, ecc.)
        freq_start = float(row[2])      # Hz
        bin_width = float(row[3])       # Hz
        num_bins = int(row[4])
    except (ValueError, IndexError):
        return None, None, None

    # Potenze in dBFS (o dB relativi) nei bin
    powers_raw = row[5:5 + num_bins]
    if not powers_raw:
        return None, None, None

    try:
        powers = [float(x) for x in powers_raw]
    except ValueError:
        return None, None, None

    if not powers:
        return None, None, None

    powers_sorted = sorted(powers)

    # Stima del rumore: prendiamo il 20° percentile dei bin
    # (assumendo che la maggior parte della banda sia "rumore" e il segnale sia un picco).
    idx = max(0, int(0.2 * len(powers_sorted)) - 1)
    noise_db = powers_sorted[idx]

    # Segnale = picco massimo
    signal_db = max(powers)

    snr_db = signal_db - noise_db
    return snr_db, signal_db, noise_db


def compute_snr_from_stream(stream, csv_out=None):
    """
    Legge le righe CSV da uno stream (stdout di hackrf_sweep o un file aperto)
    e calcola l'SNR per ogni sweep.
    Se csv_out è un handle aperto in scrittura, salva:
      timestamp, freq_start_hz, snr_db, signal_db, noise_db
    Restituisce lista degli SNR validi.
    """
    reader = csv.reader((line.decode("utf-8", "ignore") if isinstance(line, bytes) else line
                         for line in stream))

    snr_values = []

    for row in reader:
        if not row or row[0].startswith("#"):
            continue

        # estraggo per logging opzionale
        timestamp = ""
        freq_start_hz = None
        try:
            timestamp = f"{row[0]} {row[1]}"
            freq_start_hz = float(row[2])
        except (IndexError, ValueError):
            pass

        snr_db, signal_db, noise_db = estimate_snr_from_row(row)
        if snr_db is None:
            continue

        snr_values.append(snr_db)

        if csv_out is not None and freq_start_hz is not None:
            csv_out.write(f"{timestamp},{freq_start_hz},{snr_db:.2f},{signal_db:.2f},{noise_db:.2f}\n")

    return snr_values


def run_hackrf_sweep(freq_range, sweeps, outfile=None):
    """
    Lancia hackrf_sweep sul range freq_range (stringa tipo '2400:2500')
    per 'sweeps' sweep, e calcola SNR.
    """
    cmd = [
        "hackrf_sweep",
        "-f", freq_range,
        "-n", str(sweeps)
    ]

    print(f"[INFO] Running: {' '.join(cmd)}", file=sys.stderr)

    csv_out = None
    if outfile:
        csv_out = open(outfile, "w", encoding="utf-8")
        csv_out.write("timestamp,freq_start_hz,snr_db,signal_db,noise_db\n")

    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=sys.stderr)
    except FileNotFoundError:
        print("[ERROR] hackrf_sweep non trovato nel PATH", file=sys.stderr)
        if csv_out:
            csv_out.close()
        return

    snr_values = compute_snr_from_stream(proc.stdout, csv_out)
    proc.wait()

    if csv_out:
        csv_out.close()

    if not snr_values:
        print("[RESULT] Nessuna misura SNR valida trovata.")
        return

    print(f"[RESULT] Misure SNR: n={len(snr_values)}")
    print(f"[RESULT] SNR medio   = {mean(snr_values):.2f} dB")
    print(f"[RESULT] SNR minimo  = {min(snr_values):.2f} dB")
    print(f"[RESULT] SNR massimo = {max(snr_values):.2f} dB")


def compute_snr_from_file(path, outfile=None):
    """
    Variante offline: legge un file salvato da hackrf_sweep (CSV testo)
    e calcola l'SNR.
    """
    print(f"[INFO] Analizzo file: {path}")

    csv_out = None
    if outfile:
        csv_out = open(outfile, "w", encoding="utf-8")
        csv_out.write("timestamp,freq_start_hz,snr_db,signal_db,noise_db\n")

    with open(path, "r", encoding="utf-8") as f:
        snr_values = compute_snr_from_stream(f, csv_out)

    if csv_out:
        csv_out.close()

    if not snr_values:
        print("[RESULT] Nessuna misura SNR valida trovata.")
        return

    print(f"[RESULT] Misure SNR: n={len(snr_values)}")
    print(f"[RESULT] SNR medio   = {mean(snr_values):.2f} dB")
    print(f"[RESULT] SNR minimo  = {min(snr_values):.2f} dB")
    print(f"[RESULT] SNR massimo = {max(snr_values):.2f} dB")


def main():
    parser = argparse.ArgumentParser(
        description="Misura SNR su 2.4 GHz o 5 GHz usando HackRF One e hackrf_sweep."
    )

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--band", choices=["2.4", "5"], help="Banda Wi-Fi da misurare (2.4 o 5 GHz)")
    group.add_argument("--freq-range", help="Range frequenza custom per hackrf_sweep, es. '2400:2500'")
    group.add_argument("--input", help="File di log già salvato da hackrf_sweep (analisi offline)")

    parser.add_argument("--sweeps", type=int, default=50,
                        help="Numero di sweep da acquisire (solo in modalità live, default=50)")
    parser.add_argument("--outfile", help="File CSV opzionale in cui salvare le misure SNR")

    args = parser.parse_args()

    if args.input:
        # Modalità offline: usa solo il file
        compute_snr_from_file(args.input, outfile=args.outfile)
        return

    # Modalità live: lancia hackrf_sweep
    if args.band:
        if args.band == "2.4":
            freq_range = "2400:2500"   # MHz approx (HackRF accetta anche 2400:2500)
        else:
            freq_range = "5150:5850"   # banda 5 GHz tipica Wi-Fi
    else:
        freq_range = args.freq_range

    run_hackrf_sweep(freq_range, args.sweeps, outfile=args.outfile)


if __name__ == "__main__":
    main()
