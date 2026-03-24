#!/usr/bin/env python3
import argparse
import csv
import subprocess
import sys
import time
from statistics import mean, pstdev


def estimate_noise_from_row(row, noise_percentile=0.2):
    """
    row: riga CSV di hackrf_sweep
    Formato tipico:
    date, time, freq_low_hz, freq_high_hz, bin_width_hz, num_bins, p0, p1, ..., pN

    Ritorna (timestamp_str, freq_low_hz, freq_high_hz, noise_db) oppure None se riga non valida.
    """
    if not row or row[0].startswith("#"):
        return None

    if len(row) < 7:
        return None

    try:
        timestamp = f"{row[0]} {row[1]}"
        freq_low_hz = float(row[2])
        freq_high_hz = float(row[3])
        num_bins = int(row[5])
    except (ValueError, IndexError):
        return None

    powers_raw = row[6:6 + num_bins]
    if not powers_raw:
        return None

    try:
        powers = [float(x) for x in powers_raw]
    except ValueError:
        return None

    if not powers:
        return None

    powers_sorted = sorted(powers)
    idx = max(0, int(noise_percentile * len(powers_sorted)) - 1)
    noise_db = powers_sorted[idx]

    return timestamp, freq_low_hz, freq_high_hz, noise_db


def compute_noise_for_duration(stream, duration_s, csv_out=None, noise_percentile=0.2):
    """
    Legge stdout di hackrf_sweep e accumula noise_db finché non scade duration_s.
    Se csv_out è aperto, salva: timestamp,freq_low_hz,freq_high_hz,noise_db
    """
    reader = csv.reader(
        (line.decode("utf-8", "ignore") if isinstance(line, bytes) else line
         for line in stream)
    )

    t0 = None
    noise_values = []

    for row in reader:
        rec = estimate_noise_from_row(row, noise_percentile=noise_percentile)
        if rec is None:
            continue

        if t0 is None:
            t0 = time.time()

        timestamp, f_low, f_high, noise_db = rec
        noise_values.append(noise_db)

        if csv_out is not None:
            csv_out.write(f"{timestamp},{f_low:.0f},{f_high:.0f},{noise_db:.2f}\n")
            csv_out.flush()

        if (time.time() - t0) >= duration_s:
            break

    return noise_values


def run_hackrf_sweep(freq_range, duration_s, outfile=None, noise_percentile=0.2):
    cmd = ["hackrf_sweep", "-f", freq_range]
    print(f"[INFO] Running: {' '.join(cmd)}", file=sys.stderr)
    print(f"[INFO] Measuring noise for {duration_s}s, percentile={int(noise_percentile*100)}%", file=sys.stderr)

    csv_out = None
    if outfile:
        csv_out = open(outfile, "w", encoding="utf-8", newline="")
        csv_out.write("timestamp,freq_low_hz,freq_high_hz,noise_db\n")
        csv_out.flush()

    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    except FileNotFoundError:
        print("[ERROR] hackrf_sweep non trovato nel PATH", file=sys.stderr)
        if csv_out:
            csv_out.close()
        return

    noise_values = []
    try:
        noise_values = compute_noise_for_duration(
            proc.stdout,
            duration_s=duration_s,
            csv_out=csv_out,
            noise_percentile=noise_percentile
        )
    finally:
        # chiusura pulita
        try:
            proc.terminate()
        except Exception:
            pass
        try:
            proc.wait(timeout=2)
        except Exception:
            pass
        if csv_out:
            csv_out.close()

    if not noise_values:
        print("[RESULT] Nessuna misura di rumore valida trovata.")
        return

    mu = mean(noise_values)
    mn = min(noise_values)
    mx = max(noise_values)
    sd = pstdev(noise_values) if len(noise_values) > 1 else 0.0

    print(f"[RESULT] Noise samples: n={len(noise_values)}")
    print(f"[RESULT] Noise mean  = {mu:.2f} dB")
    print(f"[RESULT] Noise std   = {sd:.2f} dB")
    print(f"[RESULT] Noise min   = {mn:.2f} dB")
    print(f"[RESULT] Noise max   = {mx:.2f} dB")

    if outfile:
        print(f"[RESULT] Saved to: {outfile}")


def main():
    parser = argparse.ArgumentParser(
        description="Misura il rumore ambientale su 2.4/5 GHz con HackRF One (hackrf_sweep) per una durata fissa."
    )
    parser.add_argument("--band", choices=["2.4", "5"], required=True,
                        help="Banda da osservare (2.4 o 5 GHz).")
    parser.add_argument("--duration", type=int, default=60,
                        help="Durata misura in secondi (default: 60).")
    parser.add_argument("--outfile", default="noise_values.csv",
                        help="CSV di output (consigliato).")
    parser.add_argument("--noise-percentile", type=float, default=0.2,
                        help="Percentile per stimare il noise floor (0.2=20%%, default 0.2).")

    args = parser.parse_args()

    if args.band == "2.4":
        freq_range = "2400:2500"
    else:
        freq_range = "5150:5850"

    # sanity
    if not (0.0 < args.noise_percentile < 1.0):
        raise SystemExit("Errore: --noise-percentile deve essere tra 0 e 1 (es. 0.2).")

    run_hackrf_sweep(
        freq_range=freq_range,
        duration_s=args.duration,
        outfile=args.outfile,
        noise_percentile=args.noise_percentile
    )


if __name__ == "__main__":
    main()
