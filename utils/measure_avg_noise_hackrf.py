# measure_avg_noise_hackrf.py

"""
Estimate the ambient RF noise floor on the 2.4 GHz or 5 GHz band using a
HackRF One software-defined radio and the hackrf_sweep command-line tool.

The script launches hackrf_sweep as a subprocess, parses its CSV output in
real time, and applies a low-percentile selection on the per-row power bins
to approximate the noise floor (i.e. the quietest spectral component in each
sweep row).  After the configured measurement duration, summary statistics
(mean, std, min, max) are printed and the per-row values are optionally saved
to a CSV file for further analysis.

Usage:
    python3 measure_avg_noise_hackrf.py --band 2.4 --duration 60 --outfile noise.csv
"""

import argparse
import subprocess
import sys
import time
from statistics import mean, pstdev


def estimate_noise_from_row(row, noise_percentile=0.2):
    """
        Extract a noise-floor estimate from one CSV row emitted by hackrf_sweep.

        hackrf_sweep output format (one row per frequency sub-band per sweep):
            date, time, freq_low_hz, freq_high_hz, bin_width_hz, num_bins, p0, p1, ..., pN

        where p0..pN are power spectral density values in dBm for each frequency
        bin within the [freq_low_hz, freq_high_hz] sub-band.

        Noise estimation strategy:
            The noise floor is approximated by taking the value at the
            noise_percentile-th quantile of the sorted power array.  Using the
            20th percentile (default) rather than the minimum avoids being misled
            by a single anomalously quiet bin; it robustly selects the quietest
            portion of the spectrum while discarding occupied channels and spurs.

        Parameters
        ----------
        row              : list of strings as produced by csv.reader.
        noise_percentile : fraction in (0, 1); lower values select quieter bins.

        Returns
        -------
        (timestamp_str, freq_low_hz, freq_high_hz, noise_db) on success, or
        None if the row is malformed, a comment line, or contains no valid powers.
    """
    # Skip empty rows and comment lines (hackrf_sweep emits '#'-prefixed headers).
    if not row or row[0].startswith("#"):
        return None

    # Minimum required columns: date, time, freq_low, freq_high, bin_width, num_bins, p0
    if len(row) < 7:
        return None

    try:
        timestamp = f"{row[0]} {row[1]}"
        freq_low_hz = float(row[2])
        freq_high_hz = float(row[3])
        num_bins = int(row[5])
    except (ValueError, IndexError):
        return None

    # Extract exactly num_bins power values starting at column index 6.
    powers_raw = row[6:6 + num_bins]
    if not powers_raw:
        return None

    try:
        powers = [float(x) for x in powers_raw]
    except ValueError:
        return None

    if not powers:
        return None

    # Select the value at the noise_percentile quantile of the sorted powers.
    # max(..., 0) guards against rounding to a negative index on very short arrays.
    powers_sorted = sorted(powers)
    idx = max(0, int(noise_percentile * len(powers_sorted)) - 1)
    noise_db = powers_sorted[idx]

    return timestamp, freq_low_hz, freq_high_hz, noise_db


def compute_noise_for_duration(stream, duration_s, csv_out=None, noise_percentile=0.2):
    """
        Read hackrf_sweep's stdout line by line and accumulate noise estimates
        until *duration_s* seconds have elapsed.

        The stream is wrapped in a csv.reader so that comma-separated fields are
        split automatically.  Lines may arrive as bytes (subprocess stdout) or
        str (e.g. in unit tests); the generator expression handles both cases.

        The measurement clock starts on the *first valid row* rather than at
        function entry, so that initialisation latency (hackrf_sweep hardware
        enumeration, USB enumeration) does not count against the measurement window.

        If *csv_out* is an open file, each valid row is written immediately and
        flushed so that data is preserved even if the process is interrupted.

        Parameters
        ----------
        stream           : iterable of lines from hackrf_sweep stdout.
        duration_s       : measurement window length in seconds.
        csv_out          : open writable text file, or None to skip saving.
        noise_percentile : passed through to estimate_noise_from_row().

        Returns
        -------
        List of noise_db floats collected during the measurement window.
    """
    reader = csv.reader(
        # Decode bytes to str on the fly; ignore undecodable characters.
        (line.decode("utf-8", "ignore") if isinstance(line, bytes) else line
         for line in stream)
    )

    t0 = None   # set on first valid row
    noise_values = []

    for row in reader:
        rec = estimate_noise_from_row(row, noise_percentile=noise_percentile)
        if rec is None:
            continue

        # Start the measurement clock on the first valid sample.
        if t0 is None:
            t0 = time.time()

        timestamp, f_low, f_high, noise_db = rec
        noise_values.append(noise_db)

        if csv_out is not None:
            csv_out.write(f"{timestamp},{f_low:.0f},{f_high:.0f},{noise_db:.2f}\n")
            csv_out.flush()

        # Stop once the measurement window has elapsed.
        if (time.time() - t0) >= duration_s:
            break

    return noise_values


def run_hackrf_sweep(freq_range, duration_s, outfile=None, noise_percentile=0.2):
    """
        Launch hackrf_sweep, collect noise data for *duration_s* seconds, and
        print summary statistics.

        hackrf_sweep is invoked via subprocess with its stdout piped to this
        process.  stderr is suppressed (DEVNULL) to avoid mixing hackrf's
        progress messages with the script's own output.

        The subprocess is always terminated in the finally block regardless of
        whether the measurement completed normally or raised an exception, to
        release the HackRF USB device cleanly.  proc.wait(timeout=2) gives the
        process a brief window to exit gracefully before the OS reclaims it.

        Parameters
        ----------
        freq_range       : frequency range string passed to hackrf_sweep -f,
                           e.g. "2400:2500" for the 2.4 GHz band (MHz units).
        duration_s       : measurement window in seconds.
        outfile          : path for the per-row CSV output, or None to skip.
        noise_percentile : quantile used by estimate_noise_from_row().
    """
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
        # Always release the HackRF device, even on exception or early exit.
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

    # Summary statistics over all collected noise estimates.
    mu = mean(noise_values)
    mn = min(noise_values)
    mx = max(noise_values)
    # pstdev (population std dev) is used because the full measurement window
    # is the population of interest, not a sample drawn from a larger one.
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
    parser.add_argument("--outfile", default="noise_values.wifi_csv",
                        help="CSV di output (consigliato).")
    parser.add_argument("--noise-percentile", type=float, default=0.2,
                        help="Percentile per stimare il noise floor (0.2=20%%, default 0.2).")

    args = parser.parse_args()

    # Map the user-facing band label to the hackrf_sweep frequency range (MHz).
    if args.band == "2.4":
        freq_range = "2400:2500"
    else:
        freq_range = "5150:5850"

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
