#!/usr/bin/env python3
# ============================================================
#  Tests for hint.py's pure pulse-width arithmetic - no GUI, no
#  serial. Checks the worked examples from the spec plus edge cases
#  (too few pulses, flat/noisy distribution).
#
#  Run: python3 host/test_hint.py
# ============================================================
import os
import random
import sys

sys.path.insert(0, os.path.dirname(__file__))
from hint import classify_pulse_width, hint_text, dominant_pulse_width


def expect(cond, msg, failures):
    if not cond:
        failures.append(msg)
        print(f"FAIL: {msg}")


def main():
    failures = []

    # ---- worked examples from the spec ----
    msg = classify_pulse_width(5.02)
    expect("Dominant pulse width 5.02 us." in msg, f"5.02us wording: {msg!r}", failures)
    expect("100 kHz" in msg and "Consistent with" in msg, f"5.02us should read as 100kHz: {msg!r}", failures)

    msg = classify_pulse_width(8.68)
    expect("115200 baud" in msg and "Consistent with" in msg, f"8.68us should read as 115200 baud: {msg!r}", failures)

    # ---- 400 kHz I2C fast mode: half-period = 1.25 us ----
    msg = classify_pulse_width(1.25)
    expect("400 kHz" in msg and "Consistent with" in msg, f"1.25us should read as 400kHz: {msg!r}", failures)

    # ---- 1-Wire reset pulse ----
    msg = classify_pulse_width(480.0)
    expect("1-Wire" in msg and "Consistent with" in msg, f"480us should read as 1-Wire reset: {msg!r}", failures)

    # ---- a value that matches nothing well (arbitrary, between all
    #      candidates) - must not claim a clean match ----
    msg = classify_pulse_width(37.0)
    expect("Consistent with" not in msg, f"37us should not claim a clean match: {msg!r}", failures)

    # ---- too few pulses: no dominant width should be claimed ----
    expect(dominant_pulse_width([1.0, 2.0, 3.0]) is None,
           "3 pulses should not produce a dominant width", failures)
    expect(hint_text([1.0, 2.0, 3.0]) == "No pulses captured yet." or
           "No dominant" in hint_text([1.0, 2.0, 3.0]),
           f"too-few-pulses hint text unexpected: {hint_text([1.0, 2.0, 3.0])!r}", failures)

    # ---- empty ----
    expect(hint_text([]) == "No pulses captured yet.", f"empty hint text: {hint_text([])!r}", failures)

    # ---- flat/noisy distribution (uniform random spread, no cluster)
    #      - must report "no dominant pulse width", not a false guess ----
    random.seed(3)
    noisy = [random.uniform(0.5, 500.0) for _ in range(500)]
    text = hint_text(noisy)
    expect("noise or contact bounce" in text, f"flat distribution should read as noise: {text!r}", failures)

    # ---- a real dominant cluster should be found even amid some
    #      background noise, and should match the injected period ----
    random.seed(4)
    cluster = [8.68 + random.uniform(-0.1, 0.1) for _ in range(300)]
    background = [random.uniform(1.0, 100.0) for _ in range(30)]
    widths = cluster + background
    result = dominant_pulse_width(widths)
    expect(result is not None, "cluster+background should still find a dominant width", failures)
    if result is not None:
        w, conc = result
        expect(abs(w - 8.68) < 0.5, f"dominant width should be near 8.68us, got {w:.3f}", failures)

    # ---- regression: a bursty real-world signal (silent for seconds,
    #      then a short burst) must not let one huge idle gap wreck the
    #      reported dominant width. This is exactly what a live capture
    #      of a 100kHz I2C bus polled every ~10s looks like - one rare
    #      ~10,000,000us gap alongside hundreds of ~5us bit periods.
    #      Before the fix this reported the WINNING BIN'S CENTER (tens
    #      of thousands of us, garbage) instead of the median of the
    #      actual ~5us samples in it. ----
    random.seed(5)
    i2c_bits = [5.0 + random.uniform(-0.05, 0.05) for _ in range(300)]
    idle_gaps = [10_000_000.0 + random.uniform(-100, 100) for _ in range(3)]
    widths = i2c_bits + idle_gaps
    result = dominant_pulse_width(widths)
    expect(result is not None, "bursty signal should still find a dominant width", failures)
    if result is not None:
        w, conc = result
        expect(abs(w - 5.0) < 0.5,
               f"dominant width should be ~5us despite the 10,000,000us outlier, got {w:.3f}", failures)
    msg = hint_text(widths)
    expect("100 kHz" in msg and "Consistent with" in msg,
           f"bursty I2C signal should still read as 100kHz, got: {msg!r}", failures)

    print()
    if failures:
        print(f"---- {len(failures)} FAILURE(S) ----")
        sys.exit(1)
    else:
        print("---- all hint arithmetic tests passed ----")


if __name__ == "__main__":
    main()
