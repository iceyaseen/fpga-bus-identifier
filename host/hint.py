# ============================================================
#  hint.py - "protocol hint" arithmetic: pure pulse-width guesswork,
#  no decoding, no GUI. Kept separate from host.py so the numbers can
#  be unit-tested on their own (see test_hint.py).
#
#  The idea: find the dominant (most common) pulse width in a
#  pulse-width histogram, then check it against a short list of known
#  timing constants two different ways -
#
#    - as a clock's half-period:  frequency = 1 / (2 * width)
#    - as a UART bit period:      baud      = 1 / width
#
#  and separately against a fixed ~480 us 1-Wire reset pulse. Whichever
#  candidate has the smallest relative error becomes the guess.
#
#  IMPORTANT: this is timing arithmetic only. A 100 kHz square wave
#  could be an I2C clock, a SPI clock, or something else that just
#  happens to run at that rate - matching a number here is not the
#  same as identifying a protocol. See DISCLAIMER below, which the
#  host UI shows alongside every hint.
# ============================================================

STANDARD_BAUDS = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]
I2C_CLOCKS_HZ = [100_000, 400_000]     # standard-mode / fast-mode I2C
ONEWIRE_RESET_US = 480.0               # 1-Wire bus reset low pulse

MIN_PULSES = 5                  # too few samples to call anything "dominant"
CONCENTRATION_THRESHOLD = 0.15  # dominant bin must hold >= 15% of all pulses

GOOD_MATCH_PCT = 5.0     # <= this: "Consistent with"
ROUGH_MATCH_PCT = 25.0   # <= this: "Roughly matches"; above: "doesn't closely match"

from timefmt import format_duration_us

DISCLAIMER = (
    "Timing only, not protocol ID: a 100 kHz clock could be I2C, SPI, or "
    "something else entirely, and a matching bit period doesn't confirm UART "
    "framing either. Real identification needs structural rules (start "
    "conditions, addressing, parity), not just pulse widths - that comes later."
)


def _bucket(widths, nbins=100):
    """Sort widths into nbins equal-width buckets spanning their full
    range. Returns the list of buckets (each a list of the actual
    values that landed there) - NOT just counts, because a real signal
    that's silent for long stretches and bursts occasionally (I2C,
    e.g.) has a rare huge gap that stretches (lo, hi) enormously, and
    reporting a bucket's geometric CENTER as "the" width is meaningless
    when that bucket's real span is dozens of us to several seconds.
    Reporting the median of what ACTUALLY landed in the winning bucket
    instead gives the right answer regardless of how wide the nominal
    bucket boundaries are."""
    lo, hi = min(widths), max(widths)
    if hi <= lo:
        hi = lo + 1.0
    bin_w = (hi - lo) / nbins
    buckets = [[] for _ in range(nbins)]
    for w in widths:
        idx = int((w - lo) / bin_w)
        if idx >= nbins:
            idx = nbins - 1
        elif idx < 0:
            idx = 0
        buckets[idx].append(w)
    return buckets


def dominant_pulse_width(widths, nbins=100):
    """Returns (width_us, concentration) describing the dominant pulse
    width over the FULL range of `widths` (not whatever the user has
    zoomed into on screen - the hint should describe the whole
    capture), or None if there isn't a clear dominant width (too few
    pulses, or the distribution is too flat/spread out to call
    anything "dominant").

    width_us is the MEDIAN of the actual values in the tallest bucket,
    not that bucket's geometric center - see _bucket()'s docstring for
    why the distinction matters for a bursty signal."""
    if len(widths) < MIN_PULSES:
        return None
    buckets = _bucket(widths, nbins)
    counts = [len(b) for b in buckets]
    total = sum(counts)
    if total == 0:
        return None
    max_idx = max(range(len(counts)), key=lambda i: counts[i])
    max_count = counts[max_idx]
    concentration = max_count / total
    if concentration < CONCENTRATION_THRESHOLD:
        return None
    winners = sorted(buckets[max_idx])
    mid = len(winners) // 2
    median = winners[mid] if len(winners) % 2 else (winners[mid - 1] + winners[mid]) / 2.0
    return median, concentration


def _nearest(value, candidates):
    best = min(candidates, key=lambda c: abs(value - c))
    err_pct = abs(value - best) / best * 100.0 if best else float("inf")
    return best, err_pct


def classify_pulse_width(width_us):
    """Pure arithmetic: check one dominant width against known clocks,
    UART bauds, and the 1-Wire reset pulse; return the plain-English
    verdict line (NOT including the fixed DISCLAIMER - callers show
    that once, alongside, not repeated per line)."""
    freq_as_clock = 1e6 / (2.0 * width_us)   # width = half a clock period
    baud_as_uart = 1e6 / width_us            # width = one UART bit period

    clock_best, clock_err = _nearest(freq_as_clock, I2C_CLOCKS_HZ)
    baud_best, baud_err = _nearest(baud_as_uart, STANDARD_BAUDS)
    onewire_err = abs(width_us - ONEWIRE_RESET_US) / ONEWIRE_RESET_US * 100.0

    kind, guess, err_pct = min(
        [
            ("clock", f"a {clock_best / 1000:.0f} kHz clock", clock_err),
            ("baud", f"{baud_best:.0f} baud", baud_err),
            ("onewire", "a 1-Wire reset pulse (~480 us)", onewire_err),
        ],
        key=lambda c: c[2],
    )

    if err_pct <= GOOD_MATCH_PCT:
        verdict = f"Consistent with {guess} ({err_pct:.1f}% off)."
    elif err_pct <= ROUGH_MATCH_PCT:
        verdict = f"Roughly matches {guess}, but {err_pct:.1f}% off - not a clean match."
    else:
        verdict = (f"Doesn't closely match a common clock, UART baud, or 1-Wire reset. "
                   f"Closest guess: {guess} ({err_pct:.1f}% off).")

    return f"Dominant pulse width {format_duration_us(width_us)}. {verdict}"


def hint_text(widths):
    """Top-line hint text for a pooled/selected pulse-width list.
    Always safe to call, including on an empty list."""
    if not widths:
        return "No pulses captured yet."
    result = dominant_pulse_width(widths)
    if result is None:
        return "No dominant pulse width. Signal looks like noise or contact bounce."
    width_us, _concentration = result
    return classify_pulse_width(width_us)
