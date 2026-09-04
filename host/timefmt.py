# ============================================================
#  timefmt.py - format a duration/timestamp given in real microseconds
#  (already converted from raw ticks - see protocol.TICKS_PER_US) using
#  whichever of ns/us/ms/s reads naturally for its magnitude.
#
#  Shared by host.py (waveform axis, cursor/marker readouts, stats
#  panel) and hint.py (the hint sentence) so every place a timestamp is
#  displayed agrees on the same formatting - no risk of e.g. the axis
#  saying "5.02 ms" while a label next to it says "5020.00 us" for the
#  same instant.
# ============================================================

# (unit name, upper bound in us before switching to the next unit up)
_UNIT_THRESHOLDS = [
    ("ns", 1.0),
    ("us", 1_000.0),
    ("ms", 1_000_000.0),
    ("s", float("inf")),
]


def best_unit_for(value_us):
    """Pick the unit that reads best for this magnitude. Returns
    (unit_name, scale) where value_us / scale is the number to show in
    that unit (e.g. best_unit_for(5000) -> ("ms", 1000.0), meaning
    5000us / 1000.0 = 5.0 ms)."""
    abs_v = abs(value_us)
    if abs_v < 1.0:
        return "ns", 0.001
    if abs_v < 1_000.0:
        return "us", 1.0
    if abs_v < 1_000_000.0:
        return "ms", 1_000.0
    return "s", 1_000_000.0


def format_duration_us(value_us, decimals=2):
    """value_us: a duration or absolute timestamp, already in real
    microseconds. Returns e.g. "5.02 us", "364.09 ms", "1.823 s"."""
    unit, scale = best_unit_for(value_us)
    return f"{value_us / scale:.{decimals}f} {unit}"
