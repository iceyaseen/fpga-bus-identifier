# ============================================================
#  protocol.py - wire protocol for the Tang Nano 9K edge-capture engine.
#  No GUI, no serial I/O - pure parsing so it can be unit-tested and
#  reused by any front end (this is the same FrameParser previously
#  embedded in host.py, unchanged, just factored out for the PySide6
#  rewrite - see host.py for the module this feeds).
#
#  ---- wire protocol (must match rtl/top.v exactly) ----
#  Every message is [MARKER][PAYLOAD...][CHECKSUM]. The marker byte
#  alone tells you both "this is a frame" and, since payload length is
#  fixed per marker, exactly how many more bytes to expect.
#
#    0xA5 EDGE_RECORD     payload=5B (ts[31:24..7:0], state)         total 7B
#    0xA6 STATUS_REPLY    payload=5B (overflow, high_water(2B),
#                                     fifo_depth(2B))                 total 7B
#    0xA7 ACK             payload=1B (which command)                 total 3B
#    0xA8 ERROR           payload=2B (error code, offending byte)    total 4B
#    0xA9 PRESCAN_RESULT  payload=2B (channel, category)             total 4B
#    0xAA ADDR_HIT        payload=1B (7-bit address that ACKed)      total 3B
#
#  CHECKSUM = 8-bit sum (mod 256) of marker + every payload byte.
#
#  Resync: on a checksum mismatch (false marker match, or real
#  corruption), advance exactly ONE byte and rescan - not the frame's
#  whole claimed length - so the next real frame is never skipped.
#  FrameParser below is a plain, GUI-free class specifically so this
#  behaviour can be unit-tested without a display (see
#  test_frame_parser.py).
#
#  Commands TO the FPGA are plain single bytes - no framing needed on
#  that side, it's a low-rate trusted control channel, not the noisy
#  high-rate stream that actually needs resync robustness:
#    'S' start capture         'A'/'a' channel A pull-up on/off
#    'X' stop capture          'B'/'b' channel B pull-up on/off
#    'R' reset timestamp+ovf   'N'/'W' normal/swapped SDA-SCL pin mapping
#    'V' request status        'E' run electrical pre-scan (Part 3)
#                               'I' run I2C address scan (Part 5)
#
#  'E'/'I' are long-running: their ACK arrives only once the operation
#  they kicked off actually finishes (see top.v), not when the command
#  byte was received - so the host can safely treat that ACK as "the
#  pre-scan/scan is done, whatever PRESCAN_RESULT/ADDR_HIT frames it
#  was going to send have already been sent (or are the very next
#  bytes in flight)".
# ============================================================
import struct

NUM_CHANNELS = 2          # must match NUM_CHANNELS in rtl/top.v
TICKS_PER_US = 27.0       # 27 MHz free-running timestamp counter
FIFO_DEPTH_DEFAULT = 256  # reported by the FPGA itself in STATUS_REPLY; this is only a display fallback before the first status arrives

MARKER_EDGE = 0xA5
MARKER_STAT = 0xA6
MARKER_ACK = 0xA7
MARKER_ERR = 0xA8
MARKER_PRESCAN = 0xA9
MARKER_ADDRHIT = 0xAA

# marker -> (name, bytes AFTER the marker i.e. payload+checksum length)
FRAME_INFO = {
    MARKER_EDGE: ("EDGE", 6),
    MARKER_STAT: ("STAT", 6),
    MARKER_ACK: ("ACK", 2),
    MARKER_ERR: ("ERR", 3),
    MARKER_PRESCAN: ("PRESCAN", 3),
    MARKER_ADDRHIT: ("ADDRHIT", 2),
}

ERR_CODE_NAMES = {1: "unknown command", 2: "busy (pre-scan/scan already running)"}

# pre-scan category codes (top.v's i2c_prescan.v) -> plain-language label
PRESCAN_CATEGORY_NAMES = {
    0: "has its own pull-up",
    1: "needs ours",
    2: "held low (device driving it, or a short)",
    3: "floating (nothing connected)",
}


# ============================================================
#  Pure frame parser - no I/O, no GUI, fully unit-testable.
#  Feed it bytes as they arrive; get back a list of (kind, payload)
#  events. Tracks resync/checksum-failure counts for diagnostics.
# ============================================================
class FrameParser:
    def __init__(self):
        self.buf = bytearray()
        self.resync_count = 0          # bytes dropped while scanning for a marker
        self.checksum_fail_count = 0   # frames that matched a marker but failed checksum

    def feed(self, data):
        self.buf.extend(data)
        events = []
        while self.buf:
            b0 = self.buf[0]
            info = FRAME_INFO.get(b0)
            if info is None:
                # not a marker byte at all - this IS the resync scan
                del self.buf[0]
                self.resync_count += 1
                continue

            _name, rest_len = info
            total_len = 1 + rest_len
            if len(self.buf) < total_len:
                break   # wait for the rest of the frame on the next feed()

            frame = bytes(self.buf[:total_len])
            marker = frame[0]
            payload = frame[1:-1]
            checksum = frame[-1]
            computed = (marker + sum(payload)) & 0xFF

            if computed == checksum:
                events.append((FRAME_INFO[marker][0], payload))
                del self.buf[:total_len]
            else:
                # false marker match, or real corruption - resync by
                # exactly one byte, NOT the whole claimed frame length,
                # so a real frame overlapping this window isn't skipped
                self.checksum_fail_count += 1
                self.resync_count += 1
                del self.buf[0]
        return events


def decode_edge(payload):
    ts = struct.unpack(">I", payload[0:4])[0]
    state = payload[4]
    return ts, state


def decode_stat(payload):
    overflow = payload[0]
    high_water = struct.unpack(">H", payload[1:3])[0]
    depth = struct.unpack(">H", payload[3:5])[0]
    return overflow, high_water, depth


def decode_ack(payload):
    return payload[0]


def decode_err(payload):
    return payload[0], payload[1]


def decode_prescan(payload):
    channel = payload[0]      # 0 = A, 1 = B
    category = payload[1]     # see PRESCAN_CATEGORY_NAMES
    return channel, category


def decode_addrhit(payload):
    return payload[0]         # the 7-bit address that ACKed
