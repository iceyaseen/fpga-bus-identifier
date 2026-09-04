#!/usr/bin/env python3
# ============================================================
#  Fuzz test for host.py's FrameParser: feeds it deliberately
#  corrupted data (truncated records, random bytes injected
#  mid-stream, a byte stream chopped into arbitrary chunk sizes)
#  and checks it always resynchronises to the next valid frame
#  rather than emitting a wrong/garbage record.
#
#  Run: python3 host/test_frame_parser.py
# ============================================================
import os
import random
import struct
import sys

sys.path.insert(0, os.path.dirname(__file__))
from protocol import FrameParser, MARKER_EDGE, MARKER_STAT, MARKER_ACK, MARKER_ERR


def build_edge(ts, state):
    payload = struct.pack(">I", ts & 0xFFFFFFFF) + bytes([state])
    checksum = (MARKER_EDGE + sum(payload)) & 0xFF
    return bytes([MARKER_EDGE]) + payload + bytes([checksum])


def build_stat(overflow, hw, depth):
    payload = bytes([overflow]) + struct.pack(">H", hw) + struct.pack(">H", depth)
    checksum = (MARKER_STAT + sum(payload)) & 0xFF
    return bytes([MARKER_STAT]) + payload + bytes([checksum])


def build_ack(cmd):
    checksum = (MARKER_ACK + cmd) & 0xFF
    return bytes([MARKER_ACK, cmd, checksum])


def build_err(code, bad):
    checksum = (MARKER_ERR + code + bad) & 0xFF
    return bytes([MARKER_ERR, code, bad, checksum])


def feed_in_random_chunks(parser, data):
    """Feed the whole buffer through parser.feed(), split at random
    byte boundaries, to also exercise the 'wait for the rest of the
    frame on the next feed()' partial-frame path."""
    events = []
    i = 0
    while i < len(data):
        n = random.randint(1, 7)
        chunk = data[i:i + n]
        events.extend(parser.feed(chunk))
        i += n
    return events


def expect(cond, msg, failures):
    if not cond:
        failures.append(msg)
        print(f"FAIL: {msg}")


def main():
    random.seed(1234)
    failures = []

    # ---- test 1: clean stream, no corruption - sanity baseline ----
    parser = FrameParser()
    frames = [build_edge(i, i % 4) for i in range(20)]
    stream = b"".join(frames)
    events = feed_in_random_chunks(parser, stream)
    expect(len(events) == 20, f"clean stream: expected 20 EDGE events, got {len(events)}", failures)
    expect(parser.resync_count == 0, f"clean stream: expected 0 resyncs, got {parser.resync_count}", failures)
    for i, (kind, payload) in enumerate(events):
        ts = struct.unpack(">I", payload[0:4])[0]
        state = payload[4]
        expect(kind == "EDGE" and ts == i and state == i % 4,
               f"clean stream: event {i} decoded wrong: kind={kind} ts={ts} state={state}", failures)

    # ---- test 2: truncated record at end of stream (no next frame
    #      arrives at all) - parser must not emit anything for it, and
    #      must not crash; a later feed() with the rest completes it ----
    parser = FrameParser()
    good = build_edge(42, 3)
    truncated = build_edge(99, 1)[:4]   # marker + 3 payload bytes, missing the rest
    events = parser.feed(good + truncated)
    expect(len(events) == 1, f"truncated tail: expected 1 event before truncation, got {len(events)}", failures)
    expect(parser.buf == bytearray(truncated), "truncated tail: partial frame should stay buffered, not be discarded", failures)
    # now the rest of that frame arrives
    rest = build_edge(99, 1)[4:]
    events2 = parser.feed(rest)
    expect(len(events2) == 1, f"truncated tail: expected the completed frame after the rest arrives, got {len(events2)}", failures)

    # ---- test 3: random garbage bytes injected between valid frames -
    #      parser must resync (report resyncs) and still recover every
    #      valid frame that follows, never a garbage/mismatched record ----
    parser = FrameParser()
    rng = random.Random(99)
    expected = []
    stream = bytearray()
    for i in range(200):
        # inject 0-5 random bytes before each real frame
        junk_len = rng.randint(0, 5)
        junk = bytes(rng.randint(0, 255) for _ in range(junk_len))
        stream += junk
        ts = i * 37
        state = i % 4
        stream += build_edge(ts, state)
        expected.append((ts, state))

    events = feed_in_random_chunks(parser, bytes(stream))
    expect(len(events) == len(expected),
           f"random junk injection: expected {len(expected)} EDGE events, got {len(events)}", failures)
    for i, ((kind, payload), (exp_ts, exp_state)) in enumerate(zip(events, expected)):
        ts = struct.unpack(">I", payload[0:4])[0]
        state = payload[4]
        expect(kind == "EDGE" and ts == exp_ts and state == exp_state,
               f"random junk injection: event {i} mismatch: got kind={kind} ts={ts} state={state}, "
               f"expected ts={exp_ts} state={exp_state}", failures)
    expect(parser.resync_count > 0, "random junk injection: expected resync_count > 0 given injected junk", failures)

    # ---- test 4: a byte stream that happens to contain a marker value
    #      as random junk (false marker match) followed by a bad
    #      checksum - parser must resync by exactly ONE byte, not skip
    #      the whole claimed frame length, so a real frame overlapping
    #      that window is never lost ----
    parser = FrameParser()
    real1 = build_edge(10, 1)
    real2 = build_edge(20, 2)
    # a byte sequence that starts with MARKER_EDGE but is NOT a valid
    # frame (bad checksum) - the false marker match itself
    fake = bytes([MARKER_EDGE, 0x11, 0x22, 0x33, 0x44, 0x00])  # checksum deliberately wrong
    stream = real1 + fake + real2
    events = parser.feed(stream)
    expect(len(events) == 2, f"false marker match: expected 2 real EDGE events, got {len(events)}", failures)
    expect(parser.checksum_fail_count >= 1,
           f"false marker match: expected checksum_fail_count >= 1, got {parser.checksum_fail_count}", failures)
    if len(events) == 2:
        ts0 = struct.unpack(">I", events[0][1][0:4])[0]
        ts1 = struct.unpack(">I", events[1][1][0:4])[0]
        expect(ts0 == 10 and ts1 == 20,
               f"false marker match: expected timestamps 10,20 got {ts0},{ts1}", failures)

    # ---- test 5: mixed frame types (STAT/ACK/ERR) interleaved with
    #      EDGE, plus junk, still all decode correctly and in order ----
    parser = FrameParser()
    stream = bytearray()
    stream += os.urandom(3)
    stream += build_edge(1, 1)
    stream += build_ack(ord('S'))
    stream += os.urandom(2)
    stream += build_stat(1, 57, 256)
    stream += build_err(1, 0x5A)
    stream += build_edge(2, 2)
    events = feed_in_random_chunks(parser, bytes(stream))
    kinds = [k for k, _ in events]
    expect(kinds == ["EDGE", "ACK", "STAT", "ERR", "EDGE"],
           f"mixed frame types: expected [EDGE,ACK,STAT,ERR,EDGE], got {kinds}", failures)

    # ---- test 6: pure random noise, no valid frames at all - parser
    #      must terminate (not hang/crash) and emit zero events ----
    parser = FrameParser()
    noise = bytes(rng.randint(0, 255) for _ in range(5000))
    events = feed_in_random_chunks(parser, noise)
    # a handful of "events" could coincidentally form a valid checksum
    # by pure chance out of 5000 random bytes; the real invariant is
    # "doesn't crash / doesn't hang", already implicitly proven by
    # reaching this point - just sanity-check it's a tiny fraction
    expect(len(events) < 20, f"pure noise: suspiciously many ({len(events)}) 'valid' frames out of noise", failures)

    print()
    if failures:
        print(f"---- {len(failures)} FAILURE(S) ----")
        sys.exit(1)
    else:
        print("---- all frame-parser resync/corruption tests passed ----")


if __name__ == "__main__":
    main()
