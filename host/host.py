#!/usr/bin/env python3
# ============================================================
#  GUI logic-analyser viewer for the Tang Nano 9K edge-capture engine.
#
#  Tkinter only (ships with Python) + pyserial. No other deps.
#
#  ---- wire protocol (must match top.v exactly) ----
#  Every message is [MARKER][PAYLOAD...][CHECKSUM]. The marker byte
#  alone tells you both "this is a frame" and, since payload length is
#  fixed per marker, exactly how many more bytes to expect.
#
#    0xA5 EDGE_RECORD  payload=5B (ts[31:24..7:0], state)      total 7B
#    0xA6 STATUS_REPLY payload=5B (overflow, high_water(2B),
#                                  fifo_depth(2B))              total 7B
#    0xA7 ACK          payload=1B (which command: 'S'/'X'/'R')  total 3B
#    0xA8 ERROR        payload=2B (error code, offending byte)  total 4B
#
#  CHECKSUM = 8-bit sum (mod 256) of marker + every payload byte.
#
#  Resync: on a checksum mismatch (false marker match, or real
#  corruption), advance exactly ONE byte and rescan - not the frame's
#  whole claimed length - so the next real frame is never skipped.
#  FrameParser below is a plain, GUI-free class specifically so this
#  behaviour can be unit-tested without a display (see the project's
#  test script that feeds it deliberately corrupted data).
#
#  Commands TO the FPGA are plain single bytes: 'S' start, 'X' stop,
#  'R' reset timestamp+overflow, 'V' request status. No framing needed
#  on that side - it's a low-rate trusted control channel, not the
#  noisy high-rate stream that actually needs resync robustness.
#
#  Needs pyserial:  pip install pyserial
# ============================================================

import sys
import time
import struct
import threading
import queue
import csv
import bisect

import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import serial
import serial.tools.list_ports


# ---- protocol constants - keep in sync with top.v ----
NUM_CHANNELS = 2          # must match NUM_CHANNELS in top.v
TICKS_PER_US = 27.0       # 27 MHz free-running timestamp counter

MARKER_EDGE = 0xA5
MARKER_STAT = 0xA6
MARKER_ACK  = 0xA7
MARKER_ERR  = 0xA8

# marker -> (name, bytes AFTER the marker i.e. payload+checksum length)
FRAME_INFO = {
    MARKER_EDGE: ("EDGE", 6),
    MARKER_STAT: ("STAT", 6),
    MARKER_ACK:  ("ACK", 2),
    MARKER_ERR:  ("ERR", 3),
}

ERR_CODE_NAMES = {1: "unknown command"}

UI_HZ        = 20
UI_PERIOD_MS = int(1000 / UI_HZ)
STATS_PERIOD_MS = 500      # recompute the stats panel at most twice a second

LOG_MAX_LINES     = 2000
WAVE_MAX_SAMPLES  = 20000   # cap so waveform redraw cost is bounded regardless of session length


# ============================================================
#  Pure frame parser - no I/O, no Tkinter, fully unit-testable.
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


# ============================================================
#  Background thread: read raw bytes off the serial port, hand them
#  to the UI thread via a queue. Never touches a Tkinter widget.
# ============================================================
class SerialReader(threading.Thread):
    def __init__(self, ser, out_queue, stop_event):
        super().__init__(daemon=True)
        self.ser = ser
        self.out_queue = out_queue
        self.stop_event = stop_event

    def run(self):
        while not self.stop_event.is_set():
            try:
                chunk = self.ser.read(4096)
            except (serial.SerialException, OSError) as exc:
                self.out_queue.put(("error", str(exc)))
                return
            if chunk:
                self.out_queue.put(("data", chunk))


# ============================================================
#  Main application
# ============================================================
class App:
    CHANNEL_COLORS = ["#1a73e8", "#e8641a", "#1aa33a", "#a31ae8", "#c0c000", "#e81a5a"]

    def __init__(self, root, initial_port=None):
        self.root = root
        root.title("FPGA Edge Capture Viewer")
        root.geometry("1050x780")

        # ---- connection state ----
        self.ser = None
        self.reader_thread = None
        self.stop_event = None
        self.rx_queue = queue.Queue()
        self.parser = FrameParser()
        self.connected = False

        # ---- capture state / unified record store (live or loaded CSV) ----
        self.prev_ts = None
        self.records = []                  # (ts_ticks, state) - unbounded, source of truth for CSV save + stats
        self.wave_records = []             # capped view of self.records, for waveform redraw cost
        self.recv_times = []               # wall-clock arrival times (monotonic), for edges/sec
        self.records_received = 0
        self.last_record_wall_time = None
        self.overflow_since_reset = False
        self.fifo_high_water = 0
        self.fifo_depth = 0                # reported by the FPGA itself, not assumed

        # ---- waveform view state ----
        self.us_per_pixel = 50.0
        self.view_end_us = None            # right edge of the visible window; None = follow live data
        self.cursor_us = None
        self.markers = []                  # up to 2 placed marker timestamps (us)
        self._pan_last_x = None

        # ---- stats state ----
        self.stats_dirty = True
        self.pulse_widths = {ch: [] for ch in range(NUM_CHANNELS)}
        self.shortest_pulse_us = None
        self.longest_gap_us = None
        self.hist_channel = tk.IntVar(value=0)

        self._build_ui()
        self._refresh_ports(preselect=initial_port)
        self._set_controls_enabled(False)

        self.root.after(UI_PERIOD_MS, self._tick)
        self.root.after(STATS_PERIOD_MS, self._stats_tick)

    # --------------------------------------------------------
    #  UI construction
    # --------------------------------------------------------
    def _build_ui(self):
        top = ttk.Frame(self.root, padding=6)
        top.pack(side="top", fill="x")

        ttk.Label(top, text="Port:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=16, state="readonly")
        self.port_combo.pack(side="left", padx=(2, 4))
        ttk.Button(top, text="Refresh", command=lambda: self._refresh_ports()).pack(side="left")

        ttk.Label(top, text="Baud:").pack(side="left", padx=(10, 0))
        self.baud_var = tk.StringVar(value="115200")
        ttk.Entry(top, textvariable=self.baud_var, width=8).pack(side="left", padx=(2, 4))

        self.connect_btn = ttk.Button(top, text="Connect", command=self._connect)
        self.connect_btn.pack(side="left", padx=(10, 2))
        self.disconnect_btn = ttk.Button(top, text="Disconnect", command=self._disconnect, state="disabled")
        self.disconnect_btn.pack(side="left")

        self.status_light = tk.Label(top, text="●", font=("Helvetica", 16), fg="red")
        self.status_light.pack(side="left", padx=(12, 2))
        self.status_text = ttk.Label(top, text="disconnected")
        self.status_text.pack(side="left")

        ctrl = ttk.Frame(self.root, padding=(6, 0))
        ctrl.pack(side="top", fill="x")

        self.start_btn = ttk.Button(ctrl, text="Start Capture", command=self._cmd_start)
        self.start_btn.pack(side="left", padx=2)
        self.stop_btn = ttk.Button(ctrl, text="Stop Capture", command=self._cmd_stop)
        self.stop_btn.pack(side="left", padx=2)
        self.reset_btn = ttk.Button(ctrl, text="Reset", command=self._cmd_reset)
        self.reset_btn.pack(side="left", padx=2)
        ttk.Button(ctrl, text="Clear Display", command=self._clear_display).pack(side="left", padx=(12, 2))
        ttk.Button(ctrl, text="Save to CSV", command=self._save_csv).pack(side="left", padx=2)
        ttk.Button(ctrl, text="Load CSV", command=self._load_csv).pack(side="left", padx=2)

        counters = ttk.Frame(self.root, padding=6, relief="groove")
        counters.pack(side="top", fill="x", padx=6, pady=4)

        self.count_var = tk.StringVar(value="Records: 0")
        ttk.Label(counters, textvariable=self.count_var, width=14).pack(side="left", padx=8)

        self.rate_var = tk.StringVar(value="0 edges/sec")
        ttk.Label(counters, textvariable=self.rate_var, width=14).pack(side="left", padx=8)

        self.since_var = tk.StringVar(value="Time since last edge: -")
        ttk.Label(counters, textvariable=self.since_var, width=26).pack(side="left", padx=8)

        ttk.Label(counters, text="Overflow since last reset:").pack(side="left", padx=(20, 2))
        self.overflow_label = tk.Label(counters, text="no", width=6, relief="sunken")
        self.overflow_label.pack(side="left")
        self._overflow_ok_bg = self.overflow_label.cget("bg")

        self.resync_var = tk.StringVar(value="resyncs: 0")
        ttk.Label(counters, textvariable=self.resync_var, width=14).pack(side="left", padx=(20, 0))

        # ---- tabbed body: Waveform / Statistics / Log ----
        nb = ttk.Notebook(self.root)
        nb.pack(side="top", fill="both", expand=True, padx=6, pady=(0, 6))

        wave_tab = ttk.Frame(nb)
        stats_tab = ttk.Frame(nb)
        log_tab = ttk.Frame(nb)
        nb.add(wave_tab, text="Waveform")
        nb.add(stats_tab, text="Statistics")
        nb.add(log_tab, text="Log / Diagnostics")

        self._build_wave_tab(wave_tab)
        self._build_stats_tab(stats_tab)
        self._build_log_tab(log_tab)

    def _build_wave_tab(self, parent):
        header = ttk.Frame(parent, padding=(4, 4))
        header.pack(side="top", fill="x")
        ttk.Button(header, text="Zoom In", command=self._zoom_in).pack(side="right", padx=2)
        ttk.Button(header, text="Zoom Out", command=self._zoom_out).pack(side="right", padx=2)
        ttk.Button(header, text="Follow Live", command=self._follow_live).pack(side="right", padx=(12, 2))
        ttk.Button(header, text="Clear Markers", command=self._clear_markers).pack(side="right", padx=2)
        self.zoom_var = tk.StringVar()
        ttk.Label(header, textvariable=self.zoom_var).pack(side="right", padx=8)
        ttk.Label(header, text="Drag to pan, click to place a marker (2 max), scroll wheel to zoom").pack(side="left")

        self.wave_canvas = tk.Canvas(parent, bg="white", height=220, highlightthickness=1,
                                      highlightbackground="#aaaaaa")
        self.wave_canvas.pack(side="top", fill="both", expand=True, padx=4)
        self.wave_canvas.bind("<ButtonPress-1>", self._wave_button_press)
        self.wave_canvas.bind("<B1-Motion>", self._wave_drag)
        self.wave_canvas.bind("<ButtonRelease-1>", self._wave_button_release)
        self.wave_canvas.bind("<Motion>", self._wave_motion)
        self.wave_canvas.bind("<MouseWheel>", self._wave_wheel)     # Windows/Mac
        self.wave_canvas.bind("<Button-4>", lambda e: self._zoom_in())    # Linux scroll up
        self.wave_canvas.bind("<Button-5>", lambda e: self._zoom_out())   # Linux scroll down

        info = ttk.Frame(parent, padding=(4, 2))
        info.pack(side="top", fill="x")
        self.cursor_var = tk.StringVar(value="cursor: -")
        ttk.Label(info, textvariable=self.cursor_var, width=28).pack(side="left")
        self.marker_var = tk.StringVar(value="markers: none placed")
        ttk.Label(info, textvariable=self.marker_var).pack(side="left", padx=12)
        self._update_zoom_label()

    def _build_stats_tab(self, parent):
        top = ttk.Frame(parent, padding=8)
        top.pack(side="top", fill="x")

        self.stat_total_var = tk.StringVar(value="Total edges: 0")
        ttk.Label(top, textvariable=self.stat_total_var, width=20).pack(side="left", padx=8)
        self.stat_shortest_var = tk.StringVar(value="Shortest pulse: -")
        ttk.Label(top, textvariable=self.stat_shortest_var, width=24).pack(side="left", padx=8)
        self.stat_longest_gap_var = tk.StringVar(value="Longest gap: -")
        ttk.Label(top, textvariable=self.stat_longest_gap_var, width=24).pack(side="left", padx=8)

        chan_frame = ttk.Frame(parent, padding=(8, 0))
        chan_frame.pack(side="top", fill="x")
        ttk.Label(chan_frame, text="Histogram channel:").pack(side="left")
        for ch in range(NUM_CHANNELS):
            ttk.Radiobutton(chan_frame, text=f"CH{ch}", variable=self.hist_channel, value=ch,
                             command=self._redraw_histogram).pack(side="left", padx=4)

        ttk.Label(parent, text="Pulse-width histogram (pooled high+low durations) - "
                               "a UART line clusters tightly around one bit period",
                  padding=(8, 4)).pack(side="top", anchor="w")

        self.hist_canvas = tk.Canvas(parent, bg="white", height=280, highlightthickness=1,
                                      highlightbackground="#aaaaaa")
        self.hist_canvas.pack(side="top", fill="both", expand=True, padx=8, pady=(0, 8))
        # the first _redraw_histogram() call can land while this tab isn't
        # visible yet, when winfo_width() is just a 1px placeholder - the
        # draw bails out on that and nothing else was re-triggering it once
        # the tab actually became visible with real dimensions. <Configure>
        # fires whenever the canvas is resized OR first mapped, so bind a
        # redraw there too (not just the stats_dirty-driven one in _stats_tick)
        self.hist_canvas.bind("<Configure>", lambda e: self._redraw_histogram())

    def _build_log_tab(self, parent):
        header = ttk.Frame(parent, padding=(4, 4))
        header.pack(side="top", fill="x")
        self.paused_log = tk.BooleanVar(value=False)
        ttk.Checkbutton(header, text="Pause scrolling", variable=self.paused_log).pack(side="right")
        self.raw_mode = tk.BooleanVar(value=False)
        ttk.Checkbutton(header, text="Show raw bytes (diagnostic mode - bypasses frame parsing)",
                         variable=self.raw_mode, command=self._on_raw_mode_toggle).pack(side="right", padx=12)

        body = ttk.Frame(parent)
        body.pack(side="top", fill="both", expand=True, padx=4, pady=(0, 4))
        self.log_text = tk.Text(body, height=12, state="disabled", wrap="none", font=("Courier New", 10))
        scroll = ttk.Scrollbar(body, orient="vertical", command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scroll.set)
        self.log_text.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")

        self.log_text.tag_configure("record", foreground="black")
        self.log_text.tag_configure("status", foreground="#0060c0")
        self.log_text.tag_configure("ack", foreground="#0a8a3a")
        self.log_text.tag_configure("error", foreground="white", background="#c00000")
        self.log_text.tag_configure("raw", foreground="#a05a00")
        self.log_text.tag_configure("system", foreground="#777777")

    # --------------------------------------------------------
    #  Port list
    # --------------------------------------------------------
    def _refresh_ports(self, preselect=None):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if preselect and preselect in ports:
            self.port_var.set(preselect)
        elif ports:
            for p in ports:
                if "ttyUSB" in p or "ttyACM" in p:
                    self.port_var.set(p)
                    break
            else:
                self.port_var.set(ports[0])

    # --------------------------------------------------------
    #  Connect / disconnect
    # --------------------------------------------------------
    def _connect(self):
        if self.connected:
            return
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("No port selected", "Choose a serial port first.")
            return
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            messagebox.showerror("Bad baud rate", "Baud rate must be an integer.")
            return

        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
        except serial.SerialException as exc:
            messagebox.showerror("Connection failed", str(exc))
            return

        self.stop_event = threading.Event()
        self.reader_thread = SerialReader(self.ser, self.rx_queue, self.stop_event)
        self.reader_thread.start()

        self.connected = True
        self.parser = FrameParser()
        self.prev_ts = None
        self._set_status_light(True)
        self._set_controls_enabled(True)
        self._log_system(f"Connected to {port} @ {baud} baud")

        self._schedule_status_poll()

    def _disconnect(self, reason=None):
        if not self.connected:
            return
        if self.stop_event is not None:
            self.stop_event.set()
        if self.reader_thread is not None:
            self.reader_thread.join(timeout=1.0)
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.reader_thread = None
        self.stop_event = None
        self.connected = False
        self._set_status_light(False)
        self._set_controls_enabled(False)
        self._log_system(f"Disconnected: {reason}" if reason else "Disconnected")

    def _set_status_light(self, connected):
        self.status_light.configure(fg="green" if connected else "red")
        self.status_text.configure(text="connected" if connected else "disconnected")

    def _set_controls_enabled(self, connected):
        self.connect_btn.configure(state="disabled" if connected else "normal")
        self.disconnect_btn.configure(state="normal" if connected else "disabled")
        for b in (self.start_btn, self.stop_btn, self.reset_btn):
            b.configure(state="normal" if connected else "disabled")

    # --------------------------------------------------------
    #  Commands out to the FPGA
    # --------------------------------------------------------
    def _send_byte(self, b):
        if not self.connected or self.ser is None:
            return
        try:
            self.ser.write(bytes([b]))
        except (serial.SerialException, OSError) as exc:
            self._disconnect(reason=str(exc))

    def _cmd_start(self):
        self._send_byte(ord('S'))

    def _cmd_stop(self):
        self._send_byte(ord('X'))

    def _cmd_reset(self):
        self._send_byte(ord('R'))
        self.prev_ts = None
        self.overflow_since_reset = False
        self._log_system("Sent RESET ('R')")

    def _cmd_status(self):
        self._send_byte(ord('V'))

    def _schedule_status_poll(self):
        if not self.connected:
            return
        self._cmd_status()
        self.root.after(1000, self._schedule_status_poll)

    # --------------------------------------------------------
    #  Recurring UI-thread tick
    # --------------------------------------------------------
    def _tick(self):
        try:
            drained = 0
            chunks = []
            while drained < 4096:
                try:
                    kind, payload = self.rx_queue.get_nowait()
                except queue.Empty:
                    break
                drained += 1
                if kind == "error":
                    self._disconnect(reason=payload)
                    break
                elif kind == "data":
                    chunks.append(payload)

            if chunks:
                data = b"".join(chunks)
                if self.raw_mode.get():
                    self._log_raw_bytes(data)
                else:
                    for kind, payload in self.parser.feed(data):
                        self._handle_frame(kind, payload)
                    self.resync_var.set(f"resyncs: {self.parser.resync_count}")

            self._refresh_live_counters()
            self._redraw_waveform()
        except Exception as exc:
            self._log(f"[internal error in UI update: {exc}]", tag="error")
        finally:
            self.root.after(UI_PERIOD_MS, self._tick)

    def _stats_tick(self):
        if self.stats_dirty:
            self._recompute_stats()
            self._refresh_stats_labels()
            self._redraw_histogram()
            self.stats_dirty = False
        self.root.after(STATS_PERIOD_MS, self._stats_tick)

    def _log_raw_bytes(self, data):
        hexline = " ".join(f"{b:02X}" for b in data)
        self._log(f"[RAW] {hexline}", tag="raw")

    # --------------------------------------------------------
    #  Frame dispatch
    # --------------------------------------------------------
    def _handle_frame(self, kind, payload):
        if kind == "EDGE":
            ts, state = decode_edge(payload)
            self._add_record(ts, state)
        elif kind == "STAT":
            overflow, hw, depth = decode_stat(payload)
            if overflow:
                self.overflow_since_reset = True
            self.fifo_high_water = hw
            self.fifo_depth = depth
            self._log(f"[STATUS] overflow_since_reset={'YES' if self.overflow_since_reset else 'no'}  "
                      f"high_water={hw}/{depth}", tag="status")
        elif kind == "ACK":
            cmd = decode_ack(payload)
            try:
                cmd_ch = chr(cmd)
            except ValueError:
                cmd_ch = "?"
            self._log(f"[ACK] {cmd_ch}", tag="ack")
        elif kind == "ERR":
            code, bad = decode_err(payload)
            name = ERR_CODE_NAMES.get(code, f"code {code}")
            self._log(f"[ERROR] {name}: offending byte 0x{bad:02X}", tag="error")

    def _add_record(self, ts, state):
        now = time.monotonic()
        if self.prev_ts is None:
            delta_us = 0.0
        else:
            delta_ticks = (ts - self.prev_ts) & 0xFFFFFFFF
            delta_us = delta_ticks / TICKS_PER_US
        self.prev_ts = ts

        self.records.append((ts, state))
        self.wave_records.append((ts, state))
        if len(self.wave_records) > WAVE_MAX_SAMPLES * 2:
            self.wave_records = self.wave_records[-WAVE_MAX_SAMPLES:]

        self.recv_times.append(now)
        self.last_record_wall_time = now
        self.records_received += 1
        self.stats_dirty = True

        state_bits = format(state, f"0{NUM_CHANNELS}b")
        self._log(f"t={ts:10d}  state={state_bits}  dt={delta_us:9.2f} us", tag="record")

    # --------------------------------------------------------
    #  Log widget
    # --------------------------------------------------------
    def _log(self, text, tag="record"):
        self.log_text.configure(state="normal")
        self.log_text.insert("end", text + "\n", tag)
        line_count = int(self.log_text.index("end-1c").split(".")[0])
        if line_count > LOG_MAX_LINES:
            self.log_text.delete("1.0", "2.0")
        if not self.paused_log.get():
            self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _log_system(self, text):
        self._log(text, tag="system")

    def _on_raw_mode_toggle(self):
        mode = "RAW BYTE" if self.raw_mode.get() else "framed"
        self._log_system(f"Switched to {mode} display mode")

    # --------------------------------------------------------
    #  Live counters
    # --------------------------------------------------------
    def _refresh_live_counters(self):
        now = time.monotonic()
        while self.recv_times and now - self.recv_times[0] > 1.0:
            self.recv_times.pop(0)
        self.rate_var.set(f"{len(self.recv_times)} edges/sec")
        self.count_var.set(f"Records: {self.records_received}")

        if self.last_record_wall_time is None:
            self.since_var.set("Time since last edge: -")
        else:
            elapsed = now - self.last_record_wall_time
            self.since_var.set(f"Time since last edge: {elapsed:6.2f} s")

        if self.overflow_since_reset:
            self.overflow_label.configure(text="YES", fg="white", bg="#c00000")
        else:
            self.overflow_label.configure(text="no", fg="black", bg=self._overflow_ok_bg)

    # --------------------------------------------------------
    #  Statistics
    # --------------------------------------------------------
    def _recompute_stats(self):
        records = self.records
        self.stat_total_var.set(f"Total edges: {len(records)}")

        pulse_widths = {ch: [] for ch in range(NUM_CHANNELS)}
        last_change_ts = [None] * NUM_CHANNELS
        last_level = [None] * NUM_CHANNELS

        for ts, state in records:
            for ch in range(NUM_CHANNELS):
                level = (state >> ch) & 1
                if last_level[ch] is None:
                    last_level[ch] = level
                    last_change_ts[ch] = ts
                elif level != last_level[ch]:
                    width_ticks = (ts - last_change_ts[ch]) & 0xFFFFFFFF
                    pulse_widths[ch].append(width_ticks / TICKS_PER_US)
                    last_level[ch] = level
                    last_change_ts[ch] = ts
        self.pulse_widths = pulse_widths

        all_widths = [w for lst in pulse_widths.values() for w in lst]
        self.shortest_pulse_us = min(all_widths) if all_widths else None

        gaps = []
        for i in range(1, len(records)):
            d = (records[i][0] - records[i - 1][0]) & 0xFFFFFFFF
            gaps.append(d / TICKS_PER_US)
        self.longest_gap_us = max(gaps) if gaps else None

    def _refresh_stats_labels(self):
        if self.shortest_pulse_us is None:
            self.stat_shortest_var.set("Shortest pulse: -")
        else:
            self.stat_shortest_var.set(f"Shortest pulse: {self.shortest_pulse_us:.2f} us")
        if self.longest_gap_us is None:
            self.stat_longest_gap_var.set("Longest gap: -")
        else:
            self.stat_longest_gap_var.set(f"Longest gap: {self.longest_gap_us:.2f} us")

    def _redraw_histogram(self):
        c = self.hist_canvas
        c.delete("all")
        width = c.winfo_width()
        height = c.winfo_height()
        if width < 20 or height < 20:
            return

        ch = self.hist_channel.get()
        widths = self.pulse_widths.get(ch, [])
        pad_l, pad_r, pad_t, pad_b = 50, 10, 10, 30

        if not widths:
            c.create_text(width // 2, height // 2, text="(no pulses yet on this channel)", fill="#888888")
            return

        lo, hi = min(widths), max(widths)
        if hi <= lo:
            hi = lo + 1.0
        nbins = 40
        bin_w = (hi - lo) / nbins
        counts = [0] * nbins
        for w in widths:
            idx = int((w - lo) / bin_w)
            if idx >= nbins:
                idx = nbins - 1
            counts[idx] += 1
        max_count = max(counts)

        plot_w = width - pad_l - pad_r
        plot_h = height - pad_t - pad_b
        bar_w = plot_w / nbins

        for i, cnt in enumerate(counts):
            x0 = pad_l + i * bar_w
            x1 = x0 + bar_w * 0.9
            bar_h = (cnt / max_count) * plot_h if max_count else 0
            y1 = pad_t + plot_h
            y0 = y1 - bar_h
            c.create_rectangle(x0, y0, x1, y1, fill=self.CHANNEL_COLORS[ch % len(self.CHANNEL_COLORS)],
                                outline="")

        # axes
        c.create_line(pad_l, pad_t + plot_h, pad_l + plot_w, pad_t + plot_h, fill="#888888")
        c.create_line(pad_l, pad_t, pad_l, pad_t + plot_h, fill="#888888")
        c.create_text(pad_l, pad_t, anchor="nw", text=f"{max_count}", fill="#888888")
        c.create_text(pad_l, pad_t + plot_h + 4, anchor="nw", text=f"{lo:.2f} us", fill="#555555")
        c.create_text(pad_l + plot_w, pad_t + plot_h + 4, anchor="ne", text=f"{hi:.2f} us", fill="#555555")
        c.create_text(pad_l + plot_w / 2, pad_t + plot_h + 16, anchor="n",
                      text=f"CH{ch} pulse width ({len(widths)} pulses)", fill="#333333")

    # --------------------------------------------------------
    #  Waveform: zoom / pan / cursor / markers
    # --------------------------------------------------------
    def _zoom_in(self):
        self.us_per_pixel = max(self.us_per_pixel / 1.5, 0.05)
        self._update_zoom_label()

    def _zoom_out(self):
        self.us_per_pixel = min(self.us_per_pixel * 1.5, 200000.0)
        self._update_zoom_label()

    def _wave_wheel(self, event):
        if event.delta > 0:
            self._zoom_in()
        else:
            self._zoom_out()

    def _update_zoom_label(self):
        self.zoom_var.set(f"{self.us_per_pixel:.2f} us/pixel")

    def _follow_live(self):
        self.view_end_us = None   # None means "track the latest record", restored on next redraw

    def _clear_markers(self):
        self.markers = []
        self.marker_var.set("markers: none placed")

    def _current_view_end_us(self):
        if self.view_end_us is not None:
            return self.view_end_us
        if self.wave_records:
            return self.wave_records[-1][0] / TICKS_PER_US
        return 0.0

    def _x_to_us(self, x):
        width = self.wave_canvas.winfo_width()
        view_end = self._current_view_end_us()
        window_us = width * self.us_per_pixel
        t0 = view_end - window_us
        return t0 + x * self.us_per_pixel

    def _wave_button_press(self, event):
        self._pan_last_x = event.x
        # placing a marker: click without dragging. We just always
        # place/replace on press and let drag override via _wave_drag
        # moving the view instead - simplest to treat a press as "start
        # of a potential pan", and place a marker only if release
        # happens near the press point (see _wave_button_release)
        self._press_x = event.x

    def _wave_drag(self, event):
        if self._pan_last_x is None:
            return
        dx = event.x - self._pan_last_x
        if dx != 0:
            self.view_end_us = self._current_view_end_us() - dx * self.us_per_pixel
            self._pan_last_x = event.x

    def _wave_button_release(self, event):
        if self._pan_last_x is not None and abs(event.x - self._press_x) < 3:
            # treated as a click, not a drag - place a marker
            t = self._x_to_us(event.x)
            if len(self.markers) >= 2:
                self.markers = []
            self.markers.append(t)
            if len(self.markers) == 2:
                dt = abs(self.markers[1] - self.markers[0])
                self.marker_var.set(f"markers: {self.markers[0]:.2f} us, {self.markers[1]:.2f} us  "
                                    f"(Δ = {dt:.2f} us)")
            else:
                self.marker_var.set(f"markers: {self.markers[0]:.2f} us (place one more)")
        self._pan_last_x = None

    def _wave_motion(self, event):
        t = self._x_to_us(event.x)
        self.cursor_us = t
        self.cursor_var.set(f"cursor: t={t:.2f} us")

    # --------------------------------------------------------
    #  Waveform redraw
    # --------------------------------------------------------
    def _redraw_waveform(self):
        c = self.wave_canvas
        c.delete("wave")
        width = c.winfo_width()
        height = c.winfo_height()
        if width < 10 or height < 10:
            return

        view_end = self._current_view_end_us()
        window_us = max(width * self.us_per_pixel, 1.0)
        t0 = view_end - window_us

        row_h = height / NUM_CHANNELS
        pad = 6

        if self.wave_records:
            # find the last sample at/before t0 so the trace starts at
            # the right level instead of defaulting to 0
            ts_list = [r[0] / TICKS_PER_US for r in self.wave_records]
            start_idx = bisect.bisect_right(ts_list, t0) - 1
            if start_idx < 0:
                start_idx = 0

            for ch in range(NUM_CHANNELS):
                y_hi = ch * row_h + pad
                y_lo = (ch + 1) * row_h - pad

                level = (self.wave_records[start_idx][1] >> ch) & 1
                y = y_hi if level else y_lo
                pts = [0.0, y]

                for ts, state in self.wave_records[start_idx:]:
                    t = ts / TICKS_PER_US
                    if t < t0:
                        continue
                    if t > view_end:
                        break
                    xt = (t - t0) / self.us_per_pixel
                    newlevel = (state >> ch) & 1
                    newy = y_hi if newlevel else y_lo
                    pts += [xt, y]
                    pts += [xt, newy]
                    y = newy

                pts += [width, y]

                color = self.CHANNEL_COLORS[ch % len(self.CHANNEL_COLORS)]
                if len(pts) >= 4:
                    c.create_line(*pts, fill=color, width=2, tags="wave")
                c.create_text(pad, ch * row_h + pad, anchor="nw", text=f"CH{ch}",
                              fill="#666666", tags="wave")
                c.create_line(0, (ch + 1) * row_h, width, (ch + 1) * row_h,
                              fill="#dddddd", tags="wave")

        # markers
        for mt in self.markers:
            if t0 <= mt <= view_end:
                mx = (mt - t0) / self.us_per_pixel
                c.create_line(mx, 0, mx, height, fill="#cc00cc", dash=(4, 2), tags="wave")

        if len(self.markers) == 2 and t0 <= self.markers[0] <= view_end and t0 <= self.markers[1] <= view_end:
            x0 = (self.markers[0] - t0) / self.us_per_pixel
            x1 = (self.markers[1] - t0) / self.us_per_pixel
            c.create_line(x0, 14, x1, 14, fill="#cc00cc", arrow="both", tags="wave")

    # --------------------------------------------------------
    #  Clear Display / Save / Load
    # --------------------------------------------------------
    def _clear_display(self):
        self.records.clear()
        self.wave_records.clear()
        self.recv_times.clear()
        self.records_received = 0
        self.prev_ts = None
        self.last_record_wall_time = None
        self.markers = []
        self.marker_var.set("markers: none placed")
        self.view_end_us = None
        self.stats_dirty = True

        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")
        self.wave_canvas.delete("wave")

        self._log_system("Display cleared (FPGA state untouched - use Reset to reset the timestamp counter)")

    def _save_csv(self):
        if not self.records:
            messagebox.showinfo("Nothing to save", "No records to save.")
            return
        default_name = f"capture_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            initialfile=default_name,
        )
        if not path:
            return
        try:
            with open(path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["timestamp_ticks", "state_binary", "delta_us"])
                prev = None
                for ts, state in self.records:
                    if prev is None:
                        delta_us = 0.0
                    else:
                        delta_us = ((ts - prev) & 0xFFFFFFFF) / TICKS_PER_US
                    prev = ts
                    w.writerow([ts, format(state, f"0{NUM_CHANNELS}b"), f"{delta_us:.2f}"])
            self._log_system(f"Saved {len(self.records)} records to {path}")
        except OSError as exc:
            messagebox.showerror("Save failed", str(exc))

    def _load_csv(self):
        path = filedialog.askopenfilename(
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not path:
            return
        try:
            loaded = []
            with open(path, newline="") as f:
                r = csv.reader(f)
                header = next(r, None)
                for row in r:
                    if len(row) < 2:
                        continue
                    ts = int(row[0])
                    state = int(row[1], 2)
                    loaded.append((ts, state))
        except (OSError, ValueError) as exc:
            messagebox.showerror("Load failed", str(exc))
            return

        if not loaded:
            messagebox.showinfo("Empty file", "No records found in that CSV.")
            return

        self._clear_display()
        self.records = loaded
        self.wave_records = loaded[-WAVE_MAX_SAMPLES:]
        self.records_received = len(loaded)
        self.count_var.set(f"Records: {self.records_received}")
        self.stats_dirty = True
        self.view_end_us = loaded[-1][0] / TICKS_PER_US   # view the loaded data, not "live"
        self._log_system(f"Loaded {len(loaded)} records from {path} (offline view - not connected to live data)")

    # --------------------------------------------------------
    def shutdown(self):
        self._disconnect()


def main():
    initial_port = sys.argv[1] if len(sys.argv) > 1 else None
    root = tk.Tk()
    app = App(root, initial_port=initial_port)

    def on_close():
        app.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
