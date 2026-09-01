#!/usr/bin/env python3
# ============================================================
#  GUI viewer for the Tang Nano 9K edge-capture engine.
#
#  Tkinter only (ships with Python) + pyserial. No other deps.
#
#  Wire protocol (must match top.v / edge_capture.v / fifo.v exactly -
#  see those files if you change the FPGA side):
#
#    record frame = 0xA5, ts[31:24], ts[23:16], ts[15:8], ts[7:0], state
#                   (6 bytes: 1 marker + 4-byte timestamp + 1-byte state)
#    status frame = 0xA6, overflow(0/1), high_water_hi, high_water_lo
#                   (4 bytes: 1 marker + 3 payload bytes)
#
#  The 0xA5/0xA6 marker byte exists because these binary frames share
#  the one UART wire with the FPGA's plain-ASCII "Hello from FPGA"
#  message and byte-echo debug feature - without a marker there would
#  be no way to tell a record apart from stray text.
#
#  Commands sent TO the FPGA are single bytes: 'S' start, 'X' stop,
#  'R' reset timestamp+overflow, 'V' request a status frame.
#
#  ---- threading model ----
#  A background thread (SerialReader) does nothing but call
#  ser.read() in a loop and push whatever bytes arrive onto a
#  queue.Queue. It NEVER touches a Tkinter widget - that would be
#  unsafe, Tkinter is not thread-safe.
#
#  The Tk mainloop drains that queue on a timer (`_tick`, every 50ms
#  = 20Hz) and does all parsing + all widget updates from the main
#  thread. This is also where the "batch updates, don't redraw per
#  record" requirement is satisfied: no matter how many records
#  arrived in the last 50ms, the UI only redraws once.
# ============================================================

import sys
import time
import struct
import threading
import queue
import csv

import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import serial
import serial.tools.list_ports


# ---- protocol constants - keep these in sync with top.v -------
NUM_CHANNELS = 2          # low NUM_CHANNELS bits of the state byte are real
REC_MARKER   = 0xA5
REC_LEN      = 6          # marker + 4-byte timestamp + 1-byte state
STAT_MARKER  = 0xA6
STAT_LEN     = 4          # marker + overflow + high_water_hi + high_water_lo
TICKS_PER_US = 27.0       # 27 MHz free-running timestamp counter

UI_HZ        = 20         # UI redraw/queue-drain rate
UI_PERIOD_MS = int(1000 / UI_HZ)

LOG_MAX_LINES  = 2000      # cap so a long session doesn't bog down the Text widget
WAVE_MAX_SAMPLES = 5000    # cap so waveform redraw cost doesn't grow with session length


# ============================================================
#  Background thread: read raw bytes off the serial port, hand
#  them to the UI thread via a queue. This is the ONLY thing
#  that runs off the main thread.
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
                # ser was opened with a short timeout, so this returns
                # (possibly empty) roughly every timeout period, which
                # is also how often we get a chance to notice stop_event
                chunk = self.ser.read(4096)
            except (serial.SerialException, OSError) as exc:
                # covers an unplugged USB adapter, a closed port, etc.
                self.out_queue.put(("error", str(exc)))
                return
            if chunk:
                self.out_queue.put(("data", chunk))
        # stop_event was set by the UI thread (Disconnect) - just exit,
        # closing the port is the UI thread's job


# ============================================================
#  Main application
# ============================================================
class App:
    CHANNEL_COLORS = ["#1a73e8", "#e8641a", "#1aa33a", "#a31ae8", "#c0c000", "#e81a5a"]

    def __init__(self, root, initial_port=None):
        self.root = root
        root.title("FPGA Edge Capture Viewer")
        root.geometry("1000x720")

        # ---- connection state ----
        self.ser = None
        self.reader_thread = None
        self.stop_event = None
        self.rx_queue = queue.Queue()
        self.rx_buf = bytearray()
        self.connected = False

        # ---- capture state ----
        self.prev_ts = None                # for delta-us computation
        self.all_records = []              # (ts_ticks, state, delta_us) - full history, for CSV
        self.wave_records = []             # (ts_ticks, state) - capped, for the waveform only
        self.recv_times = []               # wall-clock arrival times (monotonic), for edges/sec
        self.records_received = 0
        self.last_record_wall_time = None
        self.overflow = False
        self.high_water = 0

        # ---- waveform view state ----
        self.us_per_pixel = 50.0           # zoom level: microseconds represented by one pixel

        self._build_ui()
        self._refresh_ports(preselect=initial_port)
        self._set_controls_enabled(False)

        # kick off the recurring UI update loop - runs for the life of
        # the app, whether or not we're connected
        self.root.after(UI_PERIOD_MS, self._tick)

    # --------------------------------------------------------
    #  UI construction
    # --------------------------------------------------------
    def _build_ui(self):
        # ---- top bar: port/baud/connect + status light ----
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

        # status light: a coloured dot, red = disconnected, green = connected
        self.status_light = tk.Label(top, text="●", font=("Helvetica", 16), fg="red")
        self.status_light.pack(side="left", padx=(12, 2))
        self.status_text = ttk.Label(top, text="disconnected")
        self.status_text.pack(side="left")

        # ---- control buttons ----
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

        # ---- live counters ----
        counters = ttk.Frame(self.root, padding=6, relief="groove")
        counters.pack(side="top", fill="x", padx=6, pady=4)

        self.count_var = tk.StringVar(value="Records: 0")
        ttk.Label(counters, textvariable=self.count_var, width=16).pack(side="left", padx=8)

        self.rate_var = tk.StringVar(value="0 edges/sec")
        ttk.Label(counters, textvariable=self.rate_var, width=16).pack(side="left", padx=8)

        self.since_var = tk.StringVar(value="Time since last edge: -")
        ttk.Label(counters, textvariable=self.since_var, width=26).pack(side="left", padx=8)

        ttk.Label(counters, text="Overflow:").pack(side="left", padx=(20, 2))
        self.overflow_label = tk.Label(counters, text="ok", width=10, relief="sunken")
        self.overflow_label.pack(side="left")
        # remember the platform's real default background instead of using a
        # hardcoded name - "SystemButtonFace" etc. are Windows-only in Tk and
        # raise TclError on Linux/Mac
        self._overflow_ok_bg = self.overflow_label.cget("bg")

        # ---- waveform ----
        wave_frame = ttk.Frame(self.root, padding=6)
        wave_frame.pack(side="top", fill="both", expand=True, padx=6)

        wave_header = ttk.Frame(wave_frame)
        wave_header.pack(side="top", fill="x")
        ttk.Label(wave_header, text="Waveform (scrolls right-to-left, most recent edge on the right)").pack(side="left")
        ttk.Button(wave_header, text="Zoom In", command=self._zoom_in).pack(side="right", padx=2)
        ttk.Button(wave_header, text="Zoom Out", command=self._zoom_out).pack(side="right", padx=2)
        self.zoom_var = tk.StringVar()
        ttk.Label(wave_header, textvariable=self.zoom_var).pack(side="right", padx=8)

        self.wave_canvas = tk.Canvas(wave_frame, bg="white", height=180, highlightthickness=1,
                                      highlightbackground="#aaaaaa")
        self.wave_canvas.pack(side="top", fill="both", expand=True, pady=(4, 0))
        self._update_zoom_label()

        # ---- raw log panel ----
        log_frame = ttk.Frame(self.root, padding=6)
        log_frame.pack(side="top", fill="both", expand=True, padx=6, pady=(0, 6))

        log_header = ttk.Frame(log_frame)
        log_header.pack(side="top", fill="x")
        ttk.Label(log_header, text="Log").pack(side="left")

        self.paused_log = tk.BooleanVar(value=False)
        ttk.Checkbutton(log_header, text="Pause scrolling", variable=self.paused_log).pack(side="right")

        self.raw_mode = tk.BooleanVar(value=False)
        ttk.Checkbutton(log_header, text="Show raw bytes (diagnostic mode - bypasses record parsing)",
                         variable=self.raw_mode, command=self._on_raw_mode_toggle).pack(side="right", padx=12)

        log_body = ttk.Frame(log_frame)
        log_body.pack(side="top", fill="both", expand=True)
        self.log_text = tk.Text(log_body, height=12, state="disabled", wrap="none",
                                 font=("Courier New", 10))
        log_scroll = ttk.Scrollbar(log_body, orient="vertical", command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=log_scroll.set)
        self.log_text.pack(side="left", fill="both", expand=True)
        log_scroll.pack(side="right", fill="y")

        self.log_text.tag_configure("record", foreground="black")
        self.log_text.tag_configure("status", foreground="#0060c0")
        self.log_text.tag_configure("status_bad", foreground="white", background="#c00000")
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
            # prefer a ttyUSB/ttyACM candidate (the FPGA's USB-serial adapter)
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
            # short read timeout: lets the background thread notice a
            # disconnect/stop request promptly instead of blocking forever
            self.ser = serial.Serial(port, baud, timeout=0.05)
        except serial.SerialException as exc:
            messagebox.showerror("Connection failed", str(exc))
            return

        self.stop_event = threading.Event()
        self.reader_thread = SerialReader(self.ser, self.rx_queue, self.stop_event)
        self.reader_thread.start()

        self.connected = True
        self.rx_buf.clear()
        self.prev_ts = None
        self._set_status_light(True)
        self._set_controls_enabled(True)
        self._log_system(f"Connected to {port} @ {baud} baud")

        # keep the overflow/high-water reading fresh on its own - 'V' is
        # a cheap 4-byte reply and low priority on the FPGA side, so
        # polling it doesn't meaningfully compete with capture records
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
    #  Commands out to the FPGA (single bytes, called from the UI
    #  thread - fine to write from here while the reader thread only
    #  ever reads, pyserial doesn't need extra locking for that split)
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
        self._log_system("Sent START ('S')")

    def _cmd_stop(self):
        self._send_byte(ord('X'))
        self._log_system("Sent STOP ('X')")

    def _cmd_reset(self):
        self._send_byte(ord('R'))
        self.prev_ts = None   # first record after a reset shouldn't show a bogus huge delta
        self._log_system("Sent RESET ('R') - FPGA timestamp counter + overflow cleared")

    def _cmd_status(self):
        self._send_byte(ord('V'))

    def _schedule_status_poll(self):
        if not self.connected:
            return
        self._cmd_status()
        self.root.after(1000, self._schedule_status_poll)

    # --------------------------------------------------------
    #  The recurring UI-thread tick: drain the queue, parse, redraw.
    #  Everything Tkinter-related happens only in here (or in things
    #  it calls), i.e. only ever on the main thread.
    # --------------------------------------------------------
    def _tick(self):
        # the whole body is guarded: _tick reschedules itself at the very
        # end, so ANY unhandled exception in here (a bad widget option, a
        # parsing edge case, anything) would otherwise silently kill every
        # future redraw/queue-drain for the rest of the session, with no
        # crash and no obvious symptom besides "nothing updates anymore".
        # Learned this the hard way in testing - see git history.
        try:
            # bound how much we drain per tick so a byte flood can't make
            # a single _tick call run indefinitely - leftovers just get
            # picked up on the next tick, 50ms later
            drained = 0
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
                    self.rx_buf.extend(payload)

            if self.raw_mode.get():
                self._drain_raw_bytes()
            else:
                self._drain_framed_records()

            self._refresh_live_counters()
            self._redraw_waveform()
        except Exception as exc:
            self._log(f"[internal error in UI update: {exc}]", tag="raw")
        finally:
            self.root.after(UI_PERIOD_MS, self._tick)

    # --------------------------------------------------------
    #  Diagnostic mode: no framing at all, just hex-dump every byte
    # --------------------------------------------------------
    def _drain_raw_bytes(self):
        if not self.rx_buf:
            return
        hexline = " ".join(f"{b:02X}" for b in self.rx_buf)
        self._log(f"[RAW] {hexline}", tag="raw")
        self.rx_buf.clear()

    # --------------------------------------------------------
    #  Normal mode: scan for 0xA5/0xA6 frames; anything else is
    #  shown as hex instead of being silently dropped
    # --------------------------------------------------------
    def _drain_framed_records(self):
        junk = bytearray()
        while self.rx_buf:
            b0 = self.rx_buf[0]
            if b0 == REC_MARKER:
                if len(self.rx_buf) < REC_LEN:
                    break   # wait for the rest of the frame next tick
                frame = bytes(self.rx_buf[:REC_LEN])
                del self.rx_buf[:REC_LEN]
                self._handle_record(frame)
            elif b0 == STAT_MARKER:
                if len(self.rx_buf) < STAT_LEN:
                    break
                frame = bytes(self.rx_buf[:STAT_LEN])
                del self.rx_buf[:STAT_LEN]
                self._handle_status(frame)
            else:
                junk.append(self.rx_buf.pop(0))

        if junk:
            hexline = " ".join(f"{b:02X}" for b in junk)
            ascii_preview = "".join(chr(b) if 32 <= b < 127 else "." for b in junk)
            self._log(f"[unframed] {hexline}   ascii: {ascii_preview}", tag="raw")

    def _handle_record(self, frame):
        ts, state = struct.unpack(">IB", frame[1:])
        now = time.monotonic()

        if self.prev_ts is None:
            delta_us = 0.0
        else:
            delta_ticks = (ts - self.prev_ts) & 0xFFFFFFFF   # 32-bit counter wraparound
            delta_us = delta_ticks / TICKS_PER_US
        self.prev_ts = ts

        self.all_records.append((ts, state, delta_us))
        self.wave_records.append((ts, state))
        if len(self.wave_records) > WAVE_MAX_SAMPLES * 2:
            self.wave_records = self.wave_records[-WAVE_MAX_SAMPLES:]

        self.recv_times.append(now)
        self.last_record_wall_time = now
        self.records_received += 1

        state_bits = format(state, f"0{NUM_CHANNELS}b")
        self._log(f"t={ts:10d}  state={state_bits}  dt={delta_us:9.2f} us", tag="record")

    def _handle_status(self, frame):
        overflow, hw_hi, hw_lo = frame[1], frame[2], frame[3]
        self.overflow = bool(overflow)
        self.high_water = (hw_hi << 8) | hw_lo
        tag = "status_bad" if overflow else "status"
        self._log(f"[STATUS] overflow={'YES' if overflow else 'no'}  high_water={self.high_water}", tag=tag)

    # --------------------------------------------------------
    #  Log widget helper - pausing stops auto-scroll, not the
    #  actual appending, so nothing is lost while you're reading
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
        # switching modes mid-stream is fine - whatever's currently
        # buffered just gets interpreted under whichever mode is now
        # active, nothing needs to be reset
        mode = "RAW BYTE" if self.raw_mode.get() else "framed record"
        self._log_system(f"Switched to {mode} display mode")

    # --------------------------------------------------------
    #  Live counters
    # --------------------------------------------------------
    def _refresh_live_counters(self):
        now = time.monotonic()

        # edges/sec: drop arrival times older than 1 second, count what's left
        while self.recv_times and now - self.recv_times[0] > 1.0:
            self.recv_times.pop(0)
        self.rate_var.set(f"{len(self.recv_times)} edges/sec")

        self.count_var.set(f"Records: {self.records_received}")

        if self.last_record_wall_time is None:
            self.since_var.set("Time since last edge: -")
        else:
            elapsed = now - self.last_record_wall_time
            self.since_var.set(f"Time since last edge: {elapsed:6.2f} s")

        if self.overflow:
            self.overflow_label.configure(text="OVERFLOW", fg="white", bg="#c00000")
        else:
            self.overflow_label.configure(text="ok", fg="black", bg=self._overflow_ok_bg)

    # --------------------------------------------------------
    #  Waveform
    # --------------------------------------------------------
    def _zoom_in(self):
        self.us_per_pixel = max(self.us_per_pixel / 1.5, 0.05)
        self._update_zoom_label()

    def _zoom_out(self):
        self.us_per_pixel = min(self.us_per_pixel * 1.5, 200000.0)
        self._update_zoom_label()

    def _update_zoom_label(self):
        self.zoom_var.set(f"{self.us_per_pixel:.2f} us/pixel")

    def _redraw_waveform(self):
        c = self.wave_canvas
        c.delete("wave")
        width = c.winfo_width()
        height = c.winfo_height()
        if width < 10 or height < 10 or not self.wave_records:
            return

        now_us = self.wave_records[-1][0] / TICKS_PER_US
        window_us = max(width * self.us_per_pixel, 1.0)
        t0 = now_us - window_us

        # find the last sample at/before the left edge of the window,
        # so the waveform starts at the correct level instead of at 0
        start_idx = 0
        for i, (ts, _state) in enumerate(self.wave_records):
            if ts / TICKS_PER_US <= t0:
                start_idx = i
            else:
                break

        row_h = height / NUM_CHANNELS
        pad = 6

        for ch in range(NUM_CHANNELS):
            y_hi = ch * row_h + pad          # logic 1 (top of this channel's row)
            y_lo = (ch + 1) * row_h - pad     # logic 0 (bottom of this channel's row)

            level = (self.wave_records[start_idx][1] >> ch) & 1
            y = y_hi if level else y_lo
            pts = [0.0, y]

            for ts, state in self.wave_records[start_idx:]:
                t = ts / TICKS_PER_US
                if t < t0:
                    continue
                if t > now_us:
                    break
                xt = (t - t0) / self.us_per_pixel
                newlevel = (state >> ch) & 1
                newy = y_hi if newlevel else y_lo
                pts += [xt, y]     # hold the previous level up to the transition
                pts += [xt, newy]  # then step to the new level
                y = newy

            pts += [width, y]      # hold the current level out to "now" (right edge)

            color = self.CHANNEL_COLORS[ch % len(self.CHANNEL_COLORS)]
            if len(pts) >= 4:
                c.create_line(*pts, fill=color, width=2, tags="wave")
            c.create_text(pad, ch * row_h + pad, anchor="nw", text=f"CH{ch}",
                          fill="#666666", tags="wave")
            c.create_line(0, (ch + 1) * row_h, width, (ch + 1) * row_h,
                          fill="#dddddd", tags="wave")

    # --------------------------------------------------------
    #  Clear Display / Save to CSV
    # --------------------------------------------------------
    def _clear_display(self):
        self.all_records.clear()
        self.wave_records.clear()
        self.recv_times.clear()
        self.records_received = 0
        self.prev_ts = None
        self.last_record_wall_time = None

        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")
        self.wave_canvas.delete("wave")

        self._log_system("Display cleared (FPGA state untouched - use Reset to reset the timestamp counter)")

    def _save_csv(self):
        if not self.all_records:
            messagebox.showinfo("Nothing to save", "No records received yet.")
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
                for ts, state, delta_us in self.all_records:
                    w.writerow([ts, format(state, f"0{NUM_CHANNELS}b"), f"{delta_us:.2f}"])
            self._log_system(f"Saved {len(self.all_records)} records to {path}")
        except OSError as exc:
            messagebox.showerror("Save failed", str(exc))

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
