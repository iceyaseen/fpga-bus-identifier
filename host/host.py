#!/usr/bin/env python3
# ============================================================
#  GUI logic-analyser viewer for the Tang Nano 9K edge-capture engine.
#
#  PySide6 + pyqtgraph + pyserial. See protocol.py for the wire format
#  (framing/checksums/resync) - this file is UI only and never
#  reimplements that parsing.
#
#  Layout (see _build_ui):
#    +----------+----------------------------------------------+
#    | sidebar  |  waveform (P_A / P_B, pan+zoom, cursor,       |
#    | (connect,|   two draggable markers)                     |
#    |  start/  +---------------------+------------------------+
#    |  stop/   | pulse-width         | stats + protocol hint  |
#    |  reset,  | histogram           | panel                  |
#    |  csv)    | (log-scale count,   |                        |
#    |          |  zoomable time axis)|                        |
#    +----------+---------------------+------------------------+
#    | status strip: link dot, rate, high-water, overflow dot  |
#    +-----------------------------------------------------------+
#
#  Serial I/O lives entirely on a background thread (SerialReader)
#  that only ever reads bytes and pushes them onto a queue.Queue - it
#  never touches a Qt widget. A QTimer on the UI thread drains that
#  queue at a fixed ~20 Hz rate, feeds bytes to FrameParser, and only
#  then updates widgets - so a flood of records batches into one
#  redraw instead of one redraw per record.
#
#  Needs: pip install PySide6 pyqtgraph pyserial
# ============================================================
import sys
import os
import csv
import html
import math
import time
import threading
import queue

import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

import serial
import serial.tools.list_ports

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from protocol import (
    NUM_CHANNELS, TICKS_PER_US, FIFO_DEPTH_DEFAULT,
    FrameParser, decode_edge, decode_stat, decode_ack, decode_err, ERR_CODE_NAMES,
    decode_prescan, decode_addrhit, PRESCAN_CATEGORY_NAMES,
)
from hint import hint_text, DISCLAIMER
from timefmt import format_duration_us, best_unit_for

# ---- dark instrument theme - exact palette from the spec ----
COLOR_BG = "#1A1D21"
COLOR_PANEL = "#24282E"
COLOR_GRID = "#2E333A"
COLOR_TEXT = "#D8DEE4"
COLOR_CH_A = "#7FD962"
COLOR_CH_B = "#4FC3F7"
COLOR_ACCENT = "#7FD962"
COLOR_WARN = "#F5A05C"
COLOR_ERROR = "#E06C75"
COLOR_MARKER = "#E06C75"   # not in the base 8, chosen so marker lines never
                           # visually blend with a channel trace or the grid
COLOR_MUTED_TEXT = "#7B8794"   # for de-emphasised text (the hint disclaimer) - COLOR_GRID
                                # is meant for plot gridlines and is nearly invisible as text
                                # against the similarly-dark COLOR_PANEL background

MONO_FONT_FAMILY = "DejaVu Sans Mono"

UI_HZ = 20
UI_PERIOD_MS = int(1000 / UI_HZ)     # batched redraw rate
STATS_PERIOD_MS = 500                # stats/histogram/hint recompute rate
BLINK_PERIOD_MS = 600                # "link alive" dot blink rate
STATUS_POLL_MS = 1000                # how often we send 'V' while connected
RESYNC_MIN_SETTLE_SECONDS = 0.2       # see _connect()/_tick(): minimum time before an empty
                                      # rx_queue counts as "caught up" rather than "reader
                                      # thread hasn't attempted its first read yet" - comfortably
                                      # more than that thread's own 50ms read timeout

WAVE_MAX_SAMPLES = 20000             # cap so redraw cost is bounded regardless of session length


def mono_font(size=10, bold=False):
    f = QtGui.QFont(MONO_FONT_FAMILY, size)
    f.setStyleHint(QtGui.QFont.Monospace)
    f.setBold(bold)
    return f


class TimeAxisItem(pg.AxisItem):
    """A time axis whose tick labels use the SAME ns/us/ms/s scheme as
    the cursor/marker/stats labels (format_duration_us / best_unit_for)
    instead of pyqtgraph's own SI-prefix system. That's not cosmetic:
    plotting already-converted microseconds with units="us" and letting
    pyqtgraph auto-prefix produces nonsense like "kus"/"Mus" (compounding
    an SI prefix onto a unit that's already one), and even with that
    disabled, pyqtgraph's own scaling would use different thresholds
    than our labels, so the axis and the readouts would disagree about
    what to call the same instant.

    The UNIT is picked from the values' own magnitude (so "~80s into
    the capture" stays visible even zoomed in tight), but the decimal
    PRECISION is picked from `spacing` - how far apart adjacent ticks
    actually are - not from the values themselves. Getting that wrong
    is exactly the failure mode this app needs to handle well: zoom in
    on a 5us burst that happens 80 REAL SECONDS into a capture, and
    every tick's absolute value rounds to the same "80.0 s" unless the
    precision comes from the (tiny) spacing instead.
    """
    def tickStrings(self, values, scale, spacing):
        if not values:
            return []
        ref = max((abs(v) for v in values), default=0.0)
        unit, unit_scale = best_unit_for(ref)
        spacing_in_unit = spacing / unit_scale
        decimals = 2
        if spacing_in_unit > 0:
            decimals = max(0, min(9, int(math.ceil(-math.log10(spacing_in_unit))) + 1))
        return [f"{v / unit_scale:.{decimals}f} {unit}" for v in values]


# ============================================================
#  Background thread: read raw bytes off the serial port, hand them
#  to the UI thread via a queue. Never touches a Qt widget - the UI
#  thread (MainWindow._tick) is the only thing allowed to do that.
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
#  Small "indicator light" widget: a filled circle whose colour is
#  set programmatically. Used for the link-alive dot (blinks while
#  connected) and the overflow dot (solid once overflow has latched).
# ============================================================
class Dot(QtWidgets.QLabel):
    def __init__(self, diameter=12, parent=None):
        super().__init__(parent)
        self.diameter = diameter
        self.setFixedSize(diameter, diameter)
        self._color = QtGui.QColor(COLOR_GRID)

    def set_color(self, hex_color):
        self._color = QtGui.QColor(hex_color)
        self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.setBrush(QtGui.QBrush(self._color))
        painter.setPen(QtCore.Qt.NoPen)
        painter.drawEllipse(0, 0, self.diameter, self.diameter)


# ============================================================
#  Main window
# ============================================================
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, initial_port=None):
        super().__init__()
        self.setWindowTitle("FPGA Edge Capture Viewer")
        self.resize(1400, 900)

        # ---- connection state ----
        self.ser = None
        self.reader_thread = None
        self.stop_event = None
        self.rx_queue = queue.Queue()
        self.parser = FrameParser()
        self.connected = False
        self.raw_mode = False

        # ---- capture state / unified record store (live or loaded CSV) ----
        self.prev_ts = None
        self.records = []          # (ts_ticks, state) - unbounded, source of truth for CSV save + stats
        self.wave_records = []     # capped view of self.records, for waveform redraw cost
        self.recv_times = []       # wall-clock arrival times (monotonic), for records/sec
        self.records_received = 0
        self.last_record_wall_time = None
        self.overflow_since_reset = False
        self.fifo_high_water = 0
        self.fifo_depth = FIFO_DEPTH_DEFAULT

        # ---- I2C scan state (Part B4) ----
        self.prescan_results = {}   # channel (0/1) -> category code
        self.found_addresses = []   # 7-bit addresses that ACKed, in scan order
        self.i2c_scan_stage = None  # None idle; 'prescan' waiting for E's ack; 'sweep' waiting for I's ack

        # ---- stats / histogram state ----
        self.stats_dirty = True
        self.pulse_widths = {ch: [] for ch in range(NUM_CHANNELS)}
        self.shortest_pulse_us = None
        self.longest_gap_us = None
        self.hist_channel = 0
        self.hist_view_min = None   # None = auto (full range of selected channel)
        self.hist_view_max = None
        self._hist_redraw_in_progress = False   # guards against our own setXRange() re-triggering the handler below

        # ---- resync-vs-startup bookkeeping (see _connect()/_tick()) ----
        self._resync_baseline = None

        # ---- waveform navigation state ----
        self.follow_latest = True       # oscilloscope "roll mode": view tracks the newest data
        self._wave_program_range_change = False   # guards our own setXRange() calls, same idea as the histogram's

        # ---- console panel state ----
        self._log_paused = False
        self._log_buffer = []   # (text, color) queued while paused - nothing is lost, just not shown yet

        self._build_ui()
        self._refresh_ports(preselect=initial_port)
        self._set_controls_enabled(False)

        self.ui_timer = QtCore.QTimer(self)
        self.ui_timer.timeout.connect(self._tick)
        self.ui_timer.start(UI_PERIOD_MS)

        self.stats_timer = QtCore.QTimer(self)
        self.stats_timer.timeout.connect(self._stats_tick)
        self.stats_timer.start(STATS_PERIOD_MS)

        self.blink_timer = QtCore.QTimer(self)
        self.blink_timer.timeout.connect(self._blink_tick)
        self.blink_timer.start(BLINK_PERIOD_MS)
        self._blink_on = False

        self.status_poll_timer = QtCore.QTimer(self)
        self.status_poll_timer.timeout.connect(self._cmd_status)

    # ========================================================
    #  UI construction
    # ========================================================
    def _build_ui(self):
        self.setStyleSheet(f"""
            QMainWindow, QWidget {{ background: {COLOR_BG}; color: {COLOR_TEXT}; }}
            #Sidebar, #StatsPanel {{ background: {COLOR_PANEL}; }}
            QPushButton {{
                background: {COLOR_PANEL}; color: {COLOR_TEXT};
                border: 1px solid {COLOR_GRID}; border-radius: 3px; padding: 5px;
            }}
            QPushButton:hover {{ border-color: {COLOR_ACCENT}; }}
            QPushButton:disabled {{ color: {COLOR_GRID}; }}
            QPushButton#Accent {{ color: {COLOR_ACCENT}; font-weight: bold; }}
            QPushButton:checked {{
                background: {COLOR_ACCENT}; color: {COLOR_BG};
                font-weight: bold; border-color: {COLOR_ACCENT};
            }}
            QLineEdit, QComboBox {{
                background: {COLOR_BG}; color: {COLOR_TEXT};
                border: 1px solid {COLOR_GRID}; border-radius: 3px; padding: 3px;
            }}
            QCheckBox, QLabel, QRadioButton {{ color: {COLOR_TEXT}; }}
            QGroupBox {{
                border: 1px solid {COLOR_GRID}; border-radius: 4px; margin-top: 8px;
                color: {COLOR_TEXT}; font-weight: bold;
            }}
            QGroupBox::title {{ subcontrol-origin: margin; left: 8px; padding: 0 3px; }}
            QSplitter::handle {{ background: {COLOR_GRID}; }}
            QStatusBar {{ background: {COLOR_PANEL}; }}
            QPlainTextEdit {{ background: {COLOR_BG}; color: {COLOR_TEXT}; }}
        """)

        main_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.setCentralWidget(main_splitter)

        main_splitter.addWidget(self._build_sidebar())

        right = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        right.addWidget(self._build_waveform_panel())

        bottom = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        bottom.addWidget(self._build_histogram_panel())
        bottom.addWidget(self._build_stats_hint_panel())
        bottom.setSizes([700, 500])
        right.addWidget(bottom)
        right.setSizes([500, 350])

        main_splitter.addWidget(right)
        main_splitter.setSizes([220, 1180])

        self._build_status_strip()
        self._build_log_dock()

    def _build_sidebar(self):
        panel = QtWidgets.QWidget()
        panel.setObjectName("Sidebar")
        panel.setMinimumWidth(200)
        panel.setMaximumWidth(260)
        lay = QtWidgets.QVBoxLayout(panel)
        lay.setSpacing(8)

        lay.addWidget(QtWidgets.QLabel("<b>Serial</b>"))
        self.port_combo = QtWidgets.QComboBox()
        lay.addWidget(self.port_combo)
        refresh_btn = QtWidgets.QPushButton("Refresh Ports")
        refresh_btn.clicked.connect(lambda: self._refresh_ports())
        lay.addWidget(refresh_btn)

        lay.addWidget(QtWidgets.QLabel("Baud:"))
        self.baud_edit = QtWidgets.QLineEdit("921600")
        self.baud_edit.setFont(mono_font())
        lay.addWidget(self.baud_edit)

        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.connect_btn.setObjectName("Accent")
        self.connect_btn.clicked.connect(self._toggle_connect)
        lay.addWidget(self.connect_btn)

        lay.addSpacing(10)
        lay.addWidget(QtWidgets.QLabel("<b>Capture</b>"))
        self.start_btn = QtWidgets.QPushButton("Start Capture")
        self.start_btn.clicked.connect(self._cmd_start)
        lay.addWidget(self.start_btn)
        self.stop_btn = QtWidgets.QPushButton("Stop Capture")
        self.stop_btn.clicked.connect(self._cmd_stop)
        lay.addWidget(self.stop_btn)
        self.reset_btn = QtWidgets.QPushButton("Reset")
        self.reset_btn.clicked.connect(self._cmd_reset)
        lay.addWidget(self.reset_btn)

        lay.addSpacing(10)
        lay.addWidget(QtWidgets.QLabel("<b>I2C</b>"))
        self.scan_i2c_btn = QtWidgets.QPushButton("Scan I2C")
        self.scan_i2c_btn.setObjectName("Accent")
        self.scan_i2c_btn.setToolTip("Runs the electrical pre-scan, then sweeps addresses 0x08-0x77")
        self.scan_i2c_btn.clicked.connect(self._cmd_scan_i2c)
        lay.addWidget(self.scan_i2c_btn)

        pu_row = QtWidgets.QHBoxLayout()
        self.pullup_a_check = QtWidgets.QCheckBox("Pull-up A")
        self.pullup_a_check.toggled.connect(self._on_pullup_a_toggled)
        pu_row.addWidget(self.pullup_a_check)
        self.pullup_b_check = QtWidgets.QCheckBox("Pull-up B")
        self.pullup_b_check.toggled.connect(self._on_pullup_b_toggled)
        pu_row.addWidget(self.pullup_b_check)
        lay.addLayout(pu_row)

        # which physical probe channel is SDA vs SCL isn't auto-detected
        # (see i2c_scanner section of the README for why) - if a scan
        # comes back empty, try the other mapping and scan again
        lay.addWidget(QtWidgets.QLabel("SDA/SCL mapping:"))
        self.pin_map_group = QtWidgets.QButtonGroup(panel)
        self.pin_map_normal_radio = QtWidgets.QRadioButton("Normal (A=SDA, B=SCL)")
        self.pin_map_normal_radio.setChecked(True)
        self.pin_map_swapped_radio = QtWidgets.QRadioButton("Swapped (A=SCL, B=SDA)")
        self.pin_map_group.addButton(self.pin_map_normal_radio, 0)
        self.pin_map_group.addButton(self.pin_map_swapped_radio, 1)
        self.pin_map_normal_radio.toggled.connect(self._on_pin_map_changed)
        lay.addWidget(self.pin_map_normal_radio)
        lay.addWidget(self.pin_map_swapped_radio)

        lay.addSpacing(10)
        lay.addWidget(QtWidgets.QLabel("<b>Data</b>"))
        self.load_csv_btn = QtWidgets.QPushButton("Load CSV")
        self.load_csv_btn.clicked.connect(self._load_csv)
        lay.addWidget(self.load_csv_btn)
        self.save_csv_btn = QtWidgets.QPushButton("Save CSV")
        self.save_csv_btn.clicked.connect(self._save_csv)
        lay.addWidget(self.save_csv_btn)
        clear_btn = QtWidgets.QPushButton("Clear Display")
        clear_btn.clicked.connect(self._clear_display)
        lay.addWidget(clear_btn)

        lay.addSpacing(10)
        self.console_toggle_btn = QtWidgets.QPushButton("Show Console")
        self.console_toggle_btn.setCheckable(True)
        self.console_toggle_btn.toggled.connect(self._on_console_toggle_clicked)
        lay.addWidget(self.console_toggle_btn)

        lay.addStretch(1)
        return panel

    def _build_waveform_panel(self):
        container = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(container)
        v.setContentsMargins(4, 4, 4, 4)

        v.addLayout(self._build_wave_nav_toolbar())

        self.wave_widget = pg.GraphicsLayoutWidget()
        v.addWidget(self.wave_widget, stretch=1)

        self.plot_a = self.wave_widget.addPlot(row=0, col=0)
        self.wave_widget.nextRow()
        self.plot_b = self.wave_widget.addPlot(row=1, col=0, axisItems={"bottom": TimeAxisItem(orientation="bottom")})

        for p, color, name in ((self.plot_a, COLOR_CH_A, "P_A"), (self.plot_b, COLOR_CH_B, "P_B")):
            p.showGrid(x=True, y=False, alpha=0.4)
            p.setYRange(-0.3, 1.3, padding=0)
            p.getAxis("left").setStyle(showValues=False)
            p.getAxis("left").setPen(pg.mkPen(COLOR_GRID))
            p.getAxis("bottom").setPen(pg.mkPen(COLOR_GRID))
            p.setLabel("left", name, color=color, **{"font-weight": "bold"})
            p.getViewBox().setMouseEnabled(x=True, y=False)
            p.setMenuEnabled(False)
            p.hideButtons()   # pyqtgraph's default "A" auto-range corner button - it bypasses
                               # our own view-management (follow_latest, guarded range changes)

        self.plot_a.getAxis("bottom").setStyle(showValues=False)
        self.plot_b.setLabel("bottom", "Time", color=COLOR_TEXT)
        self.plot_b.setXLink(self.plot_a)
        # this is the ONLY x-range change hook we need: plot_b is linked to
        # plot_a, so any range change on either shows up here once
        self.plot_a.getViewBox().sigXRangeChanged.connect(self._on_wave_range_changed)

        self.curve_a = self.plot_a.plot(pen=pg.mkPen(COLOR_CH_A, width=2))
        self.curve_b = self.plot_b.plot(pen=pg.mkPen(COLOR_CH_B, width=2))

        # non-interactive crosshair that follows the mouse (one line per
        # plot, kept at the same x so it visually spans both channels)
        cursor_pen = pg.mkPen(COLOR_TEXT, width=1, style=QtCore.Qt.DashLine)
        self.cursor_line_a = pg.InfiniteLine(angle=90, movable=False, pen=cursor_pen)
        self.cursor_line_b = pg.InfiniteLine(angle=90, movable=False, pen=cursor_pen)
        self.plot_a.addItem(self.cursor_line_a)
        self.plot_b.addItem(self.cursor_line_b)

        # two draggable markers for manual delta-t measurement (e.g. a bit
        # period). Each marker is really TWO InfiniteLine instances (one per
        # stacked plot) kept in sync programmatically so the line visually
        # spans both channel rows even though they're separate PlotItems.
        marker_pen = pg.mkPen(COLOR_MARKER, width=1, style=QtCore.Qt.DashLine)
        self.marker1_a = pg.InfiniteLine(angle=90, movable=True, pen=marker_pen)
        self.marker1_b = pg.InfiniteLine(angle=90, movable=True, pen=marker_pen)
        self.marker2_a = pg.InfiniteLine(angle=90, movable=True, pen=marker_pen)
        self.marker2_b = pg.InfiniteLine(angle=90, movable=True, pen=marker_pen)
        for m, pos in ((self.marker1_a, 0), (self.marker1_b, 0), (self.marker2_a, 10), (self.marker2_b, 10)):
            m.setPos(pos)
        self.plot_a.addItem(self.marker1_a)
        self.plot_b.addItem(self.marker1_b)
        self.plot_a.addItem(self.marker2_a)
        self.plot_b.addItem(self.marker2_b)
        self._link_markers(self.marker1_a, self.marker1_b)
        self._link_markers(self.marker2_a, self.marker2_b)

        self.wave_widget.scene().sigMouseMoved.connect(self._on_wave_mouse_moved)

        readout = QtWidgets.QHBoxLayout()
        self.cursor_label = QtWidgets.QLabel("cursor: -")
        self.cursor_label.setFont(mono_font())
        readout.addWidget(self.cursor_label)
        readout.addSpacing(20)
        self.marker_label = QtWidgets.QLabel()
        self.marker_label.setFont(mono_font())
        readout.addWidget(self.marker_label)
        readout.addStretch(1)
        v.addLayout(readout)
        self._update_marker_readout()   # fill in the initial text using real formatting

        # a deliberate default window width, rather than leaving it to
        # whatever pyqtgraph's own auto-fit-to-data happens to land on
        # before any real navigation action - this also disables that
        # auto-fit immediately (setXRange does that as a side effect),
        # which follow_latest's own scrolling depends on from this
        # point on: it always adjusts an existing explicit range, never
        # lets the ViewBox silently auto-fit underneath it
        self._set_wave_xrange(0.0, 1_000_000.0)   # 1 second, arbitrary position
        self._update_wave_width_field(1_000_000.0)

        return container

    def _build_wave_nav_toolbar(self):
        # explicit, reliable navigation controls - mouse wheel zoom and
        # drag pan still work (the ViewBox's own built-in handling, never
        # touched here), but these buttons are the primary way to get
        # around, per the request that drove this: mouse-only pan/zoom
        # was too fiddly to control precisely.
        bar = QtWidgets.QHBoxLayout()

        zoom_in_btn = QtWidgets.QPushButton("Zoom In")
        zoom_in_btn.clicked.connect(lambda: self._wave_zoom(1 / 2.0))
        bar.addWidget(zoom_in_btn)
        zoom_out_btn = QtWidgets.QPushButton("Zoom Out")
        zoom_out_btn.clicked.connect(lambda: self._wave_zoom(2.0))
        bar.addWidget(zoom_out_btn)

        bar.addWidget(QtWidgets.QLabel("Window:"))
        self.wave_width_edit = QtWidgets.QLineEdit()
        self.wave_width_edit.setFont(mono_font())
        self.wave_width_edit.setFixedWidth(80)
        self.wave_width_edit.editingFinished.connect(self._on_wave_width_edited)
        bar.addWidget(self.wave_width_edit)
        self.wave_width_unit_label = QtWidgets.QLabel("us")
        self.wave_width_unit_label.setFont(mono_font())
        bar.addWidget(self.wave_width_unit_label)

        step_left_btn = QtWidgets.QPushButton("◀")
        step_left_btn.setToolTip("Step left (back one window width)")
        step_left_btn.clicked.connect(lambda: self._wave_step(-1))
        bar.addWidget(step_left_btn)
        step_right_btn = QtWidgets.QPushButton("▶")
        step_right_btn.setToolTip("Step right (forward one window width)")
        step_right_btn.clicked.connect(lambda: self._wave_step(1))
        bar.addWidget(step_right_btn)

        bar.addStretch(1)

        # checkable button, not a checkbox: the spec asks for the "on"
        # state to be CLEARLY visible - QPushButton:checked (styled with
        # the accent colour in _build_ui's stylesheet) reads as "lit up"
        # far more clearly than a small checkbox tick would
        self.follow_latest_btn = QtWidgets.QPushButton("Follow Latest")
        self.follow_latest_btn.setCheckable(True)
        self.follow_latest_btn.setChecked(True)
        self.follow_latest_btn.toggled.connect(self._on_follow_latest_toggled)
        bar.addWidget(self.follow_latest_btn)

        return bar

    def _link_markers(self, line_a, line_b):
        # avoid feedback loops: block the mirrored line's own signal while
        # we programmatically move it to match the one the user dragged
        def make_handler(source, mirror):
            def handler():
                mirror.blockSignals(True)
                mirror.setPos(source.value())
                mirror.blockSignals(False)
                self._update_marker_readout()
            return handler
        line_a.sigPositionChanged.connect(make_handler(line_a, line_b))
        line_b.sigPositionChanged.connect(make_handler(line_b, line_a))

    def _update_marker_readout(self):
        a = self.marker1_a.value()
        b = self.marker2_a.value()
        delta = abs(b - a)
        self.marker_label.setText(
            f"markers: A={format_duration_us(a)}  B={format_duration_us(b)}  "
            f"(delta = {format_duration_us(delta)})"
        )

    def _on_wave_mouse_moved(self, pos):
        vb = None
        if self.plot_a.sceneBoundingRect().contains(pos):
            vb = self.plot_a.getViewBox()
        elif self.plot_b.sceneBoundingRect().contains(pos):
            vb = self.plot_b.getViewBox()
        if vb is None:
            return
        t = vb.mapSceneToView(pos).x()
        self.cursor_line_a.setPos(t)
        self.cursor_line_b.setPos(t)
        self.cursor_label.setText(f"cursor: t={format_duration_us(t)}")

    # ========================================================
    #  Waveform navigation - explicit buttons are the primary way to
    #  get around; mouse wheel zoom / drag pan (the ViewBox's own
    #  built-in handling) keep working alongside them untouched.
    # ========================================================
    def _set_wave_xrange(self, lo, hi):
        # guarded: setXRange() below fires sigXRangeChanged, which is
        # how we detect a REAL user-driven pan/zoom (to turn off Follow
        # Latest) - this flag tells that handler "this one's just us"
        self._wave_program_range_change = True
        try:
            self.plot_a.setXRange(lo, hi, padding=0)
        finally:
            self._wave_program_range_change = False

    def _wave_current_range(self):
        lo, hi = self.plot_a.getViewBox().viewRange()[0]
        return lo, hi

    def _wave_zoom(self, factor, center=None):
        # deliberately does NOT disable follow_latest: changing the
        # zoom level while still tracking the newest data is normal
        # ("adjust time/div while a scope is still in roll mode"), not
        # a request to stop following - the next auto-scroll just uses
        # the new width
        lo, hi = self._wave_current_range()
        if center is None:
            center = (lo + hi) / 2.0
        half = max((hi - lo) * factor / 2.0, 1e-6)
        self._set_wave_xrange(center - half, center + half)

    def _wave_step(self, direction):
        # unlike zoom, stepping to a specific different position IS
        # incompatible with roll mode - without this it would just get
        # overridden on the very next auto-scroll tick
        self._set_follow_latest(False)
        lo, hi = self._wave_current_range()
        width = hi - lo
        shift = direction * width
        self._set_wave_xrange(lo + shift, hi + shift)

    def _set_follow_latest(self, enabled):
        # routes through the button so its visual "lit up" state and
        # self.follow_latest always agree, however this got triggered
        self.follow_latest_btn.setChecked(enabled)

    def _on_follow_latest_toggled(self, checked):
        self.follow_latest = checked
        if checked and self.wave_records:
            # snap straight to the latest data instead of waiting for
            # the next edge to arrive before the view catches up
            latest_us = self.wave_records[-1][0] / TICKS_PER_US
            lo, hi = self._wave_current_range()
            width = hi - lo
            self._set_wave_xrange(latest_us - width, latest_us)

    def _on_wave_range_changed(self, vb, xrange):
        lo, hi = xrange
        self._update_wave_width_field(hi - lo)
        if not self._wave_program_range_change:
            # a real mouse drag/wheel-zoom, not one of our own button
            # actions or the follow-latest auto-scroll - the user is
            # navigating manually, so stop auto-following out from
            # under them
            self._set_follow_latest(False)

    def _update_wave_width_field(self, width_us):
        unit, scale = best_unit_for(width_us)
        self.wave_width_unit_label.setText(unit)
        self.wave_width_edit.setText(f"{width_us / scale:.3f}")

    def _on_wave_width_edited(self):
        try:
            value = float(self.wave_width_edit.text())
        except ValueError:
            self._update_wave_width_field(self._wave_current_range()[1] - self._wave_current_range()[0])
            return
        unit = self.wave_width_unit_label.text()
        scale = {"ns": 0.001, "us": 1.0, "ms": 1000.0, "s": 1_000_000.0}[unit]
        width_us = value * scale
        if width_us <= 0:
            return
        lo, hi = self._wave_current_range()
        center = (lo + hi) / 2.0
        self._set_wave_xrange(center - width_us / 2.0, center + width_us / 2.0)

    def _build_histogram_panel(self):
        box = QtWidgets.QGroupBox("Pulse-Width Histogram")
        v = QtWidgets.QVBoxLayout(box)

        header = QtWidgets.QHBoxLayout()
        header.addWidget(QtWidgets.QLabel("Channel:"))
        self.hist_group = QtWidgets.QButtonGroup(box)
        self.hist_radio_a = QtWidgets.QRadioButton("P_A")
        self.hist_radio_b = QtWidgets.QRadioButton("P_B")
        self.hist_radio_a.setChecked(True)
        self.hist_group.addButton(self.hist_radio_a, 0)
        self.hist_group.addButton(self.hist_radio_b, 1)
        self.hist_radio_a.toggled.connect(self._on_hist_channel_changed)
        header.addWidget(self.hist_radio_a)
        header.addWidget(self.hist_radio_b)
        header.addStretch(1)
        reset_view_btn = QtWidgets.QPushButton("Reset View")
        reset_view_btn.clicked.connect(self._hist_reset_view)
        header.addWidget(reset_view_btn)
        v.addLayout(header)

        self.hist_plot_widget = pg.PlotWidget(axisItems={"bottom": TimeAxisItem(orientation="bottom")})
        p = self.hist_plot_widget.getPlotItem()
        p.showGrid(x=True, y=True, alpha=0.4)
        p.getAxis("left").setPen(pg.mkPen(COLOR_GRID))
        p.getAxis("bottom").setPen(pg.mkPen(COLOR_GRID))
        p.setLabel("left", "count (log)", color=COLOR_TEXT)
        p.setLabel("bottom", "pulse width", color=COLOR_TEXT)
        p.getViewBox().setMouseEnabled(x=True, y=False)   # zoomable time axis; log-count axis is fixed
        p.setMenuEnabled(False)
        p.hideButtons()   # pyqtgraph's default "A" auto-range corner button - see the waveform plots
        self.hist_bars = pg.BarGraphItem(x=[0], height=[0], width=1, brush=pg.mkBrush(COLOR_CH_A))
        p.addItem(self.hist_bars)
        # user pans/zooms the time axis for free via the ViewBox's built-in
        # mouse handling; this just tells us when that happened so we can
        # re-bin the visible slice at full resolution (that's what makes
        # zooming into e.g. a histogram spike actually reveal structure,
        # rather than just visually stretching the same 60 coarse bins)
        p.getViewBox().sigXRangeChanged.connect(self._on_hist_range_changed)
        v.addWidget(self.hist_plot_widget, stretch=1)

        self.hist_info_label = QtWidgets.QLabel("(no pulses yet)")
        self.hist_info_label.setFont(mono_font(9))
        v.addWidget(self.hist_info_label)

        # guarded so this doesn't get mistaken for a user zoom (see
        # _on_hist_range_changed): without an explicit initial range,
        # pyqtgraph's own auto-fit-to-(empty)-data fires sigXRangeChanged
        # on some arbitrary tiny default range BEFORE any real histogram
        # data exists, which would otherwise permanently latch
        # hist_view_min/max to that nonsense range - every future
        # channel switch would then bin against it instead of computing
        # a real full range, until "Reset View" was clicked by hand
        self._hist_redraw_in_progress = True
        try:
            p.setXRange(0, 1)
        finally:
            self._hist_redraw_in_progress = False

        return box

    def _build_stats_hint_panel(self):
        panel = QtWidgets.QWidget()
        panel.setObjectName("StatsPanel")
        v = QtWidgets.QVBoxLayout(panel)

        stats_box = QtWidgets.QGroupBox("Statistics")
        sv = QtWidgets.QVBoxLayout(stats_box)
        self.stat_total_label = QtWidgets.QLabel("Total edges: 0")
        self.stat_shortest_label = QtWidgets.QLabel("Shortest pulse: -")
        self.stat_longest_label = QtWidgets.QLabel("Longest gap: -")
        for lbl in (self.stat_total_label, self.stat_shortest_label, self.stat_longest_label):
            lbl.setFont(mono_font())
            sv.addWidget(lbl)
        v.addWidget(stats_box)

        hint_box = QtWidgets.QGroupBox("Protocol Hint")
        hv = QtWidgets.QVBoxLayout(hint_box)
        self.hint_label = QtWidgets.QLabel("No pulses captured yet.")
        self.hint_label.setFont(mono_font(10, bold=True))
        self.hint_label.setWordWrap(True)
        hv.addWidget(self.hint_label)
        disclaimer_label = QtWidgets.QLabel(DISCLAIMER)
        disclaimer_label.setWordWrap(True)
        disclaimer_label.setStyleSheet(f"color: {COLOR_MUTED_TEXT}; font-style: italic;")
        f = disclaimer_label.font()
        f.setPointSize(8)
        disclaimer_label.setFont(f)
        hv.addWidget(disclaimer_label)
        v.addWidget(hint_box)

        i2c_box = QtWidgets.QGroupBox("I2C Scan")
        iv = QtWidgets.QVBoxLayout(i2c_box)
        self.i2c_prescan_a_label = QtWidgets.QLabel("Channel A: -")
        self.i2c_prescan_b_label = QtWidgets.QLabel("Channel B: -")
        for lbl in (self.i2c_prescan_a_label, self.i2c_prescan_b_label):
            lbl.setFont(mono_font())
            lbl.setWordWrap(True)
            iv.addWidget(lbl)
        self.i2c_addresses_label = QtWidgets.QLabel("Run a scan to look for devices.")
        self.i2c_addresses_label.setFont(mono_font(10, bold=True))
        self.i2c_addresses_label.setWordWrap(True)
        iv.addWidget(self.i2c_addresses_label)
        v.addWidget(i2c_box)

        v.addStretch(1)
        return panel

    def _refresh_i2c_panel(self):
        def describe(channel):
            if channel not in self.prescan_results:
                return "-"
            return PRESCAN_CATEGORY_NAMES.get(self.prescan_results[channel], "?")

        self.i2c_prescan_a_label.setText(f"Channel A: {describe(0)}")
        self.i2c_prescan_b_label.setText(f"Channel B: {describe(1)}")

        if self.i2c_scan_stage == "prescan":
            self.i2c_addresses_label.setText("Running electrical pre-scan...")
            self.i2c_addresses_label.setStyleSheet(f"color: {COLOR_TEXT};")
        elif self.i2c_scan_stage == "sweep":
            self.i2c_addresses_label.setText(f"Sweeping addresses... {len(self.found_addresses)} found so far")
            self.i2c_addresses_label.setStyleSheet(f"color: {COLOR_TEXT};")
        elif self.found_addresses:
            addrs = ", ".join(f"0x{a:02X}" for a in self.found_addresses)
            self.i2c_addresses_label.setText(f"Found {len(self.found_addresses)} device(s): {addrs}")
            self.i2c_addresses_label.setStyleSheet(f"color: {COLOR_ACCENT};")
        elif self.prescan_results:
            # a scan has actually completed with zero hits - say so
            # plainly rather than leaving it looking like nothing happened
            self.i2c_addresses_label.setText("No devices found.")
            self.i2c_addresses_label.setStyleSheet(f"color: {COLOR_WARN};")
        else:
            self.i2c_addresses_label.setText("Run a scan to look for devices.")
            self.i2c_addresses_label.setStyleSheet(f"color: {COLOR_TEXT};")

    def _build_status_strip(self):
        bar = self.statusBar()

        self.link_dot = Dot()
        self.link_dot.set_color(COLOR_GRID)
        bar.addWidget(self.link_dot)
        self.status_text_label = QtWidgets.QLabel("disconnected")
        bar.addWidget(self.status_text_label)

        self.rate_label = QtWidgets.QLabel("0 records/sec")
        self.rate_label.setFont(mono_font())
        bar.addPermanentWidget(self.rate_label)

        self.high_water_label = QtWidgets.QLabel(f"high water: - / {self.fifo_depth}")
        self.high_water_label.setFont(mono_font())
        bar.addPermanentWidget(self.high_water_label)

        bar.addPermanentWidget(QtWidgets.QLabel("overflow:"))
        self.overflow_dot = Dot()
        self.overflow_dot.set_color(COLOR_GRID)
        bar.addPermanentWidget(self.overflow_dot)

        self.resync_label = QtWidgets.QLabel("resyncs: 0")
        self.resync_label.setFont(mono_font())
        bar.addPermanentWidget(self.resync_label)

    def _build_log_dock(self):
        # the console: scrolling log + raw-byte diagnostic mode + pause
        # + clear, all together since they're all about inspecting the
        # wire traffic, not about the waveform/histogram. Hidden by
        # default; hiding it lets the waveform/histogram above expand
        # into the space (ordinary QDockWidget behaviour, no extra code
        # needed for that part).
        self.log_dock = QtWidgets.QDockWidget("Console", self)
        self.log_dock.setObjectName("ConsoleDock")
        self.log_dock.visibilityChanged.connect(self._on_console_visibility_changed)

        body = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(body)
        v.setContentsMargins(4, 4, 4, 4)

        header = QtWidgets.QHBoxLayout()
        self.raw_mode_check = QtWidgets.QCheckBox("Show raw bytes (diagnostic)")
        self.raw_mode_check.toggled.connect(self._on_raw_mode_toggle)
        header.addWidget(self.raw_mode_check)
        self.log_pause_check = QtWidgets.QCheckBox("Pause")
        self.log_pause_check.toggled.connect(self._on_log_pause_toggled)
        header.addWidget(self.log_pause_check)
        header.addStretch(1)
        clear_console_btn = QtWidgets.QPushButton("Clear")
        clear_console_btn.clicked.connect(self._clear_console)
        header.addWidget(clear_console_btn)
        v.addLayout(header)

        self.log_text = QtWidgets.QPlainTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(mono_font(9))
        self.log_text.setMaximumBlockCount(2000)
        v.addWidget(self.log_text)

        self.log_dock.setWidget(body)
        self.addDockWidget(QtCore.Qt.BottomDockWidgetArea, self.log_dock)
        self.log_dock.setVisible(False)

    def _on_console_toggle_clicked(self, checked):
        self.log_dock.setVisible(checked)

    def _on_console_visibility_changed(self, visible):
        # keep the sidebar button's "lit up" state in sync even if the
        # dock was closed/reopened some other way (its own titlebar X,
        # dragging it back in, etc.) rather than via our own button
        self.console_toggle_btn.setChecked(visible)
        self.console_toggle_btn.setText("Hide Console" if visible else "Show Console")

    def _on_log_pause_toggled(self, checked):
        self._log_paused = checked
        if not checked and self._log_buffer:
            for text, color in self._log_buffer:
                self._append_log_line(text, color)
            self._log_buffer.clear()

    def _clear_console(self):
        self.log_text.clear()
        self._log_buffer.clear()

    # ========================================================
    #  Port list
    # ========================================================
    def _refresh_ports(self, preselect=None):
        ports_info = list(serial.tools.list_ports.comports())
        ports = [p.device for p in ports_info]
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if preselect and preselect in ports:
            self.port_combo.setCurrentText(preselect)
            return

        # boards with an onboard FTDI-based JTAG+UART combo (e.g. the Tang
        # Nano 9K) expose BOTH as ttyUSB devices, with the JTAG interface
        # usually sorting first - auto-connecting to it instead of the
        # real UART reads back as pure noise (confirmed: this is exactly
        # what "the console only shows bytes, no text" turned out to be).
        # `description` is useless here (both interfaces report the same
        # generic "JTAG Debugger" string on this hardware), but pyserial's
        # `interface` field (from the USB interface descriptor, Linux
        # only) reliably differs: the JTAG interface names itself, the
        # UART interface leaves it blank. Falls back to the old
        # first-match behaviour wherever `interface` isn't populated.
        candidates = [p for p in ports_info if "ttyUSB" in p.device or "ttyACM" in p.device]
        non_jtag = [p for p in candidates if not (p.interface and "jtag" in p.interface.lower())]
        pick = non_jtag[0] if non_jtag else (candidates[0] if candidates else None)
        if pick:
            self.port_combo.setCurrentText(pick.device)

    # ========================================================
    #  Connect / disconnect
    # ========================================================
    def _toggle_connect(self):
        if self.connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_combo.currentText().strip()
        if not port:
            QtWidgets.QMessageBox.critical(self, "No port selected", "Choose a serial port first.")
            return
        try:
            baud = int(self.baud_edit.text())
        except ValueError:
            QtWidgets.QMessageBox.critical(self, "Bad baud rate", "Baud rate must be an integer.")
            return

        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
        except serial.SerialException as exc:
            QtWidgets.QMessageBox.critical(self, "Connection failed", str(exc))
            return

        self.stop_event = threading.Event()
        self.reader_thread = SerialReader(self.ser, self.rx_queue, self.stop_event)
        self.reader_thread.start()

        self.connected = True
        self.parser = FrameParser()
        self.prev_ts = None
        self.connect_btn.setText("Disconnect")
        self._set_controls_enabled(True)
        self.status_text_label.setText(f"connected ({port} @ {baud})")
        self._log_system(f"Connected to {port} @ {baud} baud")
        self.status_poll_timer.start(STATUS_POLL_MS)

        # a fresh connect landing mid-frame (the FPGA doesn't wait for a
        # listener - it may already be transmitting) causes a small,
        # ONE-TIME batch of resyncs that isn't a real problem: it's
        # bounded (self-heals within whatever was already in flight)
        # and never recurs. _tick() captures _resync_baseline once
        # BOTH the queue has actually drained empty AND a short minimum
        # time has passed since connecting - see _tick() for why either
        # signal alone is unreliable (queue-empty alone can trigger on
        # the very first tick, before the reader thread has attempted
        # its first read at all; a fixed time alone can fire before a
        # slow event loop has finished reflecting a real burst that's
        # still queued).
        self._resync_baseline = None
        self._connect_monotonic_time = time.monotonic()

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
        self.status_poll_timer.stop()
        self.connect_btn.setText("Connect")
        self._set_controls_enabled(False)
        self.link_dot.set_color(COLOR_GRID)
        self.status_text_label.setText("disconnected" if reason is None else f"disconnected: {reason}")
        self._log_system(f"Disconnected: {reason}" if reason else "Disconnected")

    def _set_controls_enabled(self, connected):
        for b in (self.start_btn, self.stop_btn, self.reset_btn, self.scan_i2c_btn,
                  self.pullup_a_check, self.pullup_b_check,
                  self.pin_map_normal_radio, self.pin_map_swapped_radio):
            b.setEnabled(connected)

    # ========================================================
    #  Commands out to the FPGA (unchanged single-byte commands)
    # ========================================================
    def _send_byte(self, b):
        if not self.connected or self.ser is None:
            return
        try:
            self.ser.write(bytes([b]))
        except (serial.SerialException, OSError) as exc:
            self._disconnect(reason=str(exc))

    def _cmd_start(self):
        self._send_byte(ord("S"))

    def _cmd_stop(self):
        self._send_byte(ord("X"))

    def _cmd_reset(self):
        self._send_byte(ord("R"))
        self.prev_ts = None
        self.overflow_since_reset = False
        self.overflow_dot.set_color(COLOR_GRID)
        self._log_system("Sent RESET ('R')")

    def _cmd_status(self):
        self._send_byte(ord("V"))

    def _on_pullup_a_toggled(self, checked):
        self._send_byte(ord("A") if checked else ord("a"))

    def _on_pullup_b_toggled(self, checked):
        self._send_byte(ord("B") if checked else ord("b"))

    def _on_pin_map_changed(self, checked):
        # fires from the "Normal" radio's toggled signal: checked=True
        # means Normal was just selected, checked=False means Swapped
        # was (radio-button exclusivity guarantees exactly one of the
        # two toggled signals fires per click)
        self._send_byte(ord("N") if checked else ord("W"))

    def _cmd_scan_i2c(self):
        """The Part 6 'Scan I2C' button: runs the pre-scan, then
        (automatically, once the pre-scan's ACK confirms it's done -
        see _handle_frame's ACK case) the address sweep. Also makes
        sure capture is running first, so the scan's own generated
        waveform is visible - a good verification the master is
        generating correct signals, and free once capture is already on."""
        if not self.connected:
            return
        self.prescan_results = {}
        self.found_addresses = []
        self.i2c_scan_stage = "prescan"   # 'prescan' -> waiting for E's ack; 'sweep' -> waiting for I's ack
        self._refresh_i2c_panel()
        self._send_byte(ord("S"))
        self._send_byte(ord("E"))
        self._log_system("I2C scan: running electrical pre-scan...")

    # ========================================================
    #  Recurring UI-thread tick: drain the queue, batch the redraw
    # ========================================================
    def _tick(self):
        try:
            drained = 0
            chunks = []
            queue_emptied = False   # did we actually hit queue.Empty (caught up), not just the drain cap?
            while drained < 4096:
                try:
                    kind, payload = self.rx_queue.get_nowait()
                except queue.Empty:
                    queue_emptied = True
                    break
                drained += 1
                if kind == "error":
                    self._disconnect(reason=payload)
                    break
                elif kind == "data":
                    chunks.append(payload)

            new_edges = False
            if chunks:
                data = b"".join(chunks)
                # raw-byte logging is an ADDITION to normal parsing, not
                # a replacement for it - it used to skip parser.feed()
                # entirely while enabled, which silently stopped the
                # waveform/stats/hint from updating at all (looked like
                # the app had broken, not like a diagnostic view)
                if self.raw_mode:
                    self._log_raw_bytes(data)
                for kind, payload in self.parser.feed(data):
                    if self._handle_frame(kind, payload):
                        new_edges = True

            if (self._resync_baseline is None and queue_emptied and self.connected
                    and time.monotonic() - self._connect_monotonic_time >= RESYNC_MIN_SETTLE_SECONDS):
                # both signals agreed: the queue is drained AND enough
                # time has passed for the reader thread to have actually
                # attempted reading (not just found nothing queued yet
                # because it hasn't run once) - so anything counted up
                # to now was genuinely already in flight at connect time
                # (the benign mid-frame-landing case, if it happened at
                # all); freeze it as the baseline so any LATER resync is
                # recognisable as new, not more of that same backlog
                self._resync_baseline = self.parser.resync_count

            self._update_resync_label()
            self._refresh_live_counters()
            if new_edges:
                self._redraw_waveform()
        except Exception as exc:
            self._log(f"[internal error in UI update: {exc}]", COLOR_ERROR)

    def _stats_tick(self):
        if self.stats_dirty:
            self._recompute_stats()
            self._refresh_stats_labels()
            self._redraw_histogram()
            self._refresh_hint()
            self.stats_dirty = False

    def _blink_tick(self):
        if not self.connected:
            return
        self._blink_on = not self._blink_on
        self.link_dot.set_color(COLOR_ACCENT if self._blink_on else COLOR_GRID)

    def _log_raw_bytes(self, data):
        hexline = " ".join(f"{b:02X}" for b in data)
        self._log(f"[RAW] {hexline}", "#A0785A")

    # ========================================================
    #  Frame dispatch
    # ========================================================
    def _handle_frame(self, kind, payload):
        """Returns True if this frame added an edge record (so the
        caller knows a waveform redraw is worth doing this tick)."""
        if kind == "EDGE":
            ts, state = decode_edge(payload)
            self._add_record(ts, state)
            return True
        elif kind == "STAT":
            overflow, hw, depth = decode_stat(payload)
            if overflow:
                self.overflow_since_reset = True
                self.overflow_dot.set_color(COLOR_WARN)
            self.fifo_high_water = hw
            self.fifo_depth = depth
            self.high_water_label.setText(f"high water: {hw} / {depth}")
            self._log(f"[STATUS] overflow_since_reset={'YES' if self.overflow_since_reset else 'no'}  "
                      f"high_water={hw}/{depth}", "#4FA0D8")
        elif kind == "ACK":
            cmd = decode_ack(payload)
            try:
                cmd_ch = chr(cmd)
            except ValueError:
                cmd_ch = "?"
            self._log(f"[ACK] {cmd_ch}", COLOR_ACCENT)
            # 'E'/'I' acks are DEFERRED by the FPGA until the operation
            # they kicked off actually finishes (see top.v) - that's
            # what makes this ack the right moment to chain into the
            # address sweep, or to declare the whole scan done
            if cmd_ch == "E" and self.i2c_scan_stage == "prescan":
                self.i2c_scan_stage = "sweep"
                self._log_system("I2C scan: pre-scan done, sweeping addresses 0x08-0x77...")
                self._send_byte(ord("I"))
            elif cmd_ch == "I" and self.i2c_scan_stage == "sweep":
                self.i2c_scan_stage = None
                if self.found_addresses:
                    addrs = ", ".join(f"0x{a:02X}" for a in self.found_addresses)
                    self._log_system(f"I2C scan: done - found {len(self.found_addresses)} device(s): {addrs}")
                else:
                    self._log_system("I2C scan: done - no devices found")
            self._refresh_i2c_panel()
        elif kind == "ERR":
            code, bad = decode_err(payload)
            name = ERR_CODE_NAMES.get(code, f"code {code}")
            self._log(f"[ERROR] {name}: offending byte 0x{bad:02X}", COLOR_ERROR)
        elif kind == "PRESCAN":
            channel, category = decode_prescan(payload)
            self.prescan_results[channel] = category
            label = "A" if channel == 0 else "B"
            self._log(f"[PRESCAN] channel {label}: {PRESCAN_CATEGORY_NAMES.get(category, category)}", "#4FA0D8")
            self._refresh_i2c_panel()
        elif kind == "ADDRHIT":
            addr = decode_addrhit(payload)
            self.found_addresses.append(addr)
            self._log(f"[I2C] found device at 0x{addr:02X}", COLOR_ACCENT)
            self._refresh_i2c_panel()
        return False

    def _add_record(self, ts, state):
        now = time.monotonic()
        self.records.append((ts, state))
        self.wave_records.append((ts, state))
        if len(self.wave_records) > WAVE_MAX_SAMPLES * 2:
            self.wave_records = self.wave_records[-WAVE_MAX_SAMPLES:]

        self.recv_times.append(now)
        self.last_record_wall_time = now
        self.records_received += 1
        self.stats_dirty = True
        self.prev_ts = ts

    # ========================================================
    #  Log widget
    # ========================================================
    def _log(self, text, color=COLOR_TEXT):
        if self._log_paused:
            # nothing lost, just held back until Pause is unchecked (see
            # _on_log_pause_toggled) - that's the whole point of pausing:
            # let the user read something without it scrolling away
            self._log_buffer.append((text, color))
            return
        self._append_log_line(text, color)

    def _append_log_line(self, text, color):
        self.log_text.appendHtml(f'<span style="color:{color}">{html.escape(text)}</span>')

    def _log_system(self, text):
        self._log(text, COLOR_MUTED_TEXT)

    def _on_raw_mode_toggle(self, checked):
        self.raw_mode = checked
        self._log_system(f"Switched to {'RAW BYTE' if checked else 'framed'} display mode")

    def _update_resync_label(self):
        count = self.parser.resync_count
        if self._resync_baseline is None:
            # haven't caught up to an empty queue yet since connecting -
            # anything counted so far might still just be backlog from
            # whatever was already in flight at connect time, so don't
            # pass judgment yet (see _tick(), which sets the baseline)
            label = f"resyncs: {count} (settling)" if self.connected else f"resyncs: {count}"
            self.resync_label.setText(label)
            self.resync_label.setStyleSheet("")
            return

        extra = count - self._resync_baseline
        if extra > 0:
            # genuinely new resyncs, counted AFTER we'd already caught
            # up and were listening cleanly - unlike the startup batch,
            # this has no benign explanation, so it's worth flagging
            self.resync_label.setText(f"resyncs: {count} (+{extra} since connect settled)")
            self.resync_label.setStyleSheet(f"color: {COLOR_WARN};")
        else:
            self.resync_label.setText(f"resyncs: {count}")
            self.resync_label.setStyleSheet("")

    # ========================================================
    #  Live counters
    # ========================================================
    def _refresh_live_counters(self):
        now = time.monotonic()
        while self.recv_times and now - self.recv_times[0] > 1.0:
            self.recv_times.pop(0)
        self.rate_label.setText(f"{len(self.recv_times):5d} records/sec")

    # ========================================================
    #  Statistics
    # ========================================================
    def _recompute_stats(self):
        records = self.records
        self.stat_total_label.setText(f"Total edges: {len(records)}")

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
            self.stat_shortest_label.setText("Shortest pulse: -")
        else:
            self.stat_shortest_label.setText(f"Shortest pulse: {format_duration_us(self.shortest_pulse_us)}")
        if self.longest_gap_us is None:
            self.stat_longest_label.setText("Longest gap: -")
        else:
            self.stat_longest_label.setText(f"Longest gap: {format_duration_us(self.longest_gap_us)}")

    def _refresh_hint(self):
        widths = self.pulse_widths.get(self.hist_channel, [])
        text = hint_text(widths)
        self.hint_label.setText(text)
        if text.startswith("Consistent"):
            color = COLOR_ACCENT
        elif "Roughly" in text:
            color = COLOR_WARN
        elif "noise" in text or "Doesn't" in text:
            color = COLOR_ERROR
        else:
            color = COLOR_TEXT
        self.hint_label.setStyleSheet(f"color: {color};")

    # ========================================================
    #  Histogram: log-scale count axis, zoomable time axis (pan/zoom
    #  is free via pyqtgraph's ViewBox - only the log transform and
    #  channel/view-range bookkeeping is custom here)
    # ========================================================
    def _on_hist_channel_changed(self, checked):
        if checked:
            self.hist_channel = 0
        else:
            self.hist_channel = 1
        self._redraw_histogram()
        self._refresh_hint()

    def _hist_full_range(self, widths):
        lo, hi = min(widths), max(widths)
        if hi <= lo:
            hi = lo + 1.0
        return lo, hi

    def _hist_current_range(self, widths):
        if self.hist_view_min is None or self.hist_view_max is None:
            return self._hist_full_range(widths)
        return self.hist_view_min, self.hist_view_max

    def _hist_reset_view(self):
        self.hist_view_min = None
        self.hist_view_max = None
        self._redraw_histogram()   # recomputes the full range and sets it below

    def _on_hist_range_changed(self, vb, xrange):
        if self._hist_redraw_in_progress:
            return   # our own setXRange() call below, not a user pan/zoom
        self.hist_view_min, self.hist_view_max = xrange
        self._redraw_histogram()

    def _redraw_histogram(self):
        widths = self.pulse_widths.get(self.hist_channel, [])
        color = COLOR_CH_A if self.hist_channel == 0 else COLOR_CH_B
        self.hist_bars.setOpts(brush=pg.mkBrush(color))

        if not widths:
            self.hist_bars.setOpts(x=[0], height=[0], width=1)
            self.hist_info_label.setText("(no pulses yet on this channel)")
            return

        # view range: full data range unless the user has zoomed (in which
        # case hist_view_min/max holds what they zoomed to, so new data
        # arriving doesn't yank their view back to "full" on every redraw)
        lo, hi = self._hist_current_range(widths)

        nbins = 60
        bin_w = (hi - lo) / nbins
        counts = [0] * nbins
        in_view = 0
        for w in widths:
            if w < lo or w > hi:
                continue
            idx = int((w - lo) / bin_w) if bin_w > 0 else 0
            if idx >= nbins:
                idx = nbins - 1
            elif idx < 0:
                idx = 0
            counts[idx] += 1
            in_view += 1

        max_count = max(counts) if counts else 0
        if max_count == 0:
            self.hist_bars.setOpts(x=[0], height=[0], width=1)
            self.hist_info_label.setText(
                f"(no pulses in {format_duration_us(lo)}-{format_duration_us(hi)} - reset view to zoom out)")
            return

        # log scale on the count axis: bar height IS log10(count+1), so a
        # rare sharp spike (e.g. a 100 kHz I2C clock) stays visible next to
        # a much taller but structureless smear instead of being flattened
        # to a sliver by a linear axis. The left-axis ticks below translate
        # these log-space heights back into real, human counts.
        centers = [lo + (i + 0.5) * bin_w for i, cnt in enumerate(counts) if cnt != 0]
        heights = [math.log10(cnt + 1) for cnt in counts if cnt != 0]

        self.hist_bars.setOpts(x=centers, height=heights, width=bin_w * 0.9)

        axis = self.hist_plot_widget.getPlotItem().getAxis("left")
        ticks = []
        mark = 1
        while mark <= max_count:
            ticks.append((math.log10(mark + 1), str(mark)))
            mark *= 10
        axis.setTicks([ticks])

        if self.hist_view_min is None:
            # guarded: this triggers sigXRangeChanged, which would otherwise
            # be indistinguishable from the user actually dragging/zooming
            self._hist_redraw_in_progress = True
            try:
                self.hist_plot_widget.getPlotItem().setXRange(lo, hi, padding=0.02)
            finally:
                self._hist_redraw_in_progress = False

        self.hist_info_label.setText(
            f"{in_view}/{len(widths)} pulses in view, {format_duration_us(lo)}-{format_duration_us(hi)}")

    # ========================================================
    #  Clear Display / Save / Load
    # ========================================================
    def _clear_display(self):
        self.records.clear()
        self.wave_records.clear()
        self.recv_times.clear()
        self.records_received = 0
        self.prev_ts = None
        self.last_record_wall_time = None
        self.stats_dirty = True
        self.log_text.clear()
        self._log_buffer.clear()
        self.curve_a.setData([], [])
        self.curve_b.setData([], [])
        self.prescan_results = {}
        self.found_addresses = []
        self.i2c_scan_stage = None
        self._refresh_i2c_panel()
        self._log_system("Display cleared (FPGA state untouched - use Reset to reset the timestamp counter)")

    def _save_csv(self):
        if not self.records:
            QtWidgets.QMessageBox.information(self, "Nothing to save", "No records to save.")
            return
        default_name = f"capture_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save CSV", default_name, "CSV files (*.csv)")
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
            QtWidgets.QMessageBox.critical(self, "Save failed", str(exc))

    def _load_csv(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Load CSV", "", "CSV files (*.csv)")
        if not path:
            return
        try:
            loaded = []
            with open(path, newline="") as f:
                r = csv.reader(f)
                next(r, None)
                for row in r:
                    if len(row) < 2:
                        continue
                    ts = int(row[0])
                    state = int(row[1], 2)
                    loaded.append((ts, state))
        except (OSError, ValueError) as exc:
            QtWidgets.QMessageBox.critical(self, "Load failed", str(exc))
            return

        if not loaded:
            QtWidgets.QMessageBox.information(self, "Empty file", "No records found in that CSV.")
            return

        self._clear_display()
        self.records = loaded
        self.wave_records = loaded[-WAVE_MAX_SAMPLES:]
        self.records_received = len(loaded)
        self.stats_dirty = True
        self._redraw_waveform()
        self._log_system(f"Loaded {len(loaded)} records from {path} (offline view - not connected to live data)")

    # ========================================================
    #  Waveform redraw - vectorised with numpy (a pyqtgraph dependency
    #  already, not a new one) rather than a per-point Python loop, so
    #  a flood of records doesn't turn this into the bottleneck pyqtgraph
    #  was chosen to avoid.
    # ========================================================
    def _redraw_waveform(self):
        if not self.wave_records:
            self.curve_a.setData([], [])
            self.curve_b.setData([], [])
            return

        ts = np.array([r[0] for r in self.wave_records], dtype=np.float64)
        states = np.array([r[1] for r in self.wave_records], dtype=np.uint8)
        t_us = ts / TICKS_PER_US

        for ch, curve in ((0, self.curve_a), (1, self.curve_b)):
            levels = (states >> ch) & 1
            if len(levels) == 1:
                curve.setData(t_us, levels.astype(np.float64))
                continue
            # step waveform: value holds at levels[i-1] right up to t[i],
            # then jumps to levels[i] at that same timestamp - i.e. each
            # (t[i], levels[i-1]) then (t[i], levels[i]) pair, for i=1..n-1,
            # preceded by the very first (t[0], levels[0]).
            xs = np.concatenate(([t_us[0]], np.repeat(t_us[1:], 2)))
            pair = np.column_stack((levels[:-1], levels[1:])).ravel()
            ys = np.concatenate(([levels[0]], pair)).astype(np.float64)
            curve.setData(xs, ys)

        if self.follow_latest:
            # oscilloscope "roll mode": keep the newest data at the right
            # edge, same window width as whatever the user last set
            latest_us = t_us[-1]
            lo, hi = self._wave_current_range()
            width = hi - lo
            self._set_wave_xrange(latest_us - width, latest_us)

    # ========================================================
    def closeEvent(self, event):
        self._disconnect()
        event.accept()


def main():
    pg.setConfigOption("background", COLOR_BG)
    pg.setConfigOption("foreground", COLOR_TEXT)
    pg.setConfigOptions(antialias=True)

    initial_port = sys.argv[1] if len(sys.argv) > 1 else None
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow(initial_port=initial_port)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
