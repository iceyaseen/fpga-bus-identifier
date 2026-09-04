# FPGA Communication Checker

A tool for figuring out how an unknown chip talks before you know anything about it. The usual situation: you have a small component with 3 or 4 pins, no datasheet, and no idea which protocol it speaks or what each pin does. Instead of guessing, you hook it up to this board and look at what actually happens on the lines.

## Status

The FPGA side is still in progress.

Working today:
- Edge capture on two probe channels, timestamped and streamed to a Python host over UART at 921600 baud.
- A framed, checksummed, self-resynchronising UART protocol between the FPGA and the host (start/stop/reset/status commands, edge records, ACK/ERROR replies).
- A from-scratch UART tx/rx pair, built and tested on the Tang Nano 9K.
- A 256-entry, block-RAM-backed capture FIFO, with high-water mark reported in the status reply.
- A PySide6 + pyqtgraph host GUI (dark instrument theme): live or offline (loaded from CSV) waveform view with pan/zoom/cursor/two draggable markers, a log-scale/zoomable pulse-width histogram, CSV export, and a "protocol hint" panel that reads the dominant pulse width and reports the closest matching clock/UART baud/1-Wire pulse - timing arithmetic only, not protocol decoding.

Not built yet:
- Protocol decoders (UART, I2C, SPI framing on top of the raw edges).
- The identification logic that would actually tell you "this looks like I2C" from the capture - the hint panel above is a timing-only nudge toward that, not the real thing.
- Switching the pull-ups on/off from software. `ctrl_a`/`ctrl_b` exist and are wired to the PCB, but they're currently hardwired off - nothing decides when to turn a pull-up on yet.

## Host software

Requires Python 3 plus:
```
pip install PySide6 pyqtgraph pyserial
```
Then run:
```
python3 host/host.py [serial-port]
```
`host/protocol.py` holds the wire-format parser (framing, checksums, resync) and `host/hint.py` the protocol-hint arithmetic; both are plain Python with no GUI dependency, and each has a standalone test script (`host/test_frame_parser.py`, `host/test_hint.py`) that can be run without any hardware attached.

## The PCB

A shield that sits on top of the Tang Nano 9K. It has two probe channels and a 4-pin socket for the device under test.
<p align="center">
<img src="hardware/images/PCBFrontWithComponents.png.png" width="45%">  <img src="hardware/images/PCBBack.png" width="45%">
<img src="hardware/images/PCBFrontWithoutComponents.png" width="65%">
<br>
Each part on the probe lines is there for a specific reason:
</p>

- **220 ohm series resistor** on each probe line. The tool is meant to connect to devices whose voltage and drive strength are unknown, so the series resistor limits current if something unexpected is being driven.
- **Schottky clamp diodes (1N5819)** to 3.3V and GND on each probe line, so an over-voltage device can't reach the FPGA pin directly. Schottky specifically, not an ordinary diode: an ordinary diode clamps around 4.0V, which is already too high for this FPGA's inputs.
- **4.7k pull-ups switched by P-channel MOSFETs.** This is the part that needs explaining: to find out whether an unknown device already has its own pull-up, you need to disconnect ours first and see what the line does on its own. A fixed pull-up would make that measurement impossible, so it's switchable instead.
- **10k gate-to-source resistors** on those MOSFETs, so they stay off during the few milliseconds after power-up while the FPGA is still loading its bitstream and its pins are floating.

## Pin assignments

| Tang Nano 9K pin | Signal    | Notes |
|---|---|---|
| 54 | `ctrl_b`   | Active low. Pulling low turns on channel B's pull-up MOSFET. |
| 55 | `ctrl_a`   | Active low. Pulling low turns on channel A's pull-up MOSFET. |
| 56 | `probe_a`  | Probe channel A. |
| 57 | `probe_b`  | Probe channel B. |

Active low because these drive P-channel MOSFETs, which turn on when their gate is pulled low.

## Using it

Plug the unknown device into the 4-pin socket. A 4-pin device fills all four positions. A 3-pin device leaves `probe_a` empty.

## Known limitations

- Only single-ended protocols are in scope. CAN and RS-485 are differential and need a transceiver, so they're not supported.
- A completely unpowered unknown chip can't be identified from the outside. This tool only works on devices that respond when probed.
- There are two probe channels, so SPI (which needs more lines) isn't reachable yet.
