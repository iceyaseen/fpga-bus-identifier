#!/usr/bin/env python3
"""
Talks to the Tang Nano 9K over the onboard USB-serial port.

Prints whatever the FPGA sends (the periodic "Hello from FPGA" message,
and anything it echoes back), and sends whatever you type, one line at
a time, terminated with \\r\\n to match the uart_rx module on the FPGA.

Needs pyserial:  pip install pyserial
"""
import sys
import threading
import serial
import serial.tools.list_ports

BAUD = 115200
CANDIDATE_PREFIXES = ("/dev/ttyUSB", "/dev/ttyACM")


def find_port():
    ports = [p.device for p in serial.tools.list_ports.comports()
             if p.device.startswith(CANDIDATE_PREFIXES)]

    if not ports:
        print("No /dev/ttyUSB* or /dev/ttyACM* device found.", file=sys.stderr)
        print("Plug in the board, or pass the port explicitly: "
              "host.py /dev/ttyUSB0", file=sys.stderr)
        sys.exit(1)

    if len(ports) == 1:
        return ports[0]

    print("Multiple serial ports found:")
    for i, dev in enumerate(ports):
        print(f"  [{i}] {dev}")
    choice = input(f"Which one? [0-{len(ports) - 1}]: ").strip()
    try:
        return ports[int(choice)]
    except (ValueError, IndexError):
        print("Not a valid choice.", file=sys.stderr)
        sys.exit(1)


def reader_thread(ser):
    while True:
        try:
            chunk = ser.read(ser.in_waiting or 1)
        except serial.SerialException:
            print("\n[port closed]", file=sys.stderr)
            return
        if chunk:
            sys.stdout.write(chunk.decode("ascii", errors="replace"))
            sys.stdout.flush()


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()

    print(f"Opening {port} at {BAUD} baud. Ctrl+C to quit.")
    ser = serial.Serial(port, BAUD, timeout=0.1)

    t = threading.Thread(target=reader_thread, args=(ser,), daemon=True)
    t.start()

    try:
        while True:
            line = input()
            ser.write(line.encode("ascii", errors="replace") + b"\r\n")
    except (KeyboardInterrupt, EOFError):
        print("\nClosing.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
