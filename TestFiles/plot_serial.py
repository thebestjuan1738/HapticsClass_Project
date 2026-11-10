"""Live plot of any Arduino sketch that prints 'label:value,label:value,...'.

Works with TestEda, TestPulse, TestPressure, or anything new: one subplot is
made for every field it finds.
  - 'ms' (if present) is used as the time axis.
  - 'beat' (if present) is drawn as red markers on the first subplot instead
    of getting its own subplot.

Usage:  python plot_serial.py [COMPORT] [window_seconds] [--csv [FILE]]
        (port auto-detected if omitted; window defaults to 15 s)
        --csv  also log every received line to a CSV file. With no FILE it
               writes TestFiles/logs/serial_<timestamp>.csv. Columns: pc_s
               (seconds since logging started, PC clock) plus every field the
               Arduino prints (including ms and beat).
Needs:  pip install pyserial matplotlib
Close the Arduino Serial Monitor and the sensor_bridge node first - only one
program can use the port.
"""
import argparse
import csv
import os
import sys
import time
from collections import deque
from datetime import datetime

import matplotlib.pyplot as plt
import serial
import serial.tools.list_ports
from matplotlib.animation import FuncAnimation

parser = argparse.ArgumentParser(description="Live plot (and optional CSV log) of an Arduino label:value stream.")
parser.add_argument("port", nargs="?", help="serial port, e.g. COM3 (auto-detected if omitted)")
parser.add_argument("window", nargs="?", type=float, default=15.0, help="seconds shown on screen (default 15)")
parser.add_argument("--csv", nargs="?", const="auto", metavar="FILE",
                    help="log every line to a CSV (default: TestFiles/logs/serial_<timestamp>.csv)")
args = parser.parse_args()

BAUD = 115200
WINDOW_S = args.window
STARTUP_TIMEOUT_S = 20  # Uno resets on connect; EDA sketch has a 5 s baseline


def find_port():
    if args.port:
        return args.port
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "arduino" in (p.description or "").lower() or p.vid in (0x2341, 0x1A86, 0x0403):
            return p.device
    if len(ports) == 1:
        return ports[0].device
    sys.exit("Could not pick a port. Available: %s\nRun: python plot_serial.py COMx"
             % [p.device for p in ports])


def parse(line):
    try:
        return {k.strip(): float(v) for k, v in (kv.split(":") for kv in line.split(","))}
    except ValueError:
        return None  # startup text etc.


def read_sample():
    """Return one parsed line, or None if nothing complete is waiting."""
    if not ser.in_waiting:
        return None
    return parse(ser.readline().decode(errors="ignore").strip())


port = find_port()
print("Opening", port)
ser = serial.Serial(port, BAUD, timeout=0.1)

# Wait for the first data line so we know which fields exist.
print("Waiting for data (the board resets when the port opens)...")
first = None
deadline = time.time() + STARTUP_TIMEOUT_S
while first is None and time.time() < deadline:
    first = read_sample()
    time.sleep(0.01)
if first is None:
    ser.close()
    sys.exit("No data received. Check the port, baud (%d) and that the sketch is running." % BAUD)

fields = [k for k in first if k not in ("ms", "beat")]
has_beat = "beat" in first
csv_file = None
csv_writer = None
all_keys = list(first.keys())
if args.csv:
    if args.csv == "auto":
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
        os.makedirs(log_dir, exist_ok=True)
        csv_path = os.path.join(log_dir, "serial_%s.csv" % datetime.now().strftime("%Y%m%d_%H%M%S"))
    else:
        csv_path = args.csv
    csv_file = open(csv_path, "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["pc_s"] + all_keys)
    print("Logging to", csv_path)

t = deque()
vals = {k: deque() for k in fields}
beat_t = deque()
beat_y = deque()
t0_wall = time.time()

fig, axes = plt.subplots(len(fields), 1, sharex=True, figsize=(10, 2.2 * len(fields) + 1))
if len(fields) == 1:
    axes = [axes]
fig.suptitle("Serial stream - %s" % port)
lines = {}
for ax, k in zip(axes, fields):
    (lines[k],) = ax.plot([], [], lw=1)
    ax.set_ylabel(k)
    ax.grid(alpha=0.3)
axes[-1].set_xlabel("seconds")
(beat_marks,) = axes[0].plot([], [], "ro", ms=6, label="beat")
if has_beat:
    axes[0].legend(loc="upper left")


def add(sample):
    if csv_writer:
        csv_writer.writerow(["%.4f" % (time.time() - t0_wall)] + [sample.get(k, "") for k in all_keys])
    x = sample["ms"] / 1000.0 if "ms" in sample else time.time() - t0_wall
    t.append(x)
    for k in fields:
        vals[k].append(sample.get(k, float("nan")))
    if has_beat and sample.get("beat", 0) >= 1 and fields:
        beat_t.append(x)
        beat_y.append(sample.get(fields[0], 0))


add(first)


def update(_):
    while ser.in_waiting:
        s = parse(ser.readline().decode(errors="ignore").strip())
        if s and all(k in s for k in fields):
            add(s)
    if not t:
        return []
    if csv_file:
        csv_file.flush()
    cutoff = t[-1] - WINDOW_S
    while t and t[0] < cutoff:
        t.popleft()
        for k in fields:
            vals[k].popleft()
    while beat_t and beat_t[0] < cutoff:
        beat_t.popleft()
        beat_y.popleft()
    for ax, k in zip(axes, fields):
        lines[k].set_data(t, vals[k])
        ys = [v for v in vals[k] if v == v]  # drop NaN
        if ys:
            lo, hi = min(ys), max(ys)
            pad = max(1e-3, (hi - lo) * 0.1)
            ax.set_ylim(lo - pad, hi + pad)
    beat_marks.set_data(beat_t, beat_y)
    axes[0].set_xlim(t[0], max(t[-1], t[0] + 1))
    return list(lines.values()) + [beat_marks]


anim = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
try:
    plt.show()
finally:
    ser.close()
    if csv_file:
        csv_file.close()
