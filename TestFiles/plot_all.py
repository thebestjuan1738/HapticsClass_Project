"""Live dashboard for TestAll.ino: one row per sensor, signal on the left and
the derived / change-from-start value on the right.

    EDA          eda (raw) + eda_s (smoothed)   |  eda_d
    Pulse        pulse (+ red dots on beats)    |  bpm
    Pressure     force (raw) + force_s          |  force_rate
    Temperature  temp_c                         |  temp_d

Usage:  python plot_all.py [COMPORT] [window_seconds] [--csv [FILE]]
        (port auto-detected if omitted; window defaults to 15 s)
        --csv  also log every received line to a CSV file. With no FILE it
               writes TestFiles/logs/all_<timestamp>.csv. Columns: pc_s
               (seconds since logging started, PC clock) plus every field the
               Arduino prints.

The title shows the measured line rate and the Arduino's 'ovr' counter. If
'ovr' keeps rising, the Arduino is missing its sampling schedule.

Needs:  pip install pyserial matplotlib
Close the Arduino Serial Monitor and the sensor_bridge node first - only one
program can use the port.
"""
import argparse
import csv
import itertools
import os
import sys
import threading
import time
from collections import deque
from datetime import datetime

import matplotlib.pyplot as plt
import serial
import serial.tools.list_ports
from matplotlib.animation import FuncAnimation

parser = argparse.ArgumentParser(description="Live dashboard for the combined sensor sketch.")
parser.add_argument("port", nargs="?", help="serial port, e.g. COM4 (auto-detected if omitted)")
parser.add_argument("window", nargs="?", type=float, default=15.0, help="seconds shown on screen (default 15)")
parser.add_argument("--csv", nargs="?", const="auto", metavar="FILE",
                    help="log every line to a CSV (default: TestFiles/logs/all_<timestamp>.csv)")
args = parser.parse_args()

BAUD = 115200
WINDOW_S = args.window
STARTUP_TIMEOUT_S = 20
MAX_DRAW_POINTS = 1500   # decimate long windows so redraws stay fast

GREY = {"color": "0.7", "lw": 0.8}
BLUE = {"color": "tab:blue", "lw": 1.2}
RED = {"color": "tab:red", "lw": 1.2}
GREEN = {"color": "tab:green", "lw": 1.0}
PURPLE = {"color": "tab:purple", "lw": 1.2}
ORANGE = {"color": "tab:orange", "lw": 1.2}

# (row title, [(field, legend label, line style)] left, [...] right, left ylabel, right ylabel)
PANELS = [
    ("EDA", [("eda", "raw", GREY), ("eda_s", "smoothed", BLUE)], [("eda_d", "change", RED)],
     "counts", "counts vs start"),
    ("Pulse", [("pulse", "signal", GREEN)], [("bpm", "bpm", PURPLE)],
     "counts", "bpm"),
    ("Pressure", [("force", "raw", GREY), ("force_s", "smoothed", BLUE)], [("force_rate", "rate", RED)],
     "counts", "counts/s"),
    ("Temperature", [("temp_c", "temp", ORANGE)], [("temp_d", "change", RED)],
     "deg C", "deg C vs start"),
]


def find_port():
    if args.port:
        return args.port
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "arduino" in (p.description or "").lower() or p.vid in (0x2341, 0x1A86, 0x0403):
            return p.device
    if len(ports) == 1:
        return ports[0].device
    sys.exit("Could not pick a port. Available: %s\nRun: python plot_all.py COMx"
             % [p.device for p in ports])


def parse(line):
    try:
        return {k.strip(): float(v) for k, v in (kv.split(":") for kv in line.split(","))}
    except ValueError:
        return None  # startup text etc.


def read_sample():
    if not ser.in_waiting:
        return None
    return parse(ser.readline().decode(errors="ignore").strip())


port = find_port()
print("Opening", port)
ser = serial.Serial(port, BAUD, timeout=0.1)

print("Waiting for data (TestAll.ino spends its first 2 s zeroing the sensors)...")
first = None
deadline = time.time() + STARTUP_TIMEOUT_S
while first is None and time.time() < deadline:
    first = read_sample()
    time.sleep(0.005)
if first is None:
    ser.close()
    sys.exit("No data received. Check the port and that TestAll.ino is running on the board.")
if "ms" not in first:
    ser.close()
    sys.exit("These lines have no 'ms' field. Is TestAll.ino the sketch on the board? Got: %s" % list(first))

keys = set(first.keys())
missing = sorted({f for _, l, r, _, _ in PANELS for f, _, _ in l + r} - keys)
if missing:
    print("Note: fields not in this stream and left out of the plots:", ", ".join(missing))

csv_file = None
csv_writer = None
all_keys = list(first.keys())
if args.csv:
    if args.csv == "auto":
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
        os.makedirs(log_dir, exist_ok=True)
        csv_path = os.path.join(log_dir, "all_%s.csv" % datetime.now().strftime("%Y%m%d_%H%M%S"))
    else:
        csv_path = args.csv
    csv_file = open(csv_path, "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["pc_s"] + all_keys)
    print("Logging to", csv_path)

t0_wall = time.time()
t = deque()
data = {k: deque() for k in all_keys if k != "ms"}
beat_t = deque()
beat_y = deque()

fig, axes = plt.subplots(len(PANELS), 2, sharex=True, figsize=(13, 2.4 * len(PANELS) + 1.2))
title = fig.suptitle("")
plot_lines = {}   # field -> Line2D
for row, (name, left, right, yl, yr) in enumerate(PANELS):
    for col, (series, ylabel) in enumerate(((left, yl), (right, yr))):
        ax = axes[row][col]
        ax.grid(alpha=0.3)
        ax.set_ylabel(ylabel)
        for field, label, style in series:
            if field in keys:
                (ln,) = ax.plot([], [], label=label, **style)
                plot_lines[field] = ln
        if any(f in keys for f, _, _ in series):
            ax.legend(loc="upper left", fontsize=8)
    axes[row][0].set_title(name, loc="left", fontweight="bold", fontsize=10)
axes[-1][0].set_xlabel("seconds (Arduino clock)")
axes[-1][1].set_xlabel("seconds (Arduino clock)")
pulse_ax = axes[1][0]
(beat_marks,) = pulse_ax.plot([], [], "ro", ms=5, label="beat")
fig.tight_layout(rect=(0, 0, 1, 0.95))


def add(sample):
    if csv_writer:
        csv_writer.writerow(["%.4f" % (time.time() - t0_wall)] + [sample.get(k, "") for k in all_keys])
    x = sample["ms"] / 1000.0
    t.append(x)
    for k, dq in data.items():
        dq.append(sample.get(k, float("nan")))
    if sample.get("beat", 0) >= 1 and "pulse" in sample:
        beat_t.append(x)
        beat_y.append(sample["pulse"])


add(first)

# Reading happens on a background thread, in big chunks. pyserial's readline()
# fetches ONE BYTE at a time, which cannot keep up with ~200 KB/s at 1 kHz and
# froze the window. Here the thread grabs whatever has arrived in a single read,
# splits it into lines, parses them, and queues them for the GUI.
incoming = deque()          # deque append/popleft are thread-safe
stop_reader = threading.Event()


def reader():
    buf = b""
    while not stop_reader.is_set():
        try:
            chunk = ser.read(max(1, ser.in_waiting))   # waits up to the timeout for data
        except (serial.SerialException, OSError):
            break
        if not chunk:
            continue
        buf += chunk
        *lines, buf = buf.split(b"\n")
        for raw_line in lines:
            s = parse(raw_line.decode(errors="ignore").strip())
            if s and keys <= s.keys():
                incoming.append(s)


reader_thread = threading.Thread(target=reader, daemon=True)
reader_thread.start()


def decimate(seq, step):
    """Every step-th element of a deque, without copying the whole thing first."""
    return list(itertools.islice(seq, 0, None, step))


def update(_):
    while incoming:
        add(incoming.popleft())
    if len(t) < 2:
        return []
    if csv_file:
        csv_file.flush()

    cutoff = t[-1] - WINDOW_S
    while t and t[0] < cutoff:
        t.popleft()
        for dq in data.values():
            dq.popleft()
    while beat_t and beat_t[0] < cutoff:
        beat_t.popleft()
        beat_y.popleft()
    if len(t) < 2:
        return []

    step = max(1, len(t) // MAX_DRAW_POINTS)
    xs = decimate(t, step)
    shown = {}
    for field, ln in plot_lines.items():
        shown[field] = decimate(data[field], step)
        ln.set_data(xs, shown[field])
    beat_marks.set_data(list(beat_t), list(beat_y))

    for row, (_, left, right, _, _) in enumerate(PANELS):
        for col, series in enumerate((left, right)):
            fields = [f for f, _, _ in series if f in plot_lines]
            ys = [v for f in fields for v in shown[f] if v == v]   # decimated points only
            if col == 0 and row == 1 and beat_y:
                ys += list(beat_y)
            if not ys:
                continue
            lo, hi = min(ys), max(ys)
            pad = max((hi - lo) * 0.1, abs(hi) * 0.01, 1e-3)   # flat lines still get a sane axis
            axes[row][col].set_ylim(lo - pad, hi + pad)
    axes[0][0].set_xlim(t[0], max(t[-1], t[0] + 1))

    span = t[-1] - t[0]
    rate = (len(t) - 1) / span if span > 0 else 0
    text = "%s | %.0f lines/s" % (port, rate)
    if "ovr" in data and data["ovr"]:
        # The board counts from its own boot, so show the change since this plot connected.
        text += " | Arduino overruns since connect: %d" % int(data["ovr"][-1] - first["ovr"])
    title.set_text(text)
    return list(plot_lines.values()) + [beat_marks]


anim = FuncAnimation(fig, update, interval=200, cache_frame_data=False)
try:
    plt.show()
finally:
    stop_reader.set()
    reader_thread.join(timeout=1.0)
    ser.close()
    if csv_file:
        csv_file.close()
