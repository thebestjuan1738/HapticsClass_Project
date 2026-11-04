"""Live plot of TestEda.ino output.

Usage:  python plot_eda.py [COMPORT]     (port auto-detected if omitted)
Needs:  pip install pyserial matplotlib
Close the Arduino Serial Monitor/Plotter first - only one program can use the port.
"""
import sys
from collections import deque

import matplotlib.pyplot as plt
import serial
import serial.tools.list_ports
from matplotlib.animation import FuncAnimation

BAUD = 115200
WINDOW = 600  # points kept on screen (~60 s at 10 Hz)


def find_port():
    if len(sys.argv) > 1:
        return sys.argv[1]
    for p in serial.tools.list_ports.comports():
        if "arduino" in (p.description or "").lower() or p.vid in (0x2341, 0x1A86, 0x0403):
            return p.device
    ports = list(serial.tools.list_ports.comports())
    if len(ports) == 1:
        return ports[0].device
    sys.exit("Could not pick a port. Available: %s\nRun: python plot_eda.py COMx"
             % [p.device for p in ports])


def parse(line):
    try:
        return {k: float(v) for k, v in (kv.split(":") for kv in line.split(","))}
    except ValueError:
        return None  # startup text etc.


port = find_port()
print("Opening", port)
ser = serial.Serial(port, BAUD, timeout=0.1)

t = deque(maxlen=WINDOW)
raw = deque(maxlen=WINDOW)
smooth = deque(maxlen=WINDOW)
delta = deque(maxlen=WINDOW)
n = 0

fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
fig.suptitle("ODAX EDA - live (higher = more skin conductance)")
(l_raw,) = ax1.plot([], [], color="0.7", label="raw")
(l_sm,) = ax1.plot([], [], color="tab:blue", label="smoothed")
(l_d,) = ax2.plot([], [], color="tab:red")
ax1.set_ylabel("ADC counts (0-1023)")
ax1.legend(loc="upper left")
ax2.set_ylabel("change from baseline")
ax2.set_xlabel("sample (10 per second)")
for ax in (ax1, ax2):
    ax.grid(alpha=0.3)


def update(_):
    global n
    while ser.in_waiting:
        d = parse(ser.readline().decode(errors="ignore").strip())
        if d and {"raw", "smoothed", "delta"} <= d.keys():
            n += 1
            t.append(n)
            raw.append(d["raw"])
            smooth.append(d["smoothed"])
            delta.append(d["delta"])
    if t:
        l_raw.set_data(t, raw)
        l_sm.set_data(t, smooth)
        l_d.set_data(t, delta)
        ax1.set_xlim(t[0], max(t[-1], t[0] + 50))
        for ax, ys in ((ax1, list(raw) + list(smooth)), (ax2, delta)):
            lo, hi = min(ys), max(ys)
            pad = max(5, (hi - lo) * 0.1)
            ax.set_ylim(lo - pad, hi + pad)
    return l_raw, l_sm, l_d


anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
try:
    plt.show()
finally:
    ser.close()
