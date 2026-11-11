"""Background serial reader for the Arduino sensor stream (no ROS dependency).

The Arduino (arduino_firmware/SensorPassthrough) prints one line per sample as
comma-separated ``label:value`` pairs, e.g.
``ms:123456,ovr:0,eda:812,pulse:8190,pressure:35,temp:2451``.
Anything that doesn't parse that way (startup banners, a half line after a
reconnect) is ignored.

At 500 Hz that is roughly 40 KB/s, so the port is read in big chunks. pyserial's
readline() fetches one byte at a time and cannot keep up at that rate.
"""
import collections
import threading

import serial
import serial.tools.list_ports

# USB vendor IDs: Arduino, CH340 clones, FTDI
_KNOWN_VIDS = (0x2341, 0x1A86, 0x0403)

# Keys that are bookkeeping, not sensor values, when averaging samples together
_LAST_VALUE_KEYS = ('ms', 'ovr')


def find_port():
    """Return the device path of the most likely Arduino, or None."""
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if 'arduino' in (p.description or '').lower() or p.vid in _KNOWN_VIDS:
            return p.device
    if len(ports) == 1:
        return ports[0].device
    return None


def parse_line(line):
    """Parse 'a:1,b:2.5' into {'a': 1.0, 'b': 2.5}; None if not a data line."""
    line = line.strip()
    if not line:
        return None
    data = {}
    try:
        for pair in line.split(','):
            key, value = pair.split(':')
            data[key.strip()] = float(value)
    except ValueError:
        return None
    return data


def reduce_samples(samples, mode):
    """Collapse a list of parsed samples into one.

    'latest': the newest sample, untouched (raw data, nothing computed).
    'mean':   the mean of each sensor value; 'ms' and 'ovr' keep the newest value.
    """
    last = samples[-1]
    if mode != 'mean' or len(samples) == 1:
        return last
    out = {}
    for key in last:
        if key in _LAST_VALUE_KEYS:
            out[key] = last[key]
        else:
            vals = [s[key] for s in samples if key in s]
            out[key] = sum(vals) / len(vals)
    return out


class SerialReader:
    """Reads parsed samples from a serial port on a background thread.

    Samples are appended to ``self.samples`` (a deque of dicts, oldest dropped if
    it overflows). If the port can't be opened or the cable is pulled, it keeps
    retrying. ``port`` can be a Windows name (COM4) or a Linux one
    (/dev/ttyACM0); None or '' means auto-detect.
    """

    def __init__(self, port=None, baud=115200, retry_s=1.0, max_samples=5000, log=None):
        self.port = port or None
        self.baud = baud
        self.retry_s = retry_s
        self.samples = collections.deque(maxlen=max_samples)
        self.lines_received = 0     # parsed lines, for rate reporting
        self.dropped = 0            # samples lost because the deque overflowed
        self.rejected = 0           # incomplete lines thrown away
        self.connected = False
        self.port_in_use = None
        self._log = log or (lambda msg: None)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)

    def _run(self):
        while not self._stop.is_set():
            port = self.port or find_port()
            if port is None:
                self._log('No serial port found, retrying...')
                self._stop.wait(self.retry_s)
                continue
            try:
                with serial.Serial(port, self.baud, timeout=0.1) as ser:
                    self._log('Opened %s @ %d baud' % (port, self.baud))
                    self.port_in_use = port
                    self.connected = True
                    ser.reset_input_buffer()   # drop stale bytes buffered before we opened
                    self._read_loop(ser)
            except (serial.SerialException, OSError) as exc:
                self._log('Serial error on %s: %s' % (port, exc))
            finally:
                self.connected = False
            self._stop.wait(self.retry_s)

    def _read_loop(self, ser):
        buf = b''
        expected = None          # the set of fields a complete line has
        first_line = True        # the first line after opening is usually cut off
        while not self._stop.is_set():
            chunk = ser.read(max(1, ser.in_waiting))   # waits up to the timeout for data
            if not chunk:
                continue
            buf += chunk
            *lines, buf = buf.split(b'\n')
            if len(buf) > 4096:       # no newline in a long time: garbage, start over
                buf = b''
            for raw in lines:
                if first_line:
                    first_line = False
                    continue          # we joined mid-line, so this one is partial
                data = parse_line(raw.decode(errors='ignore'))
                if data is None:
                    continue
                # A line cut short can still parse (e.g. 'eda:81' for 'eda:812'),
                # so only accept lines that have every field. The fullest line
                # seen defines what a complete line is.
                keys = frozenset(data)
                if expected is None or keys > expected:
                    expected = keys
                elif keys != expected:
                    self.rejected += 1
                    continue
                self.lines_received += 1
                if len(self.samples) == self.samples.maxlen:
                    self.dropped += 1
                self.samples.append(data)
