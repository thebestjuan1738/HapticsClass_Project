"""Background serial reader for Arduino sensor streams (no ROS dependency).

The Arduino prints one line per sample as comma-separated ``label:value``
pairs, e.g. ``raw:312,smoothed:310.4,delta:2.1,volts:1.527``.
Anything that doesn't parse that way (startup banners, etc.) is ignored.
"""
import queue
import threading
import time

import serial
import serial.tools.list_ports

# USB vendor IDs: Arduino, CH340 clones, FTDI
_KNOWN_VIDS = (0x2341, 0x1A86, 0x0403)


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


class SerialReader:
    """Reads parsed samples from a serial port on a background thread.

    Samples are placed in ``self.samples`` (a queue of dicts). If the port
    can't be opened or the cable is pulled, it keeps retrying.
    """

    def __init__(self, port=None, baud=115200, retry_s=1.0, log=None):
        self.port = port or None
        self.baud = baud
        self.retry_s = retry_s
        self.samples = queue.Queue(maxsize=1000)
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
                with serial.Serial(port, self.baud, timeout=0.2) as ser:
                    self._log('Opened %s @ %d baud' % (port, self.baud))
                    # Opening the port resets most Arduinos; give it a moment.
                    time.sleep(2.0)
                    ser.reset_input_buffer()
                    self._read_loop(ser)
            except (serial.SerialException, OSError) as exc:
                self._log('Serial error on %s: %s' % (port, exc))
                self._stop.wait(self.retry_s)

    def _read_loop(self, ser):
        while not self._stop.is_set():
            raw = ser.readline()  # returns b'' on timeout
            if not raw:
                continue
            data = parse_line(raw.decode(errors='ignore'))
            if data is None:
                continue
            try:
                self.samples.put_nowait(data)
            except queue.Full:
                try:
                    self.samples.get_nowait()  # drop oldest
                except queue.Empty:
                    pass
                self.samples.put_nowait(data)
