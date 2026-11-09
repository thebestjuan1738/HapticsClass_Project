# HapticsClass_Project

ROS 2 workspace for studying the creative process with a Geomagic Touch stylus
and concurrent Arduino sensors.

Packages (`src/`): `haptic_device`, `user_interface`, `sensor_bridge`.

## sensor_bridge (Arduino -> ROS 2)

Reads the Arduino's USB serial stream and publishes each field as a
`std_msgs/Float32` topic named `sensor/<field>` (e.g. `sensor/raw`,
`sensor/smoothed`, `sensor/delta`, `sensor/volts`, `sensor/ms`).
Fields are discovered automatically, so a new sensor only needs new
`label:value` pairs in the Arduino sketch.

**Arduino side:** flash one of the sketches in `src/sensor_bridge/arduino/`:
- `eda_stream/eda_stream.ino` - EDA sensor on A0. Lines are
  `ms:..,raw:..,smoothed:..,delta:..,volts:..` at 115200 baud.

Pulse sensor test sketch (not yet part of the bridge): `TestFiles/TestPulse/TestPulse.ino`
(purple->A1, red->5V, black->GND). Lines are `ms:..,signal:..,bpm:..,ibi:..,beat:..`.
Check it live with `python TestFiles/plot_serial.py`, then move it into `arduino/`.
Same `label:value` format as EDA, and the pins don't collide (EDA A0, pulse A1).

### Pressure sensor (square FSR) test sketch
`TestFiles/TestPressure/TestPressure.ino`, on **A2** (EDA A0, pulse A1).
Prints `ms,raw,smoothed,volts,fsr_kohm,delta,rate` at 50 Hz.
`delta` is force relative to a no-load zero taken for 1 s after reset (keep the
sensor unloaded; press reset to re-zero), and `rate` is counts/s over 100 ms.

Wiring is a voltage divider with the sensor and resistor **in series** and A2
on the junction between them:
```
5V ── FSR ──┬── A2
            │
          1 kΩ
            │
           GND
```
Lessons from bench testing:
- **Resistor choice:** 1 kΩ works. With 10 kΩ a moderate press saturated the
  ADC (about 1000 counts) and the node picked up noise in the lab. A smaller
  resistor takes a smaller share of the 5 V, so the sensor must fall much
  further to reach the ceiling (less sensitive, wider range). Try 2.2-3.3 kΩ
  if light changes are hard to see. Set `R_FIXED_OHMS` to match so `fsr_kohm`
  is right.
- **Mount it flat:** a bent or face-down sensor is preloaded and read about 200
  counts unloaded. Lay it flat on a rigid surface, active side up, and tape only
  the edges. Press through a flat contact (foam or rubber spreads the force).
- **Wiring mistake to avoid:** a resistor from 5V straight to GND leaves A2
  floating. A floating pin reads arbitrary values (about 200, wandering), which
  is not a board fault. Shorting A2 to GND should read 0.
- **Noise:** optionally add 0.1 uF from A2 to GND and keep the leads short.
- An FSR is a rough force indicator, not a calibrated scale. Compare relative
  changes within a session.
- **R4 Minima:** shows up as a new COM port (it was COM4). Needs the "Arduino
  UNO R4 Boards" core, a USB-C **data** cable, and a double-tap of reset if an
  upload can't find the board. It does not reset when the serial port opens, so
  the startup zero only happens at power-up or reset.

**Plotting/logging:** `python TestFiles/plot_serial.py [COMx] [seconds] [--csv [FILE]]`
plots every field it finds, and `--csv` logs every line to
`TestFiles/logs/serial_<timestamp>.csv` (git-ignored).

### Pulse sensor: known problems (to fix later)
`TestPulse.ino` works at rest (about 50 bpm, beats line up with the red dots),
but the beat detector breaks down under motion or exertion:
- **Breathing/motion artifacts:** exhaling hard gave readings around 160 bpm.
  Pressure and finger movement swing the signal, the adaptive threshold gets
  pulled off, and extra crossings are counted as beats. A false short interval
  also shrinks the refractory gap, so false beats get easier (runaway).
- **Running in place:** tracking was accurate at first and got less physical
  as exertion increased.
- **Stopping after exertion:** `bpm` dropped to 0 and stayed there until the
  finger had been still for a while. The threshold only updates on a detected
  beat, so it stays set for the large exertion signal and the smaller resting
  pulses never cross it. The 2.5 s no-beat reset then relearns from scratch.
- **`ibi` looks like a staircase:** it only updates once per beat (about once a
  second) and is held between beats. This is expected, not a bug. `bpm` is a
  10-beat average, so it also moves in steps.

Ideas tried and dropped (denoising attempts did not work well):
- Baseline removal plus smoothing, a 300-1500 ms interval limit, a 35%
  tolerance around the median, and a median of 10 beats. The 35% limit
  rejected real fast changes after exercise, 5 rejects wiped the rhythm to 0,
  and the median lagged.
- Next attempt (not yet tested): a decaying envelope so the threshold also
  falls when the signal shrinks, beat-to-beat bpm with no averaging, only
  relative checks against the last interval, and a detector that keeps running
  after signal loss so bpm returns within two beats.


**Run:**
```
colcon build --packages-select sensor_bridge
source install/setup.bash
ros2 launch sensor_bridge sensors.launch.py            # auto-detect port
ros2 launch sensor_bridge sensors.launch.py port:=/dev/ttyACM0
ros2 topic echo /sensor/delta
```

**Record alongside the haptics data** (bags timestamp every message on receipt,
so streams can be aligned afterwards):
```
ros2 bag record /pos_topic /force_topic /button_topic /sensor/raw /sensor/smoothed /sensor/delta /sensor/volts
```

**Dependencies:** `pyserial` (`sudo apt install python3-serial` or
`pip install pyserial`).

**WSL note:** the Arduino must be passed through to WSL with `usbipd-win`:
```
winget install usbipd                      # Windows, admin, once
usbipd list                                # find the Arduino BUSID
usbipd bind --busid <BUSID>                # once
usbipd attach --wsl --busid <BUSID>        # each time it is plugged in
```
In Ubuntu it appears as `/dev/ttyACM0` (Uno R3) or `/dev/ttyUSB0` (clones).
Add yourself to the serial group once: `sudo usermod -aG dialout $USER`
(then restart WSL). Only one program can hold the port, so close the
Arduino IDE Serial Monitor/Plotter first.

**Timing:** `time_topic` from `haptic_device` is seconds since that node
started, not ROS time. Use bag receive timestamps (or add stamps) to align
it with `sensor/*`.
