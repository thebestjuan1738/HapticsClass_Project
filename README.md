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
