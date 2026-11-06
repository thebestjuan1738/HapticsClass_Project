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

**Arduino side:** flash `src/sensor_bridge/arduino/eda_stream/eda_stream.ino`.
Each line is `ms:..,raw:..,smoothed:..,delta:..,volts:..` at 115200 baud.

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
