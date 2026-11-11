# HapticsClass_Project

ROS 2 workspace for studying the creative process with a Geomagic Touch stylus
and concurrent Arduino sensors.

Packages (`src/`): `haptic_device`, `user_interface`, `save_data`, `sensor_bridge`.
Arduino firmware: `arduino_firmware/`. Bench-test sketches and plotters: `TestFiles/`.

## sensor_bridge (Arduino -> ROS 2)

Two parts: Arduino firmware that only samples and passes data through, and a ROS
node that reads it over USB serial and publishes it.

### 1. Firmware: `arduino_firmware/SensorPassthrough/SensorPassthrough.ino`
Samples the four sensors in the same tick at **500 Hz** (R4, 14-bit) and sends
the **raw ADC counts** straight out the USB port. No smoothing, zeroing, beat
detection or unit conversion happens on the board; that is all done later on
the PC. One line per tick, newline terminated:
```
ms:123456,ovr:0,eda:812,pulse:8190,pressure:35,temp:2451
```
| Pin | Sensor | Field |
|---|---|---|
| A0 | EDA (ODAX) | `eda` |
| A1 | Pulse sensor | `pulse` |
| A2 | Pressure FSR (5V -- FSR --+-- 1k -- GND, A2 at the +) | `pressure` |
| A3 | LilyPad temperature (S tab; + to 3.3V) | `temp` |

`ms` is the board's `millis()` and `ovr` counts times the sample schedule
slipped (should stay 0). Converting counts downstream: `volts = counts * 5 /
ADC_MAX` (16383 on the R4, 1023 on the R3), temperature `= (volts - 0.5) * 100`
C, FSR ohms `= 1000 * (5 / volts - 1)`. Upload from the Arduino IDE (board:
Arduino UNO R4 Minima). An R3 also works, at 10-bit and 100 Hz.

### 2. Node: `sensor_node` (package `sensor_bridge`)
Reads the serial port and publishes one `std_msgs/Float32` topic per sensor,
named in the same style as the haptic topics:

| Topic | Type | Content |
|---|---|---|
| `eda_topic` | Float32 | raw counts |
| `pulse_topic` | Float32 | raw counts |
| `pressure_topic` | Float32 | raw counts |
| `temp_topic` | Float32 | raw counts |
| `sensor_time_topic` | Float64 | the Arduino clock, seconds since it booted |

Any other field the Arduino sends is published as `<field>_topic`, so a new
sensor only needs a new `label:value` pair in the firmware.

Parameters (set with `--ros-args -p name:=value` or as launch arguments):

| Parameter | Default | Meaning |
|---|---|---|
| `port` | `''` | Serial port: `COM4` (Windows) or `/dev/ttyACM0` (Linux/WSL). Empty = auto-detect the Arduino. |
| `baud` | `115200` | Ignored by the R4's native USB; matters for an R3. |
| `publish_rate_hz` | `100.0` | Publish rate. 100 Hz matches `haptic_loop` (see below). `0` or less = publish every sample (500 Hz). |
| `sample_mode` | `latest` | What to publish when the Arduino is faster than `publish_rate_hz`: `latest` (newest raw sample), `mean` (average since last tick), `all` (every sample). |

```
colcon build --packages-select sensor_bridge
source install/setup.bash
ros2 launch sensor_bridge sensors.launch.py port:=/dev/ttyACM0
ros2 launch sensor_bridge sensors.launch.py port:=COM4 publish_rate_hz:=100.0 sample_mode:=mean
ros2 run sensor_bridge sensor_node --ros-args -p port:=/dev/ttyACM0
ros2 topic hz /eda_topic
```
Every 5 s the node logs the Arduino line rate, the publish rate, dropped and
skipped lines, and warns if the Arduino's schedule slips or no data arrives.
The reader is built for this rate (and for 1 kHz): it reads the port in big chunks on a background
thread, discards the partial first line after (re)connecting, and ignores
incomplete lines. It reconnects by itself if the cable is unplugged.

**Dependencies:** `pyserial` (`sudo apt install python3-serial` or
`pip install pyserial`).

**WSL:** the Arduino must be passed through to WSL with `usbipd-win`, and then
the port is `/dev/ttyACM0`, not a COM name:
```
winget install usbipd                      # Windows, admin, once
usbipd list                                # find the Arduino BUSID
usbipd bind --busid <BUSID>                # once
usbipd attach --wsl --busid <BUSID>        # each time it is plugged in
```
Add yourself to the serial group once: `sudo usermod -aG dialout $USER` (then
restart WSL). Only one program can hold the port, so close the Arduino IDE
Serial Monitor and the plotters first. If ROS runs natively on Windows, use the
COM name.

### How it lines up with the database (`save_data`)
- `haptic_loop` publishes `pos_topic`, `force_topic` and `time_topic` from a
  0.01 s timer, i.e. **100 Hz**. The bridge's default `publish_rate_hz` is the
  same, so every haptic sample has a sensor sample next to it.
- `interface_loop` publishes `screen_topic` at 20 Hz (0.05 s timer).
- `database_loop` writes a row only when it has received one new message on each
  of its four topics, so it stores about **20 rows/s** (limited by
  `screen_topic`), with columns `screen, time, x, y, z, F_x, F_y, F_z`.
- The database node also subscribes to the sensor topics and stores them in the
  extra columns `eda, pulse, pressure, temp` (raw ADC counts) and `sensor_time`
  (the Arduino clock in seconds). They are **sample-and-hold**: they do not
  decide when a row is written, so the row rate is still set by the four
  original topics (about 20 rows/s). Each row just takes the latest sensor value
  received (at most about 10 ms old at the 100 Hz publish rate). If the bridge
  is not running, those columns are NULL and the rest of the row is unchanged.
  Because rows are written at 20 Hz, the database holds 1 in 5 of the 100 Hz
  sensor samples; the bag has all of them.
- **Fix made to `database_loop.py`:** `main()` called `node.close_database`
  without parentheses, so the cleanup never ran: samples still buffered at
  Ctrl+C (up to about 50 ms worth) were never written and the database was never
  closed cleanly. It is now `node.close_database()`.
- **Clocks differ:** the database `time` is `perf_counter` since `haptic_loop`
  started, while `sensor_time_topic` is the Arduino's own clock. They share no
  zero point, so line streams up by arrival time (a bag, or the database node
  holding the latest sensor value when it writes a row), not by these two
  numbers.
- A bag records everything on one clock:
  `ros2 bag record /pos_topic /force_topic /time_topic /button_topic /eda_topic /pulse_topic /pressure_topic /temp_topic`

## Bench tests (`TestFiles/`)
Sketches used while building and checking each sensor. They are not needed to
run the bridge; the firmware above is.

**Arduino side:** the sketches live in `TestFiles/` (the bridge reads any
`label:value,...` line, so no sketch is tied to the package):
- `TestFiles/TestEda/TestEda.ino` - EDA sensor on A0. Lines are
  `raw:..,smoothed:..,delta:..,volts:..` at 115200 baud.
- `TestFiles/TestPulse/TestPulse.ino` - pulse sensor on A1 (purple->A1,
  red->5V, black->GND). Lines are `ms:..,signal:..,bpm:..,ibi:..,beat:..`.
- `TestFiles/TestPressure/TestPressure.ino` - pressure sensor on A2 (below).

Check any of them live with `python TestFiles/plot_serial.py`. All use the same
`label:value` format and different pins (EDA A0, pulse A1, pressure A2).

### Temperature sensor (SparkFun LilyPad, DEV-08777)
`TestFiles/TestTemp/TestTemp.ino`, on **A3**. Pads: S -> A3, + -> 3.3V, - -> GND.
Prints `ms,raw,volts,temp_c,delta_c` at 10 Hz. From the SparkFun repo
(sparkfun/LilyPad_Temperature_Sensor): the output is analog, 0.5 V at 0 C,
0.75 V at 25 C, 10 mV per degree C, so `temp_c = (volts - 0.5) * 100`.
- SparkFun's example converts with `3.3 / 1023`. That is only right for a 3.3 V,
  10-bit board. Our sketch uses the real ADC reference (5 V) and resolution
  (14-bit on the R4), otherwise the temperatures would be wrong.
- At 10 mV/C a 10-bit read is about 0.5 C per count; 14-bit is about 0.03 C.
- The repo does not state accuracy, supply range, or operating range. Treat
  absolute values as rough and trust changes within a session. Tape it flat to
  the skin and let it settle for about a minute.
- `delta_c` is relative to the first second after reset (keep it in contact).

### Combined test: all sensors at 500 Hz (`TestAll.ino` + `plot_all.py`)
`TestFiles/TestAll/TestAll.ino` samples EDA (A0), pulse (A1), pressure (A2) and
temperature (A3) in the **same tick** at 500 Hz, 14-bit on the R4, and prints
one line per tick. It was first run at 1000 Hz (clean, 0 overruns) and is now set
to 500 Hz. The rate is one constant (`SAMPLE_RATE_HZ`); smoothing time constants, the pulse detector clock
and the schedule all derive from it. The raw `pulse` channel is logged at the
full rate (the on-board beat detector still gets a 10-bit copy, since its
thresholds are tuned in 10-bit counts), so beats can be re-detected offline.

```
python TestFiles/plot_all.py COM4 15            # dashboard: one row per sensor
python TestFiles/plot_all.py COM4 15 --csv      # also log every line to TestFiles/logs/
```
The title shows the measured lines/s and the Arduino's overruns since connect.
Result: **0 overruns at 1 kHz** (the rate it was tested at before moving to 500 Hz). Fields: `ms, ovr, eda, eda_s, eda_d, pulse, bpm,
ibi, beat, force, force_s, force_d, force_rate, temp, temp_c, temp_d, acq_us,
tot_us` (`*_d` are relative to the first 2 s after reset, so keep sensors in
place and the pressure sensor unloaded then; press reset to re-zero).

What we learned getting there:
- **Sensing was cheap, printing was not.** Reading all four sensors plus the
  pulse detector takes about 200 us per tick (`acq_us`). About 35 separate
  `Serial.print` calls per line caused the overruns. Each line is now built in a
  buffer with integer math and sent with one `Serial.write` (whole tick about
  700 us of the 1000 us budget, so headroom is about 30%). If overruns return,
  print every 2nd tick (`PRINT_EVERY_N`) or drop the derived fields, which can be
  recomputed offline.
- **`DRY_RUN`** (set to 1 in the sketch) samples at full rate and prints one
  summary line per second, which separates acquisition time from printing time.
- **A slip no longer causes a burst.** The old schedule ran missed ticks back to
  back to catch up, giving bunched timestamps and an overrun count that kept
  climbing. Now a slip counts one overrun, skips the missed ticks and resyncs, so
  it shows up as a gap in `ms`.
- **Python must read in chunks.** pyserial's `readline()` reads one byte at a
  time and froze the plot at about 200 KB/s. `plot_all.py` reads on a background
  thread in bulk and thins the points it draws. `plot_serial.py` still uses
  `readline()`, so use it only for the slow single-sensor sketches.
- **Channel crosstalk on the shared ADC.** All four sensors go through one ADC
  and multiplexer, and its sampling capacitor keeps part of the previous
  channel's voltage. With one discarded read per channel, a log showed the
  pressure channel leaking about 7.5% into temperature (read right after it): at
  rest temperature read about 2320 counts (22 C), and during hard presses it
  averaged about 3124 (about 24 C too high). EDA, pulse and the others were clean
  (correlation near 0). `SETTLE_READS` (now 3) sets the number of discarded
  reads; if channels still bleed, raise it or add 0.1 uF from the affected pin to
  GND. Check by pressing the pressure sensor with the temperature sensor steady
  and watching `temp`.
- **`ms` has 1 ms resolution**, so at 1 kHz it can repeat or skip a value by
  jitter. The board's `ms` restarts at 0 on reset; the CSV `pc_s` column gives
  PC time since logging started.
- **The first ROS bridge could not have taken this stream** (its reader used
  `readline()`, and it published every field of every line, about 18000
  messages/s at 18 fields; neither was measured, but the reader has the same
  flaw that froze the plotter). The current bridge reads in bulk, takes the
  lighter `SensorPassthrough` stream (4 raw channels, no processing) and
  publishes at a configurable rate, 100 Hz by default. See
  "sensor_bridge" at the top. The combined test is still the way to log the
  processed fields (smoothed, `*_d`, bpm) to CSV.
- **The haptic node's data is about 100 Hz.** `haptic_loop.py` publishes from a
  0.01 s ROS timer even if the device servo loop runs near 1 kHz, so the streams
  are not matched at 1 kHz on the ROS side unless that timer is sped up or the
  haptic data is recorded inside the device callback.

### Sampling rates and resolution
Only the lines the Arduino **prints** reach the PC, so the logged rate is the
print rate, not the internal sampling rate.

| Sketch | Pin | Internal sampling | Printed / logged | ADC resolution |
|---|---|---|---|---|
| `TestFiles/TestEda` | A0 | 50 Hz (smoothing) | 10 Hz | 14-bit on R4, 10-bit on R3 |
| `TestFiles/TestPressure` | A2 | 200 Hz | 50 Hz | 14-bit on R4, 10-bit on R3 |
| `TestFiles/TestTemp` | A3 | 50 Hz (8-read average) | 10 Hz | 14-bit on R4, 10-bit on R3 |
| `TestFiles/TestPulse` | A1 | 500 Hz | 50 Hz | 10-bit (kept at 10-bit, see below) |
| `TestFiles/TestAll` (all four sensors) | A0-A3 | 500 Hz on R4 (250 Hz on R3) | 500 Hz | 14-bit on R4 (pulse detector fed 10-bit) |
| `arduino_firmware/SensorPassthrough` (feeds the ROS node) | A0-A3 | 500 Hz on R4 (100 Hz on R3) | 500 Hz raw, node publishes 100 Hz | 14-bit on R4, 10-bit on R3 |

**Policy:** all raw data collection should use the **R4 at 14-bit** (counts
0-16383, step about 0.3 mV at 5 V). The R3 and the 10-bit test sketches are for
bench checks only. Record the resolution with every session, because counts
are 16x larger at 14-bit and old and new logs are not directly comparable.

Open items for later:
- **TestPulse stays 10-bit.** Its beat detector thresholds (512 center,
  530/100 starting values) are in 10-bit counts. To move to 14-bit, read at
  14-bit and shift down by 4 for the detector, or retune it, and log the 14-bit
  `signal`.
- **Pulse timing:** it samples at 500 Hz but prints only 50 Hz, so beat timing
  in the logs is limited to 20 ms. For heart-rate variability, stream every
  sample (about 15 KB/s, so use roughly 500000 baud) and detect beats in
  Python offline.
- **EDA timing:** the printed `raw` is a fresh read at print time, separate from
  the 50 Hz smoothing samples.

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
- **ADC resolution:** see "Sampling rates and resolution" above. The pressure
  sketch reads at 14-bit on the R4 (counts 0-16383) and 10-bit on the R3.
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
