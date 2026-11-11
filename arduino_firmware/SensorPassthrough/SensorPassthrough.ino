/*
  SensorPassthrough.ino - raw sensor sampler for the ROS sensor_bridge node
  Board: Arduino Uno R4 Minima (the R3 also builds, at 10-bit and a lower rate)

  Samples the four sensors and passes the RAW ADC counts straight out over USB
  serial. There is no smoothing, zeroing, beat detection or unit conversion on
  the board: all of that happens later, on the PC, where it can be redone.

  Sensors and pins (all grounds common; wiring details are in README.md):
    A0  EDA (ODAX) VOUT
    A1  Pulse sensor signal (purple wire; red -> 5V, black -> GND)
    A2  Pressure FSR divider (5V -- FSR --+-- 1k -- GND, A2 at the +)
    A3  LilyPad temperature sensor S tab (+ -> 3.3V, - -> GND)

  Sampling: all four are read in the SAME tick at SAMPLE_RATE_HZ (500 Hz on the
  R4; the ROS node publishes 100 Hz, matching the haptics topics). Within a tick the reads are one after another,
  well under 1 ms apart. Several discarded reads (SETTLE_READS) precede each real read so
  the shared ADC settles on the new channel (matters for the high-impedance EDA and
  pressure nodes).

  Resolution: 14-bit on the R4 (counts 0-16383), 10-bit on the R3 (0-1023).
  The ADC reference is 5 V. The bridge/analysis side needs to know which was
  used when converting counts to volts.

  Serial output: one line per tick, label:value pairs, newline terminated:
    ms:123456,ovr:0,eda:812,pulse:8190,pressure:35,temp:2451
      ms        millis() when the line was sent (the board's own clock; it
                restarts at 0 whenever the board resets)
      ovr       how many times the sample schedule slipped by a whole tick or
                more since boot (should stay 0)
      eda       raw counts, A0
      pulse     raw counts, A1
      pressure  raw counts, A2
      temp      raw counts, A3
  Each line is built in a buffer and sent with ONE Serial.write. Many small
  Serial.print calls per line were what caused overruns in the earlier
  combined test.

  If the schedule ever slips (for example the host stops reading and a USB write
  blocks) the missed ticks are skipped and the schedule restarts, so a stall
  shows up as a gap in 'ms' and one count in 'ovr', not a burst of catch-up
  lines.

  Conversions done downstream (not here):
    volts       = counts * 5.0 / ADC_MAX            (ADC_MAX = 16383 or 1023)
    temperature = (volts - 0.5) * 100  degrees C     (MCP9700-type LilyPad)
    FSR ohms    = R_fixed * (5.0 / volts - 1)        (R_fixed = 1 kOhm)

  Only one program can hold the serial port (close the Serial Monitor and the
  plotters before running the ROS node).
*/

const uint8_t EDA_PIN      = A0;
const uint8_t PULSE_PIN    = A1;
const uint8_t PRESSURE_PIN = A2;
const uint8_t TEMP_PIN     = A3;

// ADC resolution and sample rate for whichever board this is built for.
#if defined(ARDUINO_UNOR4_MINIMA) || defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_ARCH_RENESAS_UNO)
  #define ADC_BITS 14
  const uint16_t SAMPLE_RATE_HZ = 500;    // R4: 500 Hz, 14-bit (the ROS node publishes 100 Hz)
#else
  #define ADC_BITS 10
  const uint16_t SAMPLE_RATE_HZ = 100;    // R3: slow ADC and 115200 baud serial
#endif
const uint32_t SAMPLE_PERIOD_US = 1000000UL / SAMPLE_RATE_HZ;

uint32_t nextSampleUs = 0;
uint32_t overruns     = 0;

// ---- Line building: integer math into a buffer, then one write ----
char     lineBuf[128];
uint16_t lineLen = 0;

void addChar(char c) {
  if (lineLen < sizeof(lineBuf) - 1) lineBuf[lineLen++] = c;
}

void addStr(const char* str) {
  while (*str) addChar(*str++);
}

void addInt(long v) {
  char tmp[12];
  uint8_t n = 0;
  bool neg = v < 0;
  unsigned long u = neg ? (unsigned long)(-v) : (unsigned long)v;
  do { tmp[n++] = '0' + (u % 10); u /= 10; } while (u > 0);
  if (neg) addChar('-');
  while (n) addChar(tmp[--n]);
}

// The four channels share ONE ADC through a multiplexer. When it switches channel
// its sampling capacitor still holds part of the previous channel's voltage, so
// the first read after a switch is contaminated. Bench data showed the pressure
// channel leaking about 7.5% into temperature (the channel read right after it)
// with a single discarded read. SETTLE_READS throwaway reads let it settle first.
// If channels still bleed into each other, raise this (or add 0.1 uF from the
// affected pin to GND).
const uint8_t SETTLE_READS = 3;

int readAdc(uint8_t pin) {
  for (uint8_t i = 0; i < SETTLE_READS; i++) analogRead(pin);   // discarded
  return analogRead(pin);
}

void setup() {
  Serial.begin(115200);   // ignored on the R4 (native USB); sets the speed on an R3
  pinMode(EDA_PIN, INPUT);
  pinMode(PULSE_PIN, INPUT);
  pinMode(PRESSURE_PIN, INPUT);
  pinMode(TEMP_PIN, INPUT);
#if ADC_BITS > 10
  analogReadResolution(ADC_BITS);
#endif
  nextSampleUs = micros() + SAMPLE_PERIOD_US;
}

void loop() {
  // Fixed-rate schedule (signed subtraction handles micros() rollover)
  int32_t late = (int32_t)(micros() - nextSampleUs);
  if (late < 0) return;

  if (late >= (int32_t)SAMPLE_PERIOD_US) {
    overruns++;                    // slipped a whole tick or more
    nextSampleUs = micros();       // skip the missed ticks instead of bursting
  }
  nextSampleUs += SAMPLE_PERIOD_US;

  int eda      = readAdc(EDA_PIN);
  int pulse    = readAdc(PULSE_PIN);
  int pressure = readAdc(PRESSURE_PIN);
  int temp     = readAdc(TEMP_PIN);

  lineLen = 0;
  addStr("ms:");        addInt((long)millis());
  addStr(",ovr:");      addInt((long)overruns);
  addStr(",eda:");      addInt(eda);
  addStr(",pulse:");    addInt(pulse);
  addStr(",pressure:"); addInt(pressure);
  addStr(",temp:");     addInt(temp);
  addChar('\n');
  Serial.write((const uint8_t*)lineBuf, lineLen);
}
