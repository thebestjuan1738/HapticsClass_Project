/*
  TestAll.ino - all four sensors sampled together at one rate
  Board: Arduino Uno R4 Minima (built for 14-bit ADC; the R3 also builds, at 10-bit)

  Sensors and pins:
    EDA (ODAX)            VOUT      -> A0
    Pulse sensor          purple    -> A1   (red -> 5V, black -> GND)
    Pressure (FSR, 1k)    divider   -> A2   (5V -- FSR --+-- 1k -- GND, A2 at the +)
    Temperature (LilyPad) S tab     -> A3   (+ -> 3.3V, - -> GND)
  All grounds common. Wiring details are in README.md.

  Sampling: every sensor is read in the SAME tick (SAMPLE_RATE_HZ below:
  1000 Hz on the R4, to match the haptics loop). Within a tick
  the four reads happen one after another, so they are offset by well under
  1 ms. One discarded read precedes each real read so the shared ADC's
  sampling capacitor has settled on the new channel (matters for the
  high-impedance EDA and pressure nodes).

  Resolution: all raw channels are logged at 14-bit on the R4 (counts
  0-16383), 10-bit on the R3. The pulse beat detector below was tuned in
  10-bit counts, so it is fed the 14-bit reading shifted down to 10-bit; the
  logged `pulse` value is the full 14-bit reading.

  Serial: on the R4 the port is native USB, so the baud setting is ignored and
  a full line every tick is no problem. On the R3 (115200 baud) printing is cut
  to every 10th sample (50 Hz) to fit.

  Output (one line per print):
    ms:..,ovr:..,eda:..,eda_s:..,eda_d:..,pulse:..,bpm:..,ibi:..,beat:..,
    force:..,force_s:..,force_d:..,force_rate:..,temp:..,temp_c:..,temp_d:..
      ms          millis() at print time
      ovr         number of schedule slips so far: a tick that started a whole
                  period late, after which the missed ticks are skipped and
                  the schedule restarts (0 means the
                  schedule is being met; it should stay 0)
      eda         raw counts          eda_s  smoothed (about 0.2 s)
      eda_d       eda_s minus its value at startup
      pulse       raw counts (14-bit on R4)
      bpm, ibi    heart rate and last interval (ms) from the beat detector,
                  0 until it locks on. bpm is a 10-beat average (see README
                  for its known limits under motion)
      beat        1 on the line where a beat was just detected, else 0
      force       raw counts          force_s smoothed (about 25 ms)
      force_d     force_s minus its value at startup
      force_rate  change in force_s in counts/second over 100 ms
      temp        raw counts          temp_c  smoothed temperature in C
      temp_d      temp_c minus its value at startup
      acq_us      longest time (microseconds) a tick spent reading the four
                  sensors and running the pulse detector since the last
                  print. The budget is one tick: 1,000,000 / SAMPLE_RATE_HZ us
                  (1000 us at 1 kHz).
      tot_us      longest whole-tick time including printing, over the same
                  window (as of the previous tick). If acq_us is small but
                  tot_us is near 2000, printing is the bottleneck.

  Dry run: set DRY_RUN to 1 below to skip the per-sample lines entirely. The
  sketch then samples at SAMPLE_RATE_HZ exactly as it would in the real system, and
  prints ONE summary line per second: ms, ovr, acq_us (longest tick) and bpm.
  Open the Serial Monitor (any baud) or use plot_serial.py to read it. If ovr
  stays 0 in a dry run, acquisition alone is fast enough and the overruns come
  from printing.

  Startup: keep the EDA electrodes on, the pressure sensor unloaded, and the
  temperature sensor in place for the first 2 seconds after reset, because the
  *_d fields are measured against that moment. Press reset to re-zero. (The R4
  does not reset when the serial port opens.)

  Pulse timing: the detector runs at SAMPLE_RATE_HZ on the Arduino. The raw `pulse`
  channel is also logged at SAMPLE_RATE_HZ (when PRINT_EVERY_N is 1), so beats can be re-detected offline.

  The sensor_bridge ROS node publishes each field as sensor/<field>. At 1000
  lines/s x 18 fields that is about 18000 messages per second, which the
  Python bridge may not keep up with; for this test, log with
  plot_all.py --csv, or raise PRINT_EVERY_N.

  Only one program can hold the serial port (close the Serial Monitor first).

  What we learned getting this to run at 1 kHz without overruns:
    - Reading all four sensors plus the pulse detector takes only about 200 us
      per tick (acq_us). The cost was PRINTING, not sensing.
    - 35 separate Serial.print calls per line was too slow. printLine() now builds
      the line in a buffer with integer math and sends it with ONE Serial.write
      (whole tick about 700 us of the 1000 us budget).
    - DRY_RUN 1 separates the two costs: it samples at full rate and prints one
      summary line per second, so the acquisition time can be read on its own.
    - After a stall (for example the host stops reading and a USB write blocks)
      the old schedule ran the missed ticks back to back to catch up. That gave a
      burst of lines and an overrun count that kept climbing. Now a slip counts
      one overrun, skips the missed ticks and resyncs, so it shows as a gap in
      'ms'.
    - The Python side must read in big chunks on a background thread
      (plot_all.py does). pyserial's readline() reads one byte at a time and froze
      the window at about 200 KB/s.
*/

const uint8_t  EDA_PIN    = A0;
const uint8_t  PULSE_PIN  = A1;
const uint8_t  FORCE_PIN  = A2;
const uint8_t  TEMP_PIN   = A3;

const float    ADC_REF_VOLTS = 5.0;

// 1 = no per-sample lines; sample at full rate and print a 1-second timing summary
#define DRY_RUN 0

// ADC resolution: the R4 can read 14-bit (16x finer steps); the R3 is 10-bit.
#if defined(ARDUINO_UNOR4_MINIMA) || defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_ARCH_RENESAS_UNO)
  #define ADC_BITS 14
  const uint16_t PRINT_EVERY_N = 1;     // print every Nth sample (1 = every sample)
#else
  #define ADC_BITS 10
  const uint16_t PRINT_EVERY_N = 10;    // R3 serial is too slow for full-rate lines
#endif
const uint16_t ADC_MAX = (1 << ADC_BITS) - 1;     // 1023 on R3, 16383 on R4
const uint8_t  PULSE_SHIFT = ADC_BITS - 10;       // 14-bit -> 10-bit for the detector

// One sample rate for every sensor; all timing below is derived from it.
#if ADC_BITS > 10
  const uint16_t SAMPLE_RATE_HZ = 1000;   // R4: matches the haptics loop
#else
  const uint16_t SAMPLE_RATE_HZ = 250;    // R3: analogRead is slow (~112 us each)
#endif
const uint32_t SAMPLE_PERIOD_US = 1000000UL / SAMPLE_RATE_HZ;
const uint16_t TICK_MS          = SAMPLE_PERIOD_US / 1000;   // detector clock step (rate must divide 1000)
const uint16_t TARE_MS          = 2000;           // startup reference measurement
const uint16_t RATE_PERIOD_MS   = 100;            // window for force_rate

// Smoothing: time constants, converted to a per-tick factor at the chosen rate
const float DT_S        = SAMPLE_PERIOD_US / 1000000.0;
const float EDA_ALPHA   = DT_S / 0.200;   // 0.2 s time constant
const float FORCE_ALPHA = DT_S / 0.025;   // 25 ms
const float TEMP_ALPHA  = DT_S / 0.200;   // 0.2 s

// Smoothed channels and their startup references
float edaS = 0,   edaBase = 0;
float forceS = 0, forceBase = 0;
float tempS = 0,  tempBaseC = 0;

// force_rate
float    forceRate = 0;
float    forceRef  = 0;
uint32_t lastRate  = 0;

// Schedule
uint32_t nextSampleUs = 0;
uint32_t overruns     = 0;
uint16_t printCount   = 0;
uint32_t acqMax       = 0;      // longest acquisition time since last print (us)
uint32_t totMax       = 0;      // longest whole-tick time since last print (us)
uint32_t lastSummary  = 0;

// ---- Pulse beat detector (same algorithm as TestPulse.ino) ----
uint32_t sampleClockMs = 0;     // advances TICK_MS per sample
uint32_t lastBeatMs    = 0;
int      peak          = 512;
int      trough        = 512;
int      thresh        = 530;
int      amp           = 100;
bool     inPulse       = false;
bool     firstBeat     = true;
bool     secondBeat    = false;
int      ibi           = 600;   // ms, running estimate
int      bpm           = 0;
uint16_t ibiHistory[10];
bool     beatLatch     = false; // held until the next print

void resetDetector() {
  peak = trough = 512;
  thresh = 530;
  amp = 100;
  lastBeatMs = sampleClockMs;
  inPulse = false;
  firstBeat = true;
  secondBeat = false;
  bpm = 0;
  ibi = 600;
}

void processPulse(int sig) {          // sig is on the 10-bit scale
  sampleClockMs += TICK_MS;
  int sinceBeat = (int)(sampleClockMs - lastBeatMs);
  int minGap = (ibi / 5) * 3;   // ignore the dicrotic notch right after a beat

  if (sig < thresh && sinceBeat > minGap && sig < trough) trough = sig;
  if (sig > thresh && sig > peak) peak = sig;

  if (sinceBeat > 250 && sig > thresh && !inPulse && sinceBeat > minGap) {
    inPulse = true;
    ibi = sinceBeat;
    lastBeatMs = sampleClockMs;

    if (firstBeat) {
      firstBeat = false;
      secondBeat = true;
    } else {
      if (secondBeat) {
        secondBeat = false;
        for (uint8_t i = 0; i < 10; i++) ibiHistory[i] = ibi;
      }
      uint32_t sum = 0;
      for (uint8_t i = 0; i < 9; i++) {
        ibiHistory[i] = ibiHistory[i + 1];
        sum += ibiHistory[i];
      }
      ibiHistory[9] = ibi;
      sum += ibi;
      bpm = (int)(60000UL / (sum / 10));
      beatLatch = true;
    }
  }

  if (sig < thresh && inPulse) {
    inPulse = false;
    amp = peak - trough;
    thresh = amp / 2 + trough;
    peak = thresh;
    trough = thresh;
  }

  if (sinceBeat > 2500) resetDetector();
}

// One discarded read first so the ADC settles on the new channel.
int readAdc(uint8_t pin) {
  analogRead(pin);
  return analogRead(pin);
}

float countsToCelsius(float counts) {
  float volts = counts * ADC_REF_VOLTS / ADC_MAX;
  return (volts - 0.5) * 100.0;      // MCP9700-type: 0.5 V at 0 C, 10 mV/C
}

void setup() {
  Serial.begin(115200);
  pinMode(EDA_PIN, INPUT);
  pinMode(PULSE_PIN, INPUT);
  pinMode(FORCE_PIN, INPUT);
  pinMode(TEMP_PIN, INPUT);
#if ADC_BITS > 10
  analogReadResolution(ADC_BITS);
#endif

  Serial.println(F("Combined sensor test starting. Hold still for 2 seconds..."));

  // Startup reference for the *_d fields: average each channel for TARE_MS
  uint32_t start = millis();
  float sumEda = 0, sumForce = 0, sumTemp = 0;
  uint32_t n = 0;
  uint32_t t = micros();
  while (millis() - start < TARE_MS) {
    if ((int32_t)(micros() - t) >= 0) {
      t += SAMPLE_PERIOD_US;
      sumEda += readAdc(EDA_PIN);
      sumForce += readAdc(FORCE_PIN);
      sumTemp += readAdc(TEMP_PIN);
      n++;
    }
  }
  edaS = edaBase = sumEda / n;
  forceS = forceBase = sumForce / n;
  tempS = sumTemp / n;
  tempBaseC = countsToCelsius(tempS);
  forceRef = forceS;

  resetDetector();
  lastRate = millis();
  nextSampleUs = micros() + SAMPLE_PERIOD_US;
}

// ---- Fast line building ----
// A line used to be ~35 separate Serial.print calls, which is what made the
// sampling schedule slip. Instead the line is assembled in a buffer with plain
// integer math (no float printf needed) and sent with ONE Serial.write.
char     lineBuf[320];
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

// Append v with 0, 1 or 2 decimals (rounded), e.g. addFixed(25.014, 2) -> 25.01
void addFixed(float v, uint8_t decimals) {
  long scale = (decimals == 2) ? 100 : (decimals == 1 ? 10 : 1);
  long scaled = (long)(v * scale + (v < 0 ? -0.5f : 0.5f));
  bool neg = scaled < 0;
  unsigned long u = neg ? (unsigned long)(-scaled) : (unsigned long)scaled;
  if (neg) addChar('-');
  addInt((long)(u / scale));
  if (decimals > 0) {
    addChar('.');
    unsigned long frac = u % scale;
    if (decimals == 2 && frac < 10) addChar('0');
    addInt((long)frac);
  }
}

void printLine(uint32_t now, int eda, int pulse, int force, int temp) {
  float tempC = countsToCelsius(tempS);
  lineLen = 0;
  addStr("ms:");           addInt((long)now);
  addStr(",ovr:");         addInt((long)overruns);
  addStr(",eda:");         addInt(eda);
  addStr(",eda_s:");       addFixed(edaS, 1);
  addStr(",eda_d:");       addFixed(edaS - edaBase, 1);
  addStr(",pulse:");       addInt(pulse);
  addStr(",bpm:");         addInt(bpm);
  addStr(",ibi:");         addInt(bpm > 0 ? ibi : 0);
  addStr(",beat:");        addInt(beatLatch ? 1 : 0);
  addStr(",force:");       addInt(force);
  addStr(",force_s:");     addFixed(forceS, 1);
  addStr(",force_d:");     addFixed(forceS - forceBase, 1);
  addStr(",force_rate:");  addFixed(forceRate, 1);
  addStr(",temp:");        addInt(temp);
  addStr(",temp_c:");      addFixed(tempC, 2);
  addStr(",temp_d:");      addFixed(tempC - tempBaseC, 2);
  addStr(",acq_us:");      addInt((long)acqMax);
  addStr(",tot_us:");      addInt((long)totMax);
  addChar('\n');
  Serial.write((const uint8_t*)lineBuf, lineLen);   // one write for the whole line
  beatLatch = false;
  acqMax = 0;
  totMax = 0;
}

void sampleAll() {
  uint32_t t0 = micros();

  int eda   = readAdc(EDA_PIN);
  int pulse = readAdc(PULSE_PIN);
  int force = readAdc(FORCE_PIN);
  int temp  = readAdc(TEMP_PIN);

  edaS   += EDA_ALPHA   * (eda   - edaS);
  forceS += FORCE_ALPHA * (force - forceS);
  tempS  += TEMP_ALPHA  * (temp  - tempS);

  processPulse(pulse >> PULSE_SHIFT);

  uint32_t now = millis();
  if (now - lastRate >= RATE_PERIOD_MS) {
    forceRate = (forceS - forceRef) * 1000.0 / (now - lastRate);
    forceRef = forceS;
    lastRate = now;
  }

  uint32_t acq = micros() - t0;
  if (acq > acqMax) acqMax = acq;

#if DRY_RUN
  // No per-sample output: just a once-per-second timing summary.
  if (now - lastSummary >= 1000) {
    lastSummary = now;
    Serial.print(F("ms:"));      Serial.print(now);
    Serial.print(F(",ovr:"));    Serial.print(overruns);
    Serial.print(F(",acq_us:")); Serial.print(acqMax);
    Serial.print(F(",bpm:"));    Serial.println(bpm);
    acqMax = 0;
  }
#else
  if (++printCount >= PRINT_EVERY_N) {
    printCount = 0;
    printLine(now, eda, pulse, force, temp);
  }
#endif

  uint32_t tot = micros() - t0;
  if (tot > totMax) totMax = tot;
}

void loop() {
  // Fixed-rate schedule (signed subtraction handles micros() rollover)
  int32_t late = (int32_t)(micros() - nextSampleUs);
  if (late >= 0) {
    if (late >= (int32_t)SAMPLE_PERIOD_US) {
      // Slipped by a whole tick or more (for example the host stopped reading
      // and a USB write blocked). Do NOT run the missed ticks back to back to
      // catch up: that makes a burst of lines with bunched-up timestamps and
      // keeps counting overruns for as long as it takes to recover. Skip them
      // and restart the schedule from now. The gap shows up in 'ms'.
      overruns++;
      nextSampleUs = micros();
    }
    nextSampleUs += SAMPLE_PERIOD_US;
    sampleAll();
  }
}
