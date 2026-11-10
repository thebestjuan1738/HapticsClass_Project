/*
  TestPressure.ino - force-sensing resistor (FSR) serial streamer
  Board: Arduino Uno R3 (works on R4 too)
  Sensor: square FSR (e.g. Interlink FSR 406). Not polarised.

  Wiring (voltage divider with the FSR on the 5V side):
    5V --[ FSR ]--+--[ R_FIXED ]-- GND
                  |
                  +-- A2

  Output voltage rises as you press harder.
  Vout = 5V * R_FIXED / (R_FIXED + R_fsr)

  R_FIXED: 1k is what worked (10k saturated at a moderate press and picked up
  noise). A smaller resistor takes a smaller share of the 5V, so the sensor has
  to fall much further before the output hits the ceiling: less sensitive,
  wider range. Try 2.2-3.3k if light touches are hard to see, 47-100k only for
  very light touch. If you change it, update R_FIXED_OHMS below so fsr_kohm
  stays right.

  Setup notes (from bench testing):
    - Lay the sensor FLAT on a rigid surface, active side up. A bent or
      face-down sensor is preloaded and reads high (~200 counts) unloaded.
    - The FSR and the resistor must be IN SERIES, with A2 on the junction
      between them. A resistor from 5V straight to GND leaves A2 floating.
    - Floating pins read arbitrary values (~200). That is not a board fault.

  The FSR is a rough force indicator, not a calibrated scale: expect part-to-
  part variation, drift, and a response that is not linear. Compare relative
  changes within a session rather than trusting absolute numbers.

  Serial at 115200 baud, 50 lines per second. Output:
    ms:..,raw:..,smoothed:..,volts:..,fsr_kohm:..,delta:..,rate:..
      ms        millis() at print time
      raw       ADC counts (0-1023 on R3, 0-16383 on R4)
      smoothed  light low-pass of raw
      volts     smoothed, in volts
      fsr_kohm  FSR resistance in kilohms (0 means no/negligible force)
      delta     smoothed minus the no-load zero taken at startup (counts).
                Keep the sensor unloaded for the first second after reset.
      rate      change in smoothed, in counts per second, measured over
                100 ms (updates 10x per second, held between updates)

  Pin A2 keeps this clear of the EDA (A0) and pulse (A1) sensors.
  The sensor_bridge ROS node publishes each field as sensor/<field>.
  Only one program can hold the serial port (close the Serial Monitor first).
*/

const uint8_t  FSR_PIN          = A2;
const float    ADC_REF_VOLTS    = 5.0;
// ADC resolution: the R4 can read 14-bit (16x finer steps); the R3 is 10-bit.
// This picks the right one for whichever board the sketch is built for.
#if defined(ARDUINO_UNOR4_MINIMA) || defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_ARCH_RENESAS_UNO)
  #define ADC_BITS 14
#else
  #define ADC_BITS 10
#endif
const uint16_t ADC_MAX          = (1 << ADC_BITS) - 1;   // 1023 on R3, 16383 on R4
const float    R_FIXED_OHMS     = 1000.0;    // the resistor to GND
const uint16_t SAMPLE_PERIOD_MS = 5;         // 200 Hz sampling
const uint16_t PRINT_PERIOD_MS  = 20;        // 50 Hz printing
const float    SMOOTH_ALPHA     = 0.2;       // 0..1, lower = smoother
const uint16_t TARE_MS          = 1000;      // startup no-load zero measurement
const uint16_t RATE_PERIOD_MS   = 100;       // window for the rate-of-change

float    smoothed    = 0;
float    baseline    = 0;        // no-load zero
float    rate        = 0;        // counts per second
float    rateRef     = 0;
uint32_t lastRate    = 0;
uint32_t lastSample  = 0;
uint32_t lastPrint   = 0;
int      raw         = 0;

void setup() {
  Serial.begin(115200);
  pinMode(FSR_PIN, INPUT);
#if ADC_BITS > 10
  analogReadResolution(ADC_BITS);
#endif
  Serial.println(F("Pressure sensor test starting..."));
  smoothed = analogRead(FSR_PIN);

  // Zero the sensor: average the unloaded reading for a second
  uint32_t start = millis();
  float sum = 0;
  uint32_t n = 0;
  while (millis() - start < TARE_MS) {
    sum += analogRead(FSR_PIN);
    n++;
    delay(SAMPLE_PERIOD_MS);
  }
  baseline = sum / n;
  smoothed = baseline;
  rateRef = baseline;
  lastRate = millis();
}

void loop() {
  uint32_t now = millis();

  if (now - lastSample >= SAMPLE_PERIOD_MS) {
    lastSample = now;
    raw = analogRead(FSR_PIN);
    smoothed += SMOOTH_ALPHA * (raw - smoothed);
  }

  if (now - lastRate >= RATE_PERIOD_MS) {
    rate = (smoothed - rateRef) * 1000.0 / (now - lastRate);
    rateRef = smoothed;
    lastRate = now;
  }

  if (now - lastPrint >= PRINT_PERIOD_MS) {
    lastPrint = now;
    float volts = smoothed * ADC_REF_VOLTS / ADC_MAX;
    // Solve the divider for the FSR. Skip when Vout is ~0 (no force).
    float fsrKohm = 0;
    if (volts > 0.01) {
      fsrKohm = R_FIXED_OHMS * (ADC_REF_VOLTS / volts - 1.0) / 1000.0;
    }
    Serial.print(F("ms:"));          Serial.print(now);
    Serial.print(F(",raw:"));        Serial.print(raw);
    Serial.print(F(",smoothed:"));   Serial.print(smoothed, 1);
    Serial.print(F(",volts:"));      Serial.print(volts, 3);
    Serial.print(F(",fsr_kohm:"));   Serial.print(fsrKohm, 1);
    Serial.print(F(",delta:"));      Serial.print(smoothed - baseline, 1);
    Serial.print(F(",rate:"));       Serial.println(rate, 1);
  }
}
