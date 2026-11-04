/*
  TestEda.ino - proof-of-concept EDA (electrodermal activity) reader
  Board: Arduino Uno R3 (portable to R4 - see notes below)
  Sensor: ODAX EDA Maker board (datasheet ODAX-EDA-MB1-R1, Rev R1)

  Wiring (verify order against the board silkscreen):
    Sensor VIN  -> 5V   (3.3V also OK; datasheet allows 3.0-5.0V)
    Sensor GND  -> GND
    Sensor VOUT -> A0
    Electrode leads -> 3.5mm jack on the board (NOT to GND)

  SAFETY (datasheet 4.2): power from battery only while electrodes touch skin.
  No mains adapter. Run the laptop on battery (charger unplugged) when wearing
  the electrodes, or use a USB isolator.

  Signal: VOUT is ~0.1-1.2V typical, higher = more skin conductance.
  With the R3's 5V reference that is only ~20-245 ADC counts, which is
  fine for a demo. Bandwidth is ~6 Hz, so 50 Hz sampling is plenty.

  Serial at 115200 baud. Output: raw:..,smoothed:..,delta:..,volts:..
  Live plot: Arduino IDE Tools > Serial Plotter, or run plot_eda.py
  (close one before opening the other - only one program can hold the port).
*/

const uint8_t  EDA_PIN          = A0;     // analog input pin
const float    ADC_REF_VOLTS    = 5.0;    // R3 = 5V. R4 is also 5V, but ADC can be 14-bit (see below)
const uint16_t ADC_MAX          = 1023;   // 10-bit on R3. If you call analogReadResolution(14) on R4, use 16383
const uint16_t SAMPLE_PERIOD_MS = 20;     // sample at 50 Hz
const uint16_t PRINT_PERIOD_MS  = 100;    // print 10x per second
const float    SMOOTH_ALPHA     = 0.1;    // 0..1, lower = smoother
const uint16_t BASELINE_MS      = 5000;   // time spent measuring baseline at startup

float smoothed = 0;
float baseline = 0;
uint32_t lastSample = 0;
uint32_t lastPrint  = 0;

void setup() {
  Serial.begin(115200);
  pinMode(EDA_PIN, INPUT);

  Serial.println(F("EDA test starting. Keep fingers on electrodes and stay still..."));

  // Average readings for a few seconds to establish a resting baseline
  uint32_t start = millis();
  uint32_t sum = 0, n = 0;
  while (millis() - start < BASELINE_MS) {
    sum += analogRead(EDA_PIN);
    n++;
    delay(SAMPLE_PERIOD_MS);
  }
  baseline = (float)sum / n;
  smoothed = baseline;

  Serial.print(F("Baseline raw = "));
  Serial.println(baseline, 1);
  Serial.println(F("Streaming..."));
}

void loop() {
  uint32_t now = millis();

  // Fast sampling + low-pass filter (exponential moving average)
  if (now - lastSample >= SAMPLE_PERIOD_MS) {
    lastSample = now;
    int raw = analogRead(EDA_PIN);
    smoothed += SMOOTH_ALPHA * (raw - smoothed);
  }

  // Slower, human-readable printing
  if (now - lastPrint >= PRINT_PERIOD_MS) {
    lastPrint = now;
    int raw = analogRead(EDA_PIN);
    float volts = raw * ADC_REF_VOLTS / ADC_MAX;

    // "label:value" pairs: readable in Serial Monitor, plotted by the Arduino
    // IDE Serial Plotter, and parsed by plot_eda.py
    Serial.print(F("raw:"));       Serial.print(raw);
    Serial.print(F(",smoothed:")); Serial.print(smoothed, 1);
    Serial.print(F(",delta:"));    Serial.print(smoothed - baseline, 1);
    Serial.print(F(",volts:"));    Serial.println(volts, 3);
  }
}
