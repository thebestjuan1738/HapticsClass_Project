/*
  TestTemp.ino - SparkFun LilyPad Temperature Sensor (DEV-08777) serial streamer
  Board: Arduino Uno R4 Minima (also builds for the R3)
  Sensor: MCP9700-type analog temperature sensor on a LilyPad board.
          Output (from the SparkFun repo): 0.5 V at 0 C, 0.75 V at 25 C,
          10 mV per degree C.   =>   temp_c = (volts - 0.5) * 100

  Wiring (3 pads on the LilyPad):
    S tab (signal) -> A3
    + tab          -> 3.3V   (the SparkFun example code assumes 3.3V)
    - tab          -> GND

  Pins in use across the test sketches: A0 EDA, A1 pulse, A2 pressure, A3 temp.

  NOTE: the SparkFun example converts counts with 3.3 V / 1023. That is only
  right if the ADC reference is 3.3 V on a 10-bit board. The R3/R4 ADC
  reference is 5 V by default, and the R4 is read at 14-bit here, so this
  sketch uses ADC_REF_VOLTS and ADC_MAX instead. The sensor's output is not
  tied to its supply voltage, so powering it from 3.3 V while the ADC
  reference is 5 V is fine.

  Resolution: 10 mV/C against a 5 V reference gives about 0.5 C per count at
  10-bit but about 0.03 C per count at 14-bit, so the R4's 14-bit read is what
  makes small temperature changes visible. Each sample here is the average of
  several reads to cut ADC noise.

  Accuracy: not stated in the SparkFun repo. Treat absolute readings as rough
  (a degree or two) and trust RELATIVE changes within a session. Temperature
  also responds slowly (seconds), especially through tape or skin.

  Mounting: tape the sensor flat against the skin (fingertip, wrist, or the
  back of the hand) with the chip side towards the skin. Let it settle for
  about a minute before judging a reading.

  Serial at 115200 baud, 10 lines per second. Output:
    ms:..,raw:..,volts:..,temp_c:..,delta_c:..
      ms       millis() at print time
      raw      ADC counts, averaged (0-1023 on R3, 0-16383 on R4)
      volts    raw converted to volts
      temp_c   temperature in degrees C (smoothed)
      delta_c  temp_c minus the reading taken for 1 s after reset. Keep the
               sensor in its starting position, in contact, during that second.

  The sensor_bridge ROS node publishes each field as sensor/<field>.
  Only one program can hold the serial port (close the Serial Monitor first).
*/

const uint8_t  TEMP_PIN         = A3;
const float    ADC_REF_VOLTS    = 5.0;    // R3 and R4 default reference

// ADC resolution: the R4 can read 14-bit (16x finer steps); the R3 is 10-bit.
// This picks the right one for whichever board the sketch is built for.
#if defined(ARDUINO_UNOR4_MINIMA) || defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_ARCH_RENESAS_UNO)
  #define ADC_BITS 14
#else
  #define ADC_BITS 10
#endif
const uint16_t ADC_MAX          = (1 << ADC_BITS) - 1;   // 1023 on R3, 16383 on R4

const uint16_t SAMPLE_PERIOD_MS = 20;     // 50 Hz sampling
const uint16_t PRINT_PERIOD_MS  = 100;    // 10 Hz printing
const uint8_t  AVG_READS        = 8;      // analogReads averaged per sample
const float    SMOOTH_ALPHA     = 0.1;    // 0..1, lower = smoother
const uint16_t BASELINE_MS      = 1000;   // startup reference measurement

float    smoothedCounts = 0;
float    baselineC      = 0;
uint32_t lastSample     = 0;
uint32_t lastPrint      = 0;

float readCounts() {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < AVG_READS; i++) sum += analogRead(TEMP_PIN);
  return (float)sum / AVG_READS;
}

float countsToCelsius(float counts) {
  float volts = counts * ADC_REF_VOLTS / ADC_MAX;
  return (volts - 0.5) * 100.0;
}

void setup() {
  Serial.begin(115200);
  pinMode(TEMP_PIN, INPUT);
#if ADC_BITS > 10
  analogReadResolution(ADC_BITS);
#endif

  Serial.println(F("Temperature test starting. Keep the sensor in place for a second..."));

  // Reference reading for delta_c
  uint32_t start = millis();
  float sum = 0;
  uint32_t n = 0;
  while (millis() - start < BASELINE_MS) {
    sum += readCounts();
    n++;
    delay(SAMPLE_PERIOD_MS);
  }
  smoothedCounts = sum / n;
  baselineC = countsToCelsius(smoothedCounts);
}

void loop() {
  uint32_t now = millis();

  if (now - lastSample >= SAMPLE_PERIOD_MS) {
    lastSample = now;
    smoothedCounts += SMOOTH_ALPHA * (readCounts() - smoothedCounts);
  }

  if (now - lastPrint >= PRINT_PERIOD_MS) {
    lastPrint = now;
    float volts = smoothedCounts * ADC_REF_VOLTS / ADC_MAX;
    float tempC = countsToCelsius(smoothedCounts);
    Serial.print(F("ms:"));        Serial.print(now);
    Serial.print(F(",raw:"));      Serial.print(smoothedCounts, 1);
    Serial.print(F(",volts:"));    Serial.print(volts, 4);
    Serial.print(F(",temp_c:"));   Serial.print(tempC, 2);
    Serial.print(F(",delta_c:"));  Serial.println(tempC - baselineC, 2);
  }
}
