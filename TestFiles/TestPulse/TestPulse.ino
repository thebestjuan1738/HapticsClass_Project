/*
  TestPulse.ino - SparkFun/PulseSensor.com heart-rate sensor serial streamer
  Board: Arduino Uno R3 (works on R4 too)
  Sensor: Pulse Sensor (SparkFun SEN-11574, "PulseSensor Amped")

  Wiring (3 wires on the sensor):
    Purple (signal) -> A1
    Red    (+)      -> 5V   (3.3V also OK)
    Black  (-)      -> GND

  Tips:
    - Rest a fingertip (or earlobe) lightly on the front face - don't squeeze,
      that cuts off the blood flow and flattens the signal.
    - Cover the exposed side of the board with tape; skin contact with the
      electronics adds noise.
    - Keep still for the first ~10 s while it locks on to your beat.

  Signal: analog output centred at about Vcc/2 (~512 counts on the R3), with a
  bump on every pulse. Sampled at 500 Hz (every 2 ms), which is what the beat
  detector below expects.

  Serial at 115200 baud, 50 lines per second. Output:
    ms:..,signal:..,bpm:..,ibi:..,beat:..
      ms     millis() at print time
      signal raw ADC counts (0-1023)
      bpm    heart rate, averaged over the last 10 beats (0 until locked on)
      ibi    inter-beat interval in ms, latest beat (0 until locked on)
      beat   1 on the print line(s) where a new beat was just detected, else 0

  The sensor_bridge ROS node publishes each field as sensor/<field>.
  Only one program can hold the serial port (close the Serial Monitor first).

  The pulse sensor is on A1 so the EDA sensor can keep A0 on the same board
  (merge the print lines; both sketches use the same label:value format).
*/

const uint8_t  PULSE_PIN        = A1;     // A0 is left free for the EDA sensor
const uint16_t SAMPLE_PERIOD_US = 2000;   // 500 Hz sampling
const uint16_t PRINT_PERIOD_MS  = 20;     // 50 Hz printing

// Beat detector state (adaptive threshold halfway between recent peak/trough)
uint32_t sampleClockMs = 0;     // advances 2 ms per sample
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

// Output state
bool     beatLatch     = false; // held until the next print
uint32_t nextSampleUs  = 0;
uint32_t lastPrintMs   = 0;
int      signalValue   = 0;

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

void processSample(int sig) {
  sampleClockMs += 2;
  int sinceBeat = (int)(sampleClockMs - lastBeatMs);
  int minGap = (ibi / 5) * 3;   // ignore the dicrotic notch right after a beat

  // Track the trough on the falling side and the peak on the rising side
  if (sig < thresh && sinceBeat > minGap && sig < trough) trough = sig;
  if (sig > thresh && sig > peak) peak = sig;

  // Rising edge through the threshold = a beat
  if (sinceBeat > 250 && sig > thresh && !inPulse && sinceBeat > minGap) {
    inPulse = true;
    ibi = sinceBeat;
    lastBeatMs = sampleClockMs;

    if (firstBeat) {
      // Need two beats before the interval is trustworthy
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

  // Falling edge: re-centre the threshold for the next beat
  if (sig < thresh && inPulse) {
    inPulse = false;
    amp = peak - trough;
    thresh = amp / 2 + trough;
    peak = thresh;
    trough = thresh;
  }

  // No beat for 2.5 s (finger removed, etc.): start over
  if (sinceBeat > 2500) resetDetector();
}

void setup() {
  Serial.begin(115200);
  pinMode(PULSE_PIN, INPUT);
  Serial.println(F("Pulse sensor test starting. Rest a fingertip lightly on the sensor..."));
  delay(500);
  resetDetector();
  nextSampleUs = micros();
}

void loop() {
  // Fixed 500 Hz sampling (signed subtraction handles micros() rollover)
  if ((int32_t)(micros() - nextSampleUs) >= 0) {
    nextSampleUs += SAMPLE_PERIOD_US;
    signalValue = analogRead(PULSE_PIN);
    processSample(signalValue);
  }

  uint32_t now = millis();
  if (now - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = now;
    Serial.print(F("ms:"));        Serial.print(now);
    Serial.print(F(",signal:"));   Serial.print(signalValue);
    Serial.print(F(",bpm:"));      Serial.print(bpm);
    Serial.print(F(",ibi:"));      Serial.print(bpm > 0 ? ibi : 0);
    Serial.print(F(",beat:"));     Serial.println(beatLatch ? 1 : 0);
    beatLatch = false;
  }
}
