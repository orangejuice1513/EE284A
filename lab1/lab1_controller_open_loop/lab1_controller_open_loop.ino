/* Lab 1: open-loop RPM measurement via IR beam-break sensor */

const int beamPin  = 27;
const int servoPin = 13;

const int pwmResolution = 16;
const int pwmFreq       = 50;

uint32_t minPulse = 1550;
uint32_t midPulse = 1650;
uint32_t maxPulse = 1750;

volatile uint32_t lastBlockedTime = 0;
volatile uint32_t interval        = 0;
volatile bool     newReading      = false;

uint32_t dutyFromUs(uint32_t pulseUs) {
  const uint32_t maxDuty = (1UL << pwmResolution) - 1;
  return (pulseUs * maxDuty) / 20000UL;
}

// isr: runs when the IR beam is broken 
void IRAM_ATTR onBeamBreak() {

  uint32_t curTime = micros();
  uint32_t dt = curTime - lastBlockedTime;  
  if (dt < 400000) return;                  // cause the flag kinda big 
  interval = dt;
  lastBlockedTime = curTime;
  newReading = true;
}

void setup() {
  Serial.begin(115200);
  pinMode(beamPin, INPUT_PULLUP);

  ledcAttach(servoPin, pwmFreq, pwmResolution);
  ledcWrite(servoPin, dutyFromUs(maxPulse));   

  attachInterrupt(digitalPinToInterrupt(beamPin), onBeamBreak, FALLING);

  Serial.println("Measuring RPM...");
}

void loop() {
  if (newReading) {
    newReading = false;

    uint32_t localInterval = interval;

    if (localInterval > 0) {
      float rpm = 60000000.0f / localInterval;
      Serial.printf("Interval: %lu us | RPM: %.2f\n", localInterval, rpm);
    }
  }
  delay(100);  
}