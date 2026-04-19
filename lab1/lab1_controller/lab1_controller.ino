/* Lab 1: Closed-loop speed control with proportional gain
 * Target: 60 RPM using an IR beam-break sensor for feedback
 */

const int beamPin  = 27;
const int servoPin = 13;

const int pwmResolution = 16;
const int pwmFreq       = 50;

// Pulse width safe limits
const uint32_t minPulse = 1550;
const uint32_t maxPulse = 1750;

const float rpmTarget = 60.0f;

const float Kp = 1.0f; //proportional gain 

float currentPulse = 1650.0f;

volatile uint32_t lastBlockedTime = 0;
volatile uint32_t interval        = 0;
volatile bool     newReading      = false;

// Convert pulse width in microseconds to PWM duty cycle value
uint32_t dutyFromUs(uint32_t pulseUs) {
  const uint32_t maxDuty = (1UL << pwmResolution) - 1;
  return (pulseUs * maxDuty) / 20000UL;
}

// ISR: run when IR beam is broken (once per rev)
void IRAM_ATTR onBeamBreak() {
  uint32_t curTime = micros();
  uint32_t dt = curTime - lastBlockedTime;
  if (dt < 400000) return;                 // cause the flag kinda big 
  interval = dt;
  lastBlockedTime = curTime;
  newReading = true;
}

void setup() {
  Serial.begin(115200);
  pinMode(beamPin, INPUT_PULLUP);

  ledcAttach(servoPin, pwmFreq, pwmResolution);
  ledcWrite(servoPin, dutyFromUs((uint32_t)currentPulse));

  attachInterrupt(digitalPinToInterrupt(beamPin), onBeamBreak, FALLING);

  Serial.println("Starting closed-loop P control. Target = 60 RPM");
}

void loop() {
  if (!newReading) return;
  newReading = false;

  uint32_t localInterval = interval;
  if (localInterval == 0) return;

  float rpmMeasured = 60000000.0f / localInterval;

  float error = rpmTarget - rpmMeasured;   // how far off we are 
  float correction = Kp * error;            // push pulse by a proportional amount
  currentPulse += correction;

  // clamp to [minPulse, maxPulse]
  if (currentPulse < minPulse) currentPulse = minPulse;
  if (currentPulse > maxPulse) currentPulse = maxPulse;

  // Send the new pulse width to the servo
  ledcWrite(servoPin, dutyFromUs((uint32_t)currentPulse));

  Serial.printf("RPM: %.1f | Err: %+.1f | Pulse: %u\n",
                rpmMeasured, error, (uint32_t)currentPulse);
}