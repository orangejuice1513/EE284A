/* Lab 1 Closed-loop speed control with PID */

const int beamPin  = 27;
const int servoPin = 13;

const int pwmResolution = 16;
const int pwmFreq       = 50;

// Pulse width limits — safety clamp
uint32_t minPulse = 1550;
uint32_t maxPulse = 1750;

const float rpmTarget = 60.0f;

// Start with just Kp nonzero, then add Ki, then Kd.
const float Kp = 1.0f;   // proportional gain
const float Ki = 0.0f;   // integral gain
const float Kd = 0.0f;   // derivative gain

float integralSum  = 0;     // running sum of error over time
float lastError    = 0;     // previous error, for derivative
uint32_t lastPidTime = 0;   // timestamp of last PID update (ms)

float currentPulse = 1650.0f;

volatile uint32_t lastBlockedTime = 0;
volatile uint32_t interval        = 0;
volatile bool     newReading      = false;

uint32_t dutyFromUs(uint32_t pulseUs) {
  const uint32_t maxDuty = (1UL << pwmResolution) - 1;
  return (pulseUs * maxDuty) / 20000UL;
}

void IRAM_ATTR onBeamBreak() {
  uint32_t curTime = micros();
  uint32_t dt = curTime - lastBlockedTime; 
  if (dt < 400000) return;                     
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

  lastPidTime = millis();
  Serial.println("Starting closed-loop control. Target = 60 RPM");
}

void loop() {
  // Wait for RPM measurement from the ISR
  if (!newReading) return;
  newReading = false;

  uint32_t localInterval = interval;
  if (localInterval == 0) return;

  float rpmMeasured = 60000000.0f / localInterval;

  // time step (dt) since last PID update 
  uint32_t now = millis();
  float dt = (now - lastPidTime) / 1000.0f;  //  ms to seconds
  lastPidTime = now;
  if (dt <= 0) return;  // sanity check

  // PID calculation 
  float error = rpmTarget - rpmMeasured;

  float P = Kp * error;

  integralSum += error * dt;
  float I = Ki * integralSum;

  float derivative = (error - lastError) / dt;
  float D = Kd * derivative;
  lastError = error;

  float correction = P + I + D;

  currentPulse += correction;

  // clamp to [minPulse, maxPulse]
  if (currentPulse < minPulse) {
    currentPulse = minPulse;
    integralSum -= error * dt;  
  }
  if (currentPulse > maxPulse) {
    currentPulse = maxPulse;
    integralSum -= error * dt;
  }

  ledcWrite(servoPin, dutyFromUs((uint32_t)currentPulse));  // send new pulse to the servo

  Serial.printf("RPM: %.1f | Err: %+.1f | P:%+.1f I:%+.1f D:%+.1f | Pulse: %u\n",
                rpmMeasured, error, P, I, D, (uint32_t)currentPulse);
}