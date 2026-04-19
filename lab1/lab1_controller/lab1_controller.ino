/* Lab 1: Closed-loop speed control with proportional gain
 * Target: 60 RPM using an IR beam-break sensor for feedback
 */

const int beamPin  = 27;
const int servoPin = 13;

const int pwmResolution = 12;
const int pwmFreq       = 50;

const uint32_t minPulse = 1550;
const uint32_t maxPulse = 1750;

const float rpmTarget = 60.0f; // target is 60 RPM 

const float Kp = 1.0f; //proportional gain: how many us of pulse change does 1 RPM error need 

float currentPulse = 1650.0f // initialization 

volatile uint32_t lastBlockedTime = 0;
volatile uint32_t interval        = 0;
volatile bool     newReading      = false;

// Convert pulse width in microseconds to PWM duty cycle value
uint32_t dutyFromUs(uint32_t pulseUs) {                                                                          
  const uint32_t maxDuty = (1UL << pwmResolution) - 1; // 4095 
  return (pulseUs / 20000UL ) * maxDuty;  
}

// ISR: run when IR beam is broken (once per rev)
// gets the time between when the flag passes thru this time - time the flag passed thru last time 
// with the interval we can calculate RPM 
void IRAM_ATTR onBeamBreak() {
  uint32_t curTime = micros(); 
  uint32_t interval = curTime - lastBlockedTime;
  if (interval < 400000) return;   // cause the flag kinda big 
  lastBlockedTime = curTime; 
  newReading = true;
}

void setup() {
  Serial.begin(115200);

  // initializes pins 
  pinMode(beamPin, INPUT_PULLUP); 
  ledcAttach(servoPin, pwmFreq, pwmResolution);

  ledcWrite(servoPin, dutyFromUs((uint32_t)currentPulse));  // sets the current pwm to currentPulse 

  attachInterrupt(digitalPinToInterrupt(beamPin), onBeamBreak, FALLING); 

  Serial.println("Starting closed-loop P control. Target = 60 RPM");
}

void loop() {
  if (!newReading) return;   // update pwm when we have a measurement (the flag passes thru)
  newReading = false;

  uint32_t localInterval = interval;
  if (localInterval == 0) return;

  float rpmMeasured = 60000000.0f / localInterval; // rpm = 60 seconds / interval 

  float error = rpmTarget - rpmMeasured;   // how far off we are 
  float correction = Kp * error;         // push pulse by a proportional amount
  currentPulse += correction;

  // clamp to [minPulse, maxPulse] 
  if (currentPulse < minPulse) currentPulse = minPulse;
  if (currentPulse > maxPulse) currentPulse = maxPulse;

  ledcWrite(servoPin, dutyFromUs((uint32_t)currentPulse));   // send the new pulse width to the servo

  Serial.printf("RPM: %.1f | Err: %+.1f | Pulse: %u\n",
                rpmMeasured, error, (uint32_t)currentPulse);
}