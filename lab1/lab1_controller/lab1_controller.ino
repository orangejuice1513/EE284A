/* Lab 1: implementation of a  closed-loop speed control system.
* (1) measure motor speed (RPM) using the IR sensor
* (2) use the measured RPM to form a feedback loop that regulates the motor to a target speed of 60 RPM
*/

const int beamPin = 27;
const int servoPin = 13;

const int pwmResolution = 16; // 16-bit resolution
const int pwmFreq = 50;

uint32_t minPulse = 1550;
uint32_t midPulse = 1650;
uint32_t maxPulse = 1750;

const int rpmTarget = 60; 

volatile uint32_t curTime = 0; 
volatile uint32_t lastBlockedTime = 0; 
volatile uint32_t interval = 0; // in microseconds 

void setup() {
  Serial.begin(115200); 
  pinMode(beamPin, INPUT_PULLUP); // initialize IR receiver 

  ledcAttach(servoPin, pwmFreq, pwmResolution); // configures GPIO pin for pwm 
  ledcWrite(servoPin, dutyFromUs(minPulse)); //sets the PWM sent to the servo 

  // get interval 
  attachInterrupt(digitalPinToInterrupt(beamPin), beamISR, FALLING); // when beamPin == 0 (servo flag passes thru beam), take note of this 

  Serial.println("Measuring RPM...");
}

void loop() {
  if(interval > 0){ // prevent divison by 0 at initialization
    float RPM = 60000000.0f / interval; // 60 000 000 us / interval 
    Serial.printf("Interval: %lu us | RPM: %.2f\n", interval, RPM);
  }
  delay(500);
}

// Convert pulse width in microseconds to duty cycle value
uint32_t dutyFromUs(uint32_t pulseUs) {
  const uint32_t maxDuty = (1UL << pwmResolution) - 1;
  return (pulseUs * maxDuty) / 20000UL;
} 

// run this when IR sensor stops detecting the IR beam
// (the servo passes thru the beam)
void beamISR(){
  curTime = micros();
  interval = curTime - lastBlockedTime;
  lastBlockedTime = curTime;  
}


