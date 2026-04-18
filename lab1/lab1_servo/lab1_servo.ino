/*Lab 1 PWM section: configures the ESP32 to generate a 50 Hz PWM signal using a 16-bit resolution timer
*/

const int servoPin = 13;
const int pwmResolution = 16; // 16-bit resolution
const int pwmFreq = 50;
uint32_t minPulse = 1550;
uint32_t midPulse = 1650;
uint32_t maxPulse = 1750;

// Convert pulse width in microseconds to duty cycle value
uint32_t dutyFromUs(uint32_t pulseUs) {
  const uint32_t maxDuty = (1UL << pwmResolution) - 1;
  return (pulseUs * maxDuty) / 20000UL;
} 

void setup() {
  ledcAttach(servoPin, pwmFreq, pwmResolution);
  delay(100);
  ledcWrite(servoPin, dutyFromUs(minPulse)); //sets the PWM sent to the servo 
}

void loop() {
}