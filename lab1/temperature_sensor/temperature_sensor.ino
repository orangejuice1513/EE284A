/* Lab 1 Temperature Sensor Code */

#define ADC_PIN A2 // input from the temperature sensor 

void setup() {
  Serial.begin(115200);  
}

void loop() {
  analogReadResolution(12);
  analogSetPinAttenuation(ADC_PIN, ADC_6db); // lock attenuation
  
  int raw_value = analogRead(ADC_PIN);
  float temp = digital_to_celsius(raw_value);
  float voltage = digital_to_voltage(temp);
  const char* above_status = (temp > 23.0) ? "above" : "below or equal to"; 
  Serial.printf("The raw ADC value is %d, which converts to %.2f V. The temperature is %s 23C, in fact it is %.2f C\n", raw_value, voltage, above_status, temp);
  delay(50);
}

float digital_to_celsius(int DN){
  // converts a DN to temperature
  return 100.0 * ( 2.1 / (2**12 - 1) * DN - 0.5 ); 
}

float digital_to_voltage(float temp){
  // converts a temperature to voltage output from sensor 
  return 0.5 + 0.01 * temp; 
}
