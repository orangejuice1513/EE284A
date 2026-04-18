const int beamPin = 27;

void setup() {
  Serial.begin(115200);
  pinMode(beamPin, INPUT_PULLUP);
}
void loop() {
  int state = digitalRead(beamPin);
  if (state == HIGH) {
    Serial.println(1);
  } 
  else {
    Serial.println(0);
  }
  delay(10);
}