/* Lab 1 LED Code: prompts the user via serial to input which pins they would like to turn on.
   For example, 011 turns on LEDs 2 and 3, 101 turns on LEDs 1 and 3. */

#define LED_1 14
#define LED_2 32
#define LED_3 15

void setup() {
  Serial.begin(115200);
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
}

void loop() {
  Serial.println("Which leds do u wanna turn on?");

  while (Serial.available() == 0) {
    delay(10);
  }

  String input = Serial.readString();

  digitalWrite(LED_1, input[0] == '1' ? HIGH : LOW);
  digitalWrite(LED_2, input[1] == '1' ? HIGH : LOW);
  digitalWrite(LED_3, input[2] == '1' ? HIGH : LOW);

  Serial.print("Set LEDs to: ");
  Serial.println(input);
}