#define PIN_LED 13
unsigned int count, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
}

void loop() {
  digitalWrite(PIN_LED, HIGH); // update LED status.
  delay(1000); // wait for 1,000 milliseconds
  digitalWrite(PIN_LED, LOW);
  delay(1000);
}
