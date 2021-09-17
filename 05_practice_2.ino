#define PIN_LED 7

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, 0); // update LED status.
  delay(1000);
  digitalWrite(PIN_LED, 1); // update LED status.
  delay(100);
  digitalWrite(PIN_LED, 0); // update LED status.
  delay(100);
  digitalWrite(PIN_LED, 1); // update LED status.
  delay(100);
  digitalWrite(PIN_LED, 0); // update LED status.
  delay(100);
  digitalWrite(PIN_LED, 1); // update LED status.
  delay(100);
  digitalWrite(PIN_LED, 0); // update LED status.
  delay(100);
  digitalWrite(PIN_LED, 1); // update LED status.
  delay(100);
  digitalWrite(PIN_LED, 0); // update LED status.
  delay(100);
  digitalWrite(PIN_LED, 1); // update LED status.
  delay(100);
  digitalWrite(PIN_LED, 0); // update LED status.
  delay(100);
  digitalWrite(PIN_LED, 1); // update LED status.
  delay(100);
}

void loop() {
} 
