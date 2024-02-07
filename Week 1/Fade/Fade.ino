int led = 11;           // the PWM pin the LED is attached to
int brightness = 0;    // how bright the LED is

// the setup routine runs once when you press reset:
void setup() {
  // declare pin 9 to be an output:
  pinMode(led, OUTPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  for (brightness = 255; brightness >= 0; brightness -=5) {
    analogWrite(led, brightness);
    delay(300);
  }
  for (brightness = 0; brightness <= 255; brightness += 5) {
    analogWrite(led, brightness);
    delay(300);
  }
}
