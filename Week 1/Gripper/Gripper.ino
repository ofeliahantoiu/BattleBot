#include <Servo.h>

const int servoPin = 2;
Servo gripperServo;

void setup() {
  gripperServo.attach(servoPin);
}

void loop() {
  moveServo(180);  // Open gripper
  delay(2000);
  moveServo(0);    // Close gripper
  delay(2000);
}

void moveServo(int angle) {
  gripperServo.write(angle);
  delay(500);
}
