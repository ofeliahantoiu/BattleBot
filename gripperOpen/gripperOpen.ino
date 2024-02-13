// LFM = Left Forward Motor 
// LBM = Left Backward Motor
// RFM = Right Forward Motor
// RBM = Right Backward Motor
#include <Servo.h>

Servo sensorServo;
Servo gripServo;

bool wait = true;
bool isGripping = false;

const int LFM = 6;
const int LBM = 9;
const int RFM = 5;
const int RBM = 3;

const int trig = 11; // Ultrasonic sensor trigger 
const int echo = 12; // Ultrasonic sensor echo

const int servoSensor = 10;
const int servoGrip = 2;

const int minPulseWidth = 1000; // Minimum pulse width for servo
const int maxPulseWidth = 2000; // Maximum pulse width for servo

const int sensorOne = A0;
const int sensorTwo = A1;
const int sensorThree = A2;
const int sensorFour = A3;
const int sensorFive = A4;
const int sensorSix = A5;
const int sensorSeven = A6;
const int sensorEight = A7;

int distanceFront, distanceRight, distanceLeft;

int duration, distance;
int lastError = 0;

void setup() {
  pinMode(LFM, OUTPUT);
  pinMode(LBM, OUTPUT);
  pinMode(RFM, OUTPUT);
  pinMode(RBM, OUTPUT);

  pinMode(servoSensor, OUTPUT);
  pinMode(servoGrip, OUTPUT);

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  // Attach servo motors
  sensorServo.attach(servoSensor);
  gripServo.attach(servoGrip);
  gripServo.write(90); // Set initial grip position
}

void loop() {
  distanceFront = getDistance();
  distanceRight = lookRight();
  distanceLeft = lookLeft();

  // Check for object detection continuously
  detectObject();

  if (detectBlackLine()) {
    handleBlackLineDetection();
  }

  if (distanceFront > 20) {
    moveForward();
    
    if (distanceLeft > 10 && distanceLeft < 20) {
      moveForward();
    } else if (distanceLeft >= 20) {
      turnLeft();
      delay(200); 
    } else if (distanceLeft < 10 && distanceLeft > 0) {
      turnRight();
      delay(200); 
    }
  } else {
    moveStop();
    delay(500); 
    
    if (distanceRight > 20) {
      turnRight();
      delay(200); 
    } else {
      turnRight();
      delay(400); 
    }
  }
}

void detectObject() {
  int distance = getDistance();

  if (distance < 25) {
    // Check if the gripper is not currently gripping an object
    if (!isGripping) {
      gripServo.write(90); // Close the gripper to pick up the object
      isGripping = true;
    }
  } else {
    isGripping = false; // Reset the gripping status
  }
}

void handleBlackLineDetection() {
  if (isGripping) {
    gripServo.write(0); // Drop the object
    delay(1000); // Delay to ensure the object is dropped
    isGripping = false;
  } else {
    gripServo.write(90); // Close the gripper to pick up the object
    isGripping = true;
  }
  wait = true; // Set wait to true to detect the next object
}

void moveForward()
{
    analogWrite(LFM, 250);
    analogWrite(RFM, 250);
    analogWrite(LBM, 0);
    analogWrite(RBM, 0);
}

void turnLeft()
{
  analogWrite(LBM, 130);
  analogWrite(RFM, 250);
  analogWrite(LFM, 0);
  analogWrite(RBM, 0);

  delay(500);

  moveForward();
}

void turnRight()
{
  analogWrite(LFM, 250);
  analogWrite(RBM, 130);
  analogWrite(LBM, 0);
  analogWrite(RFM, 0);
  
  delay(500);

  moveForward();
}

int lookRight(){
  setServoAngle(0, servoSensor);
  delay(500);
  int distance = getDistance();
  delay(100);
  setServoAngle(90, servoSensor);
  return distance;
}

int lookLeft(){
  setServoAngle(180, servoSensor);
  delay(500);
  int distance = getDistance();
  delay(100);
  setServoAngle(90, servoSensor);
  return distance;
}

// Function to set servo angle manually
void setServoAngle(int angle, int servoPin) {
  int pulseWidth = map(angle, 0, 180, minPulseWidth, maxPulseWidth);
  digitalWrite(servoPin, HIGH); // Start the pulse
  delayMicroseconds(pulseWidth); // Wait for the pulse width
  digitalWrite(servoPin, LOW); // End the pulse
  delay(20); // Delay for servo to settle
}


//Function returning distance to nearby object (required for the function object() to work)
int getDistance() 
{
  digitalWrite(trig, LOW);
  delayMicroseconds(5);

  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  duration = pulseIn(echo, HIGH);
  distance = duration * 0.034 / 2;

  return distance;
}

bool detectBlackLine() {
  // Read analog values from all 8 sensors
  int sensorValues[8] = {
    analogRead(sensorOne),
    analogRead(sensorTwo),
    analogRead(sensorThree),
    analogRead(sensorFour),
    analogRead(sensorFive),
    analogRead(sensorSix),
    analogRead(sensorSeven),
    analogRead(sensorEight)
  };

  // Set a threshold for black line detection (adjust as needed)
  int threshold = 800;

  // Check if any sensor reads a value below the threshold
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] < threshold) {
      return true; // Black line detected
    }
  }

  return false; // No black line detected
}
