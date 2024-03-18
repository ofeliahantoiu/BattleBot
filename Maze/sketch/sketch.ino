// LFM = Left Forward Motor A2
// LBM = Left Backward Motor A1
// RFM = Right Forward Motor B1
// RBM = Right Backward Motor B2
#include <Servo.h>

Servo sensorServo;
Servo gripServo;

bool isGripping = false;

#define LFM 5
#define LBM 6
#define RFM 10
#define RBM 11

#define encoderRM 2 //encoder Right Motor
#define encoderLM 3 //encoder Left Motor
 
#define trig 13 // Ultrasonic sensor trigger 
#define echo 12 // Ultrasonic sensor echo

#define servoSensor 9 //servo used for the ultrasonic sensor 
#define servoGrip 7 //servo used for the gripper

//ir1-ir6 - IR sensors starting from the left side
#define ir1 A0
#define ir2 A1
#define ir3 A2
#define ir4 A3
#define ir5 A4
#define ir6 A5
#define ir7 A6
#define ir8 A7

int irSensors[8] = {ir1, ir2, ir3, ir4, ir5, ir6, ir7, ir8};
boolean irValues[8];

const int minPulseWidth = 500; // Minimum pulse width for servo
const int maxPulseWidth = 2500; // Maximum pulse width for servo


float distanceFront, distanceLeft;

bool waitingStart = true;
bool startSequence = false;
bool endDetected = false;

bool turnedRight = false;

volatile int countRM = 0;
volatile int countLM = 0;

void setup() 
{
  Serial.begin(9600);
  
  pinMode(LFM, OUTPUT);
  pinMode(LBM, OUTPUT);
  pinMode(RFM, OUTPUT);
  pinMode(RBM, OUTPUT);

  pinMode(servoSensor, OUTPUT);
  pinMode(servoGrip, OUTPUT);
  setServoAngle(95, servoGrip);
  setServoAngle(84, servoSensor);
  
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  pinMode(encoderRM, INPUT);
  pinMode(encoderLM, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderRM), updateRM, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderLM), updateLM, CHANGE);
}

enum CarState 
{
  STATE_FORWARD,
  STATE_OBSTACLE_DETECTED,
  STATE_TURN_RIGHT,
  STATE_TURN_LEFT,
  STATE_STOP,
  STATE_WAIT
};

CarState carState = STATE_FORWARD;

void loop() 
{
  if (waitingStart)
  {
    querySensors();
    if (distanceFront < 25)
    {
        waitingStart = false;
        startSequence = true;
    }

    return wait(100);
  }

  querySensors();
  //Debug for ultrasonic sensor
  Serial.print("Front: ");
  Serial.print(distanceFront);
  Serial.print(" | Left: ");
  Serial.println(distanceLeft);

  if(distanceLeft > 30)
  {
    wait(350);
    basicTurnLeft();
    wait(350);
  }

  if(distanceLeft < 25 && distanceFront < 12)
  {
    wait(350);
    basicTurnRight();
    wait(350);
  }

  return moveForward();

/*  //Control the performance of the car
  switch (carState) 
  { 
    case STATE_FORWARD:
      if (distanceFront > 12) 
      {
        moveForwardInTicks(60);
      } 
      else 
      {
        carState = STATE_OBSTACLE_DETECTED;
      }
      break;

    case STATE_OBSTACLE_DETECTED:
      moveBackwardInTicks(20); // Move backward for a short distance
      if (distanceLeft > 15) 
      {
        carState = STATE_TURN_LEFT;
      } 
      else
      {
        carState = STATE_TURN_RIGHT; // Move to the turn right state
      }
      break;

    case STATE_TURN_LEFT:
      wait(350);
      basicTurnLeft();
      carState = STATE_STOP; 
      break;

    case STATE_TURN_RIGHT:
      wait(350);
      moveForwardInTicks(20);
      basicTurnRight(); // Turn right
      carState = STATE_STOP; // Move to the stop state after turning
      break;

    case STATE_STOP:
      moveStop();
      carState = STATE_WAIT; // Move to the wait state
      wait(150);
      break;

    case STATE_WAIT:
      carState = STATE_FORWARD; // Move back to the forward state for the next iteration
      break;

    default:
      moveForward();
  }*/
}

// while performing a right turn, the car might need
// to be adjusted to the wall
void adjustToWall()
{
  moveStop();
  resetCounters();

  while (countRM < 10)
  {
    analogWrite(RBM, 250);
  }

  moveStop();
  resetCounters();

  while (countRM < 12)
  {
    analogWrite(RBM, 250);
    analogWrite(LBM, 255);
  }

  moveStop();
  resetCounters();

  while (countRM < 12)
  {
      analogWrite(RFM, 230);
  }

  moveStop();
  resetCounters();

  while (countRM < 8)
  {
      analogWrite(RFM, 228);
      analogWrite(LFM, 255);
  }

  moveStop();
  resetCounters();
}

bool detectBlackLine() 
{
  // Read analog values from all 8 sensors
  int sensorValues[8] = 
  {
    analogRead(ir1),
    analogRead(ir2),
    analogRead(ir3),
    analogRead(ir4),
    analogRead(ir5),
    analogRead(ir6)
  };

  // Set a threshold for black line detection
  int threshold = 800;

  // Check if any sensor reads a value below the threshold
  for (int i = 0; i < 8; i++) 
  {
    if (sensorValues[i] < threshold) 
    {
      return true; // Black line detected
    }
  }

  return false; // No black line detected
}

void moveStop()
{
  digitalWrite(RFM, LOW);
  digitalWrite(LFM, LOW);
  digitalWrite(RBM, LOW);
  digitalWrite(LBM, LOW);
}

// the function defines the behaviour of the car when it is going forward
// it adjusts the car so that it is constantly around 8.2 cm away from the wall
void moveForward()
{
  if (distanceLeft > 9.2)
  {
    analogWrite(RFM, 232);
    analogWrite(LFM, 255);
  }
  else if (distanceLeft < 7.2)
  {
    analogWrite(RFM, 228);
    analogWrite(LFM, 255);
  }
  else
  {
    analogWrite(RFM, 229);
    analogWrite(LFM, 255);
  }
  turnedRight = false;

  endDetected = allBlack();
}

void moveForwardInTicks(int ticks)
{
  resetCounters();

  while (countRM < ticks)
  {
    analogWrite(LFM, 255);
    analogWrite(RFM, 230);
    digitalWrite(LBM, LOW);
    digitalWrite(RBM, LOW);
  }

  turnedRight = false;
  endDetected = allBlack();

  moveStop();
}

void moveBackwardInTicks(int ticks)
{
  resetCounters();

  while (countRM < ticks)
  {
    analogWrite(LBM, 255);
    analogWrite(RBM, 225);
    digitalWrite(LFM, LOW);
    digitalWrite(RFM, LOW);
  }

  turnedRight = false;
  endDetected = allBlack();

  moveStop();
  
  setServoAngle(3, servoGrip);
  
}

void turnRight()
{
  moveStop();
  wait(150);

  if(distanceLeft < 6 || turnedRight)
  {
    basicTurnRight();
    turnedRight = true;
  }
  else
  {
    adjustToWall();
  }

  wait(150);

  querySensors();

  if (distanceFront > 15)
  {
    wait(100);
    moveForwardInTicks(25);
  }

  return wait(150);
}

void turnLeft()
{
  moveForwardInTicks(20);
  wait(300);

  basicTurnLeft();
  wait(150);

  querySensors();

  if (distanceFront > 25)
  {
    moveForwardInTicks(20);
  }

  turnedRight = false;

  return wait(150);
}

void basicTurnLeft()
{
  moveStop();
  resetCounters();

  while (countRM < 60)
  {
    analogWrite(LBM, 250);
    analogWrite(RFM, 230);
    analogWrite(LFM, 0);
    analogWrite(RBM, 0);
  }
  moveStop();
}

void basicTurnRight()
{
  moveStop();
  resetCounters();

  while (countLM < 85)
  {
    analogWrite(LFM, 255);
    analogWrite(RBM, 255);
    analogWrite(LBM, 0);
    analogWrite(RFM, 0);
  }
  moveStop();
}

float pulse(int proxTrig, int proxEcho)
// sends the pulse;
// needs to be supplied with trigger and echo pins
{
    digitalWrite(proxTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(proxTrig, LOW);

    float duration_us = pulseIn(proxEcho, HIGH);

    return duration_us * .017;
}

float lookFront()
{
  setServoAngle(84, servoSensor);
  wait(500);
  return round(pulse(trig, echo) * 100.0) / 100.0;
}

float lookLeft()
{
  setServoAngle(180, servoSensor);
  wait(500);
  return round(pulse(trig, echo) * 100.0) / 100.0;
}

// Function to set servo angle manually
void setServoAngle(int angle, int servoPin)
{
  int pulseWidth = map(angle, 0, 180, minPulseWidth, maxPulseWidth);
  digitalWrite(servoPin, HIGH); // Start the pulse
  delayMicroseconds(pulseWidth); // Wait for the pulse width
  digitalWrite(servoPin, LOW); // End the pulse
  delay(20); // wait for servo to settle
}

float querySensors()
{
  distanceLeft = lookLeft();
  distanceFront = lookFront();
}

void queryIRSensors()
{
  //The function sets irValues[] to the actual values
  for (int i = 0; i < 8; i++)
  {
    irValues[i] = analogRead(irSensors[i]) > 800;
  }
}

boolean allBlack()
{
  short sum = 0;
  queryIRSensors();
  for (int i = 0; i < 8; i++)
  {
    if (irValues[i])
    {
      sum++;
    }
  }
  return sum == 8;
}

void resetCounters()
{
  countRM = 0;
  countLM = 0;
}

void updateRM()
{
  noInterrupts();
  countRM++;
  interrupts();
}

void updateLM()
{
  noInterrupts();
  countLM++;
  interrupts();
}

// waits for an amount of time in milliseconds
// used to eliminate the need of using the delay() function
void wait(int timeToWait)
{
  long time = millis();

  while (millis() < time + timeToWait)
        ;
}
