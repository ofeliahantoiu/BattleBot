// LFM = Left Forward Motor A2
// LBM = Left Backward Motor A1
// RFM = Right Forward Motor B1
// RBM = Right Backward Motor B2

#define LFM 5
#define LBM 6
#define RFM 10
#define RBM 11

#define encoderRM 2 //encoder Right Motor
#define encoderLM 3 //encoder Left Motor
 
#define frontTrig 4 // Ultrasonic sensor trigger front
#define frontEcho 12 // Ultrasonic sensor echo front

#define leftTrig 8 // Ultrasonic sensor trigger front
#define leftEcho 7 // Ultrasonic sensor echo front
 
#define servoGrip 9 //servo used for the gripper

//ir1-ir6 - IR sensors starting from the left side
#define ir1 A5
#define ir2 A4
#define ir3 A3
#define ir4 A2
#define ir5 A1
#define ir6 A0

int irSensors[6] = {ir1, ir2, ir3, ir4, ir5, ir6};
boolean irValues[6];

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

  pinMode(servoGrip, OUTPUT);

  // this initialises the gripper
  for (int i = 0; i < 4; i++)
  {
    gripOpen();
  }

  gripOpen();
  
  pinMode(frontTrig, OUTPUT);
  pinMode(frontEcho, INPUT);

  pinMode(leftTrig, OUTPUT);
  pinMode(leftEcho, INPUT);

  pinMode(encoderRM, INPUT);
  pinMode(encoderLM, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderRM), updateRM, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderLM), updateLM, CHANGE);
}

void loop() 
{
  // the code for awaiting the start;
  // if the robot sees an object in front of it, it starts
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

    // the start itself;
    // the robot ought to move, pick up the stick,
    // turn left, and move forward
  if (startSequence)
  {
    wait(2000);

    moveForwardInTicks(100);
    wait(250);

    gripClose();
    wait(250);

    basicTurnLeft();
    wait(250);

    moveForwardInTicks(40);

    startSequence = false;

    return wait(250);
  }

  endDetected = allBlack();

  // end sequence
  if (endDetected)
  {
    moveStop();

    gripOpen();
    wait(150);
    
    moveBackwardInTicks(20);
    
    wait(150);
    gripClose();

    while (true)
        ;
  }

  // the main sequence

  querySensors();

  if (distanceLeft > 30 && distanceLeft < 300)
  {
    return turnLeft();
  }

  if (distanceLeft < 25 && distanceFront < 12)
  {
    return turnRight();
  }

  return moveForward();
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
  querySensors();

  if(distanceLeft > 9.2 && distanceLeft < 300)
  {
    analogWrite(RFM, 240);
    analogWrite(LFM, 255);
    digitalWrite(LBM, LOW);
    digitalWrite(RBM, LOW);
  }
  else if (distanceLeft < 7.2)
  {
    analogWrite(RFM, 220);
    analogWrite(LFM, 255);
    digitalWrite(LBM, LOW);
    digitalWrite(RBM, LOW);
  }
  else
  {
    analogWrite(RFM, 230);
    analogWrite(LFM, 255);
    digitalWrite(LBM, LOW);
    digitalWrite(RBM, LOW);
  }

  turnedRight = false;

  endDetected = allBlack();
}

void moveForwardInTicks(int ticks)
{
  resetCounters();

  while (countRM < ticks)
  {
    analogWrite(RFM, 225);
    analogWrite(LFM, 255);
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
    analogWrite(RBM, 225);
    analogWrite(LBM, 255);
    digitalWrite(LFM, LOW);
    digitalWrite(RFM, LOW);
  }

  turnedRight = false;
  endDetected = allBlack();

  moveStop();
  
  setServoAngle(2, servoGrip);
}

void turnRight()
{
  querySensors();

  moveStop();
  wait(150);

  if(distanceLeft < 10 || turnedRight)
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
    wait(150);
    moveForwardInTicks(30);
  }

  return wait(150);
}

void turnLeft()
{
  moveForwardInTicks(40);
  wait(350);

  basicTurnLeft();
  wait(155);

  querySensors();

  if (distanceFront > 25)
  {
    moveForwardInTicks(40);
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
    analogWrite(RFM, 250);
    analogWrite(LBM, 255);
    digitalWrite(RBM, LOW);
    digitalWrite(LFM, LOW);
  }

  moveStop();
}

void basicTurnRight()
{
  moveStop();
  resetCounters();

  while (countLM < 50)
  {
    analogWrite(LFM, 255);
    analogWrite(RBM, 230);
    digitalWrite(RFM, LOW);
    digitalWrite(LBM, LOW);
  }

  moveStop();
}

// while performing a right turn, the car might need
// to be adjusted to the wall
void adjustToWall()
{
  moveStop();
  resetCounters();

  while (countRM < 10)
  {
    analogWrite(RBM, 190);
  }

  moveStop();
  resetCounters();

  while (countRM < 12)
  {
    analogWrite(RBM, 225);
    analogWrite(LBM, 255);
  }

  moveStop();
  resetCounters();

  while (countRM < 12)
  {
    analogWrite(RFM, 190);
  }

  moveStop();
  resetCounters();

  while (countRM < 8)
  {
    analogWrite(RFM, 225);
    analogWrite(LFM, 255);
  }

  moveStop();
  resetCounters();
}

// sends the pulse;
// needs to be supplied with trigger and echo pins
float pulse(int proxTrig, int proxEcho)
{
    digitalWrite(proxTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(proxTrig, LOW);

    float duration_us = pulseIn(proxEcho, HIGH);

    return duration_us * .017;
}

// Function to set servo angle 
void setServoAngle(int angle, int servoPin)
{
  int pulseWidth = map(angle, 0, 180, minPulseWidth, maxPulseWidth);
  digitalWrite(servoPin, HIGH); // Start the pulse
  delayMicroseconds(pulseWidth); // Wait for the pulse width
  digitalWrite(servoPin, LOW); // End the pulse
  delay(20); // wait for servo to settle
}

// returns forward distance in cm
float getFrontDistance()
{
    return round(pulse(frontTrig, frontEcho) * 100.0) / 100.0;
}

// returns left distance in cm
float getLeftDistance()
{
    return round(pulse(leftTrig, leftEcho) * 100.0) / 100.0;
}

float querySensors()
{
  distanceLeft = getLeftDistance();
  distanceFront = getFrontDistance();
}

void queryIRSensors()
{
  //The function sets irValues[] to the actual values
  for (int i = 0; i < 6; i++)
  {
    irValues[i] = analogRead(irSensors[i]) > 800;
  }
}

boolean allBlack()
{
  short sum = 0;
  queryIRSensors();
  for (int i = 0; i < 6; i++)
  {
    if (irValues[i])
    {
      sum++;
    };
  }
  return sum == 6;
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

void gripOpen()
{
  setServoAngle(90, servoGrip);
}

void gripClose()
{
  setServoAngle(2, servoGrip);
}

// waits for an amount of time in milliseconds
// used to eliminate the need of using the delay() function
void wait(int timeToWait)
{
  long time = millis();

  while (millis() < time + timeToWait)
        ;
}