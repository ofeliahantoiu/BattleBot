// LFM = Left Forward Motor 
// LBM = Left Backward Motor
// RFM = Right Forward Motor
// RBM = Right Backward Motor

#define LFM 6
#define LBM 9
#define RFM 5
#define RBM 3

#define encoderRM 4 //encoder Right Wheel
#define encoderLM 7 //encoder Left Wheel
 
#define trig 12 // Ultrasonic sensor trigger 
#define echo 13 // Ultrasonic sensor echo

#define servoSensor 10 //servo used for the ultrasonic sensor 
#define servoGrip 2 //servo used for the gripper

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

int countRM = 0;
int countLM = 0;

void setup() 
{
  Serial.begin(9600);
  
  pinMode(LFM, OUTPUT);
  pinMode(LBM, OUTPUT);
  pinMode(RFM, OUTPUT);
  pinMode(RBM, OUTPUT);

  pinMode(servoSensor, OUTPUT);
  pinMode(servoGrip, OUTPUT);
  setServoAngle(90, servoGrip);

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  pinMode(encoderRM, INPUT);
  pinMode(encoderLM, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderRM), updateRM, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderLM), updateLM, CHANGE);
}

void loop() 
{
  
}

// while performing a right turn, the car might need
// to be adjusted to the wall
void adjustToWall()
{
  moveStop();
  resetCounters();

  while (countRM < 10)
  {
    analogWrite(RBM, 255);
  }

  moveStop();

  resetCounters();
  while (countRM < 12)
  {
    analogWrite(RBM, 235);
    analogWrite(LBM, 255);
  }

    moveStop();

    resetCounters();
    while (countRM < 12)
    {
        analogWrite(RFM, 255);
    }

    moveStop();

    resetCounters();
    while (countRM < 8)
    {
        analogWrite(RFM, 255);
        analogWrite(LFM, 255);
    }

    moveStop();

    resetCounters();
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
    analogWrite(RFM, 255);
    analogWrite(LFM, 230);
  }
  else if (distanceLeft < 7.2)
  {
    analogWrite(RFM, 225);
    analogWrite(LFM, 255);
  }
  else
  {
    analogWrite(RFM, 255);
    analogWrite(LFM, 232);
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
    analogWrite(RFM, 245);
    digitalWrite(LBM, LOW);
    digitalWrite(RBM, LOW);
  }

  turnedRight = false;
  endDetected = allBlack();

  moveStop();
  
  setServoAngle(0, servoGrip);
}

void moveBackwardInTicks(int ticks)
{
  resetCounters();

  while (countRM < ticks)
  {
    digitalWrite(LBM, HIGH);
    digitalWrite(RBM, HIGH);
    digitalWrite(LFM, LOW);
    digitalWrite(RFM, LOW);
  }

  turnedRight = false;
  endDetected = allBlack();

  moveStop();
  
  setServoAngle(0, servoGrip);
  
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

  while (countRM < 17)
  {
    analogWrite(RFM, 255);
    analogWrite(LBM, 255);
  }

  moveStop();
}

void basicTurnRight()
{
  moveStop();
  resetCounters();

  while (countLM < 17)
  {
    analogWrite(LFM, 255);
    analogWrite(RBM, 255);
  }

  moveStop();
}

//Function to calculate distance in cm
float getDistance()
{
  int distance, duration;

  digitalWrite(trig, LOW);
  delayMicroseconds(2);

  digitalWrite(trig, HIGH);
  delayMicroseconds(10);

  digitalWrite(trig, LOW);

  duration = pulseIn(echo, HIGH);
  distance= duration*0.034/2;
  return distance;
}

/*int lookRight()
{
  setServoAngle(0, servoSensor);
  wait(500);
  int distance = getDistance();
  wait(100);
  setServoAngle(90, servoSensor);
  return distance;
}*/

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
  setServoAngle(90, servoSensor);
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
  distanceFront = lookFront();
  distanceLeft = lookLeft();
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

// waits for an amount of time in milliseconds
// used to eliminate the need of using the delay() function
void wait(int timeToWait)
{
  long time = millis();

  while (millis() < time + timeToWait)
        ;
}