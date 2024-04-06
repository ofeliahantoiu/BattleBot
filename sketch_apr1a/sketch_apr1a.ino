  // LFM = Left Forward Motor A2
// LBM = Left Backward Motor A1
// RFM = Right Forward Motor B1
// RBM = Right Backward Motor B2

#include <Adafruit_NeoPixel.h>

#define LFM 5
#define LBM 6
#define RFM 10
#define RBM 11

#define encoderRM 2 //encoder Right Motor
#define encoderLM 3 //encoder Left Motor
#define COUNTER_INTERVAL 7 
 
#define frontTrig 4 // Ultrasonic sensor trigger front
#define frontEcho 12 // Ultrasonic sensor echo front

#define leftTrig 8 // Ultrasonic sensor trigger front
#define leftEcho 7 // Ultrasonic sensor echo front
 
#define servoGrip 9 //servo used for the gripper

#define ir0 A0
#define ir1 A1
#define ir2 A2
#define ir3 A3
#define ir4 A4
#define ir5 A5
#define ir6 A6 
#define ir7 A7

int irSensors[8] = {ir0, ir1, ir2, ir3, ir4, ir5, ir6, ir7};
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

// Define the number of NeoPixel LEDs and the pin they are connected to
#define NUM_PIXELS 4
#define NEOPIN 13

// Define RGB colors using Adafruit_NeoPixel library
#define RED pixels.Color(255, 0, 0)
#define GREEN pixels.Color(0, 255, 0)
#define BLUE pixels.Color(0, 0, 255)
#define YELLOW pixels.Color(255, 255, 0)
#define WHITE pixels.Color(255, 255, 255)
#define OFF pixels.Color(0, 0, 0)

int colorValues[] = {0, 0, 0, 0, 0, 0};

// Initialize NeoPixel object with defined number of pixels and pin
Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIN, NEO_RGB + NEO_KHZ800);

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

  attachInterrupt(digitalPinToInterrupt(encoderLM), updateLM, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRM), updateRM, CHANGE);

  pixels.begin();

  lightsOff();
}

void loop() 
{
  // the code for awaiting the start;
  // if the robot sees an object in front of it, it starts
  if (waitingStart)
  {
    startLights(4);
    querySensors();

    if (distanceFront < 23)
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
    wait(1300);

    moveForwardInTicks(50);
    wait(250);

    gripClose();
    wait(250);

    basicTurnLeft();
    wait(250);

    moveForwardInTicks(33);

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
    
    moveBackwardInTicks(13);
    
    wait(150);
    gripClose();

    while (true)
        ;
  }

  // the main sequence

  querySensors();

  if (distanceLeft > 30 && distanceLeft < 100)
  {
    return turnLeft();
  }

  if (distanceLeft < 25 && distanceFront < 12)
  {
    return turnRight();
  }

  if (distanceFront < 11)
  {
    return moveBackwardInTicks(10);
  }

  return moveForward();
}

void moveStop()
{
  resetCounters();
  stopLights();

  analogWrite(RFM, 0);
  analogWrite(LFM, 0);
  analogWrite(RBM, 0);
  analogWrite(LBM, 0);
}

// the function defines the behaviour of the car when it is going forward
// it adjusts the car so that it is constantly around 8.2 cm away from the wall
void moveForward()
{
  resetCounters();
  forwardLights();
  querySensors();

  if(distanceLeft > 9.2 && distanceLeft < 13)
  {
    forwardLights();
    analogWrite(RFM, 250);
    analogWrite(LFM, 255);
  }
  else if (distanceLeft < 7.2 || distanceLeft > 100)
  {
    forwardLights();
    analogWrite(RFM, 254);
    analogWrite(LFM, 255);
  }
  else
  {
    forwardLights();
    analogWrite(RFM, 252);
    analogWrite(LFM, 255);
  }

  turnedRight = false;

  endDetected = allBlack();
}

void moveForwardInTicks(int ticks)
{
  forwardLights();
  resetCounters();

  while (countLM < ticks)
  {
    forwardLights();
    analogWrite(RFM, 252);
    analogWrite(LFM, 255);
  }

  turnedRight = false;
  endDetected = allBlack();

  moveStop();
}

void moveBackwardInTicks(int ticks)
{
  resetCounters();

  while (countLM < ticks)
  {
    analogWrite(RBM, 252);
    analogWrite(LBM, 255);
  }

  turnedRight = false;
  endDetected = allBlack();

  moveStop();
  
  setServoAngle(2, servoGrip);
}

void turnRight()
{
  resetCounters();
  turnLights();

  querySensors();
  moveStop();
  wait(150);

  if(distanceLeft < 7 || turnedRight)
  {
    turnLights();
    basicTurnRight();
    turnedRight = true;
  }
  else
  {
    turnLights();
    adjustToWall();
  }

  wait(150);

  querySensors();

  if (distanceFront > 15)
  {
    wait(100);
    moveForwardInTicks(20);
  }

  return wait(150);
}

void turnLeft()
{
  resetCounters();
  leftLights();

  moveForwardInTicks(13);
  wait(350);

  basicTurnLeft();
  wait(150);

  querySensors();

  if (distanceFront > 25)
  {
    moveForwardInTicks(13);
  }

  turnedRight = false;

  return wait(150);
}

void basicTurnLeft()
{
  leftLights();
  moveStop();
  resetCounters();

  while (countLM < 12)
  {
    leftLights();
    analogWrite(RFM, 237);
    analogWrite(LBM, 240);
  }

  moveStop();
}

void basicTurnRight()
{
  turnLights();
  moveStop();
  resetCounters();

  while (countLM < 12)
  {
    turnLights();
    analogWrite(RBM, 237);
    analogWrite(LFM, 240);
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
    analogWrite(RBM, 220);
  }

  moveStop();
  resetCounters();

  while (countLM < 8)
  {
    analogWrite(RBM, 240);
    analogWrite(LBM, 255);
  }

  moveStop();
  resetCounters();

  while (countRM < 11)
  {
    analogWrite(RFM, 220);
  }

  moveStop();
  resetCounters();

  while (countLM < 9)
  {
    analogWrite(RFM, 240);
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

void updateLM()
{
  static unsigned long timer;

  static bool lastState;

  noInterrupts();

  if (millis() > timer) {

    bool state = digitalRead(encoderLM);

    if (state != lastState) {

       countLM++;

       lastState = state;

    }

    timer = millis() + COUNTER_INTERVAL;
  } 

  interrupts();
}

void updateRM() 
{
  static unsigned long timer;

  static bool lastState;

  noInterrupts();

  if (millis() > timer) {

    bool state = digitalRead(encoderRM);

    if (state != lastState) {

       countRM++;

       lastState = state;

    }

    timer = millis() + COUNTER_INTERVAL;

  }

  interrupts();
}

void resetCounters()
{
  countRM = 0;
  countLM = 0;
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

// Function to set specific LEDs to represent left movement
void leftLights()
{
  pixels.setPixelColor(0, WHITE); 
  pixels.setPixelColor(1, YELLOW); 
  pixels.setPixelColor(2, YELLOW);  
  pixels.setPixelColor(3, WHITE);  
  pixels.show(); // Update LEDs with new colors
}

// Function to set all LEDs to represent turning movement
void turnLights()
{
  pixels.setPixelColor(0, YELLOW); 
  pixels.setPixelColor(1, WHITE); 
  pixels.setPixelColor(2, WHITE);
  pixels.setPixelColor(3, YELLOW); 
  pixels.show(); // Update LEDs with new colors
}

// Function to set all LEDs to represent stopped movement
void stopLights()
{
  pixels.setPixelColor(0, RED); 
  pixels.setPixelColor(1, RED); 
  pixels.setPixelColor(2, RED); 
  pixels.setPixelColor(3, RED); 
  pixels.show(); // Update LEDs with new colors
}

// Function to set specific LED to represent starting movement
void startLights(int lightNR)
{
  pixels.setPixelColor(lightNR, BLUE); 
  pixels.show(); // Update LEDs with new colors
}

// Function to set all LEDs to represent forward movement
void forwardLights()
{
  pixels.setPixelColor(0, GREEN); 
  pixels.setPixelColor(1, GREEN); 
  pixels.setPixelColor(2, GREEN); 
  pixels.setPixelColor(3, GREEN); 
  pixels.show(); // Update LEDs with new colors
}

// Function to set all LEDs to represent normal state
void normalLights() 
{
  pixels.setPixelColor(0, WHITE); 
  pixels.setPixelColor(1, WHITE); 
  pixels.setPixelColor(2, WHITE); 
  pixels.setPixelColor(3, WHITE); 
  pixels.show(); // Update LEDs with new colors
}

// Function to set all LEDs to be turned off
void lightsOff()
{
  pixels.setPixelColor(0, OFF); 
  pixels.setPixelColor(1, OFF); 
  pixels.setPixelColor(2, OFF);
  pixels.setPixelColor(3, OFF); 
  pixels.show(); // Update LEDs with new colors
}