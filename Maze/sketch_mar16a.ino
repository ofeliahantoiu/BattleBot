// LFM = Left Forward Motor A2
// LBM = Left Backward Motor A1
// RFM = Right Forward Motor B1
// RBM = Right Backward Motor B2

#include <Adafruit_NeoPixel.h>
// NeoPixel colors
#define COLOR_RED   pixels.Color(255, 0, 0)
#define COLOR_GREEN pixels.Color(0, 255, 0)
#define COLOR_BLUE  pixels.Color(0, 0, 255)
#define COLOR_PURPLE pixels.Color(255, 0, 255)

void setColor(uint32_t color);

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

#define PIN 13          // Set the pin for NeoPixel data input
#define NUM_PIXELS 4

// Create an instance of the Adafruit_NeoPixel class
Adafruit_NeoPixel pixels(NUM_PIXELS, PIN, NEO_RGB + NEO_KHZ800);

//ir1-ir6 - IR sensors starting from the left side
#define ir1 A5
#define ir2 A4
#define ir3 A3
#define ir4 A2
#define ir5 A1
#define ir6 A0
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

  pinMode(servoGrip, OUTPUT);

  // this initialises the gripper
  for (int i = 0; i < 4; i++)
  {
    gripOpen();
  }

  gripOpen();

  // Initialize the Neopixel strip
  pixels.begin();
  pixels.show();
  
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
    if (distanceFront < 20)
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
    setColor(COLOR_PURPLE);
    wait(200);

    moveForwardInTicks(30);
    wait(150);

    gripClose();
    wait(150);

    basicTurnLeft();
    basicTurnLeft();
    basicTurnLeft();
    wait(150);

    adjust();   
    moveForwardInTicks(60);

    startSequence = false;

    return wait(250);
  }

  endDetected = allBlack();

  // end sequence
  if (endDetected)
  {
    moveStop();

    gripOpen();
    wait(100);
    
    moveBackwardInTicks(4);
    
    wait(150);
    gripClose();

    while (true)
        ;
  }

  // the main sequence
  setColor(COLOR_PURPLE);
  
  querySensors();

//  if (distanceLeft > 30 && distanceLeft < 300)
//  {
//    wait(100);
//    return turnLeft();
//  }
//
//  else if (distanceLeft < 15 && distanceFront < 15)
//  {
//    wait(100);
//    return turnRight();
//  }

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
  setColor(COLOR_PURPLE);
  querySensors();
  if (distanceLeft > 25  && distanceFront > 15)
  {
//    moveStop();
    resetCounters();
    
    moveForwardInTicks(12);
    wait(50);

    basicTurnLeft();
    wait(50);
    adjust();
    wait(50);
//    analogWrite(RFM, 230);
//    analogWrite(LFM, 255);
//    analogWrite(LBM, 0);
//    analogWrite(RBM, 0);
moveForward();
  }
                                      
//  else if(distanceLeft > 35  && distanceFront < 15)
//  {
//    moveStop();
//    resetCounters();
//    
//
//    basicTurnLeft();
////    adjust();
//    wait(100); 
//    moveStop();
////    analogWrite(RFM, 230);
////    analogWrite(LFM, 255);
////    analogWrite(LBM, 0);
////    analogWrite(RBM, 0);    
//moveForward();                                                                                            
//  }

  else if (distanceFront < 12) {
//    moveStop();
    resetCounters();
    
    analogWrite(RFM, 200);
    analogWrite(LBM, 120);
    analogWrite(LFM, 0);
    analogWrite(RBM, 0);
    
    wait(50); 
//    moveStop();
    if (distanceLeft < 6)
    {
      wait(50);
      moveBackwardInTicks(5);
      wait(50);
      basicTurnRight(); 
      wait(50);
      adjust();
      wait(50);
      moveForward();
    } else if (distanceLeft > 30)
    {
      resetCounters();
    
//    moveForwardInTicks(12);
    wait(50);

    basicTurnLeft();
    wait(50);
    adjust();
    wait(50);
    moveForward();
    }
    else {
      
    wait(50); 
    
    analogWrite(RFM, 200);
    analogWrite(LBM, 120);
    analogWrite(LFM, 0);
    analogWrite(RBM, 0);
    
    wait(50); 
//    moveStop();
    moveBackwardInTicks(5);
    wait(50);
      basicTurnRight();
      wait(50);
      adjust();
      wait(50);
      moveForward();
    }
//    basicTurnRight();
//    wait(100);
//    moveStop();
moveForward();
  }

  else if (distanceLeft > 100) {
    resetCounters();
    wait(50);
    moveBackwardInTicks(5);
      wait(50);
      basicTurnRight(); 
      adjust();
      wait(50);
      moveForward();
  }

  else
  {
//    if(distanceLeft > 10 && distanceLeft < 30)
//  {
//    moveStop();
//    resetCounters();
//    wait(50); 
//    
//    analogWrite(RFM, 200);
//    analogWrite(LBM, 120);
//    analogWrite(LFM, 0);
//    analogWrite(RBM, 0);
//    
//    wait(50); 
////    moveStop();
////
////    analogWrite(RFM, 230);
////    analogWrite(LFM, 255);
////    analogWrite(LBM, 0);
////    analogWrite(RBM, 0);
//moveForward();
//
//  }
//  else if (distanceLeft < 6)
//  {
////    moveStop();
//    resetCounters();
//    wait(50); 
//  
//    analogWrite(LFM, 200);
//    analogWrite(RBM, 120);
//    analogWrite(LBM, 0);
//    analogWrite(RFM, 0);
//  
//    wait(50); 
////    moveStop();
//
////    analogWrite(RFM, 230);
////    analogWrite(LFM, 255);
////    analogWrite(LBM, 0);
////    analogWrite(RBM, 0);
//moveForward();
//  }
    analogWrite(RFM, 230);
    analogWrite(LFM, 255);
    analogWrite(LBM, 0);
    analogWrite(RBM, 0);
  }
  

  turnedRight = false;

  endDetected = allBlack();
}

void adjust()
{
  if(distanceLeft > 10 && distanceLeft < 30)
  {
    moveStop();
    resetCounters();
    wait(100); 
    
    analogWrite(RFM, 200);
    analogWrite(LBM, 120);
    analogWrite(LFM, 0);
    analogWrite(RBM, 0);
    
    wait(100); 
    moveStop();

    analogWrite(RFM, 230);
    analogWrite(LFM, 255);
    analogWrite(LBM, 0);
    analogWrite(RBM, 0);

  }
  else if (distanceLeft < 6)
  {
    moveStop();
    resetCounters();
    wait(100); 
  
    analogWrite(LFM, 220);
    analogWrite(RBM, 189);
    analogWrite(LBM, 0);
    analogWrite(RFM, 0);
  
    wait(100); 
    moveStop();

    analogWrite(RFM, 230);
    analogWrite(LFM, 255);
    analogWrite(LBM, 0);
    analogWrite(RBM, 0);
  }
}

void moveForwardInTicks(int ticks)
{
  resetCounters();

  while (countLM < ticks)
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

  while (countLM < ticks)
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
  setColor(COLOR_BLUE);
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

  if (distanceFront > 25)
  {
    wait(150);
    moveForward();
  }

  return wait(150);
}

void turnLeft()
{
  setColor(COLOR_GREEN);
  moveForwardInTicks(4);
  wait(250);

  basicTurnLeft();
  wait(150);

  querySensors();

  if (distanceFront > 25)
  {
    wait(150);
    moveForward();
  }

  turnedRight = false;

  return wait(150);
}

void basicTurnLeft()
{
  setColor(COLOR_GREEN);
  moveStop();
  resetCounters();

  while (countRM < 11)
  {
    analogWrite(RFM, 250);
    analogWrite(LBM, 130);
    analogWrite(LFM, 0);
    analogWrite(RBM, 0);
  }

  moveStop();
}

void basicTurnRight()
{
  setColor(COLOR_BLUE);
  moveStop();
  resetCounters();

  while (countLM < 11)
  {
    analogWrite(LFM, 250);
    analogWrite(RBM, 130);
    analogWrite(LBM, 0);
    analogWrite(RFM, 0);
  }

  moveStop();
}

// while performing a right turn, the car might need
// to be adjusted to the wall
void adjustToWall()
{
  moveStop();
  resetCounters();

  while (countLM < 2)
  {
    analogWrite(RBM, 190);
  }

  moveStop();
  resetCounters();

  while (countLM < 3)
  {
    analogWrite(RBM, 225);
    analogWrite(LBM, 255);
  }

  moveStop();
  resetCounters();

  while (countLM < 3)
  {
    analogWrite(RFM, 190);
  }

  moveStop();
  resetCounters();

  while (countLM < 2)
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

// Function to set the color of the NeoPixels
void setColor(uint32_t color) {
  for (int i = 0; i < NUM_PIXELS; i++) {
    pixels.setPixelColor(i, color);
  }
  pixels.show();
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
  Serial.print("LM");
  Serial.print(countLM);
}

void updateRM() {

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
  Serial.print("RM");
  Serial.print(countRM);

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
