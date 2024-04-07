// Include the Adafruit NeoPixel library
#include <Adafruit_NeoPixel.h>

// Define pin mappings for motors
#define LFM 5   // Left Forward Motor A2
#define LBM 6   // Left Backward Motor A1
#define RFM 10  // Right Forward Motor B1
#define RBM 11  // Right Backward Motor B2

// Define pins for encoders
#define encoderRM 2     // Encoder Right Motor
#define encoderLM 3     // Encoder Left Motor
#define COUNTER_INTERVAL 7  // Interval for encoder counter update

// Define pins for ultrasonic sensors
#define frontTrig 4     // Ultrasonic sensor trigger front
#define frontEcho 12    // Ultrasonic sensor echo front
#define leftTrig 8      // Ultrasonic sensor trigger left
#define leftEcho 7      // Ultrasonic sensor echo left

// Define pin for servo used for the gripper
#define servoGrip 9     

// Define analog pins for IR sensors
#define ir0 A0
#define ir1 A1
#define ir2 A2
#define ir3 A3
#define ir4 A4
#define ir5 A5
#define ir6 A6 
#define ir7 A7

// Array to hold IR sensor pins
int irSensors[8] = {ir0, ir1, ir2, ir3, ir4, ir5, ir6, ir7};
// Array to store IR sensor readings
boolean irValues[8];

// Define servo pulse width limits
const int minPulseWidth = 500;   // Minimum pulse width for servo
const int maxPulseWidth = 2500;  // Maximum pulse width for servo

// Variables to store ultrasonic sensor readings
float distanceFront, distanceLeft;

// Flags for different states of the robot
bool waitingStart = true;
bool startSequence = false;
bool endDetected = false;

// Flag to track if the robot turned right recently
bool turnedRight = false;

// Variables to count encoder ticks
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

// Array to hold color values for NeoPixel LEDs
int colorValues[] = {0, 0, 0, 0, 0, 0};

// Initialize NeoPixel object with defined number of pixels and pin
Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIN, NEO_RGB + NEO_KHZ800);

// Setup function
void setup() 
{
  Serial.begin(9600);
  
  // Set motor pins as outputs
  pinMode(LFM, OUTPUT);
  pinMode(LBM, OUTPUT);
  pinMode(RFM, OUTPUT);
  pinMode(RBM, OUTPUT);

  // Set gripper servo pin as output
  pinMode(servoGrip, OUTPUT);

  // this initialises the gripper
  for (int i = 0; i < 4; i++)
  {
    gripOpen();
  }
  
  gripOpen();

  // Initialize ultrasonic sensors pins
  pinMode(frontTrig, OUTPUT);
  pinMode(frontEcho, INPUT);
  pinMode(leftTrig, OUTPUT);
  pinMode(leftEcho, INPUT);

  // Initialize encoder pins
  pinMode(encoderRM, INPUT);
  pinMode(encoderLM, INPUT);

  // Attach interrupts for encoder pins
  attachInterrupt(digitalPinToInterrupt(encoderLM), updateLM, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRM), updateRM, CHANGE);

  // Initialize NeoPixel LEDs
  pixels.begin();

  // Turn off all NeoPixel LEDs
  lightsOff();
}

// Main loop function
void loop() 
{
  // Code for awaiting the start;
  // If the robot sees an object in front of it, it starts
  if (waitingStart)
  {
    startLights(4); 
    querySensors(); 

    if (distanceFront < 23) // If an object is detected in front
    {
      waitingStart = false; // Move to start sequence
      startSequence = true;
    }

    return wait(100); 
  }

  // Start sequence:
  // The robot moves forward, picks up the stick, turns left, and moves forward
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

  // End sequence:
  // The robot stops, opens the gripper, moves backward and closes the gripper
  if (endDetected)
  {
    moveStop(); 
    gripOpen(); 
    wait(150); 
    
    moveBackwardInTicks(13); 
    
    wait(150); 
    gripClose(); 

    while (true); // End loop
  }

  // Main sequence:
  // The robot checks sensor readings and performs actions accordingly
  querySensors(); 

  if (distanceLeft > 30 && distanceLeft < 100) // If distance to left is within range
  {
    return turnLeft(); 
  }

  if (distanceLeft < 25 && distanceFront < 12) // If close to an obstacle on left and front
  {
    return turnRight(); 
  }

  if (distanceFront < 11) // If obstacle detected in front
  {
    return moveBackwardInTicks(10); 
  }

  return moveForward(); 
}

// Function to stop all motor movement
void moveStop()
{
  resetCounters(); 
  stopLights(); 

  analogWrite(RFM, 0);
  analogWrite(LFM, 0);
  analogWrite(RBM, 0);
  analogWrite(LBM, 0);
}

// Function to move forward while maintaining distance from left wall
void moveForward()
{
  resetCounters(); 
  forwardLights(); 
  querySensors(); 

  if(distanceLeft > 9.2 && distanceLeft < 13) // If too far from left wall
  {
    forwardLights(); 
    analogWrite(RFM, 250); 
    analogWrite(LFM, 255); 
  }
  else if (distanceLeft < 7.2 || distanceLeft > 100) // If too close from left wall
  {
    forwardLights(); 
    analogWrite(RFM, 254); 
    analogWrite(LFM, 255); 
  }
  else // If in an acceptable range from left wall
  {
    forwardLights(); 
    analogWrite(RFM, 252); 
    analogWrite(LFM, 255); 
  }

  turnedRight = false; 
  endDetected = allBlack(); 
}

// Function to move forward for a specified number of encoder ticks
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

  turnedRight = false; // Reset turn flag
  endDetected = allBlack(); // Check if all IR sensors detect black

  moveStop(); 
}

// Function to move backward for a specified number of encoder ticks
void moveBackwardInTicks(int ticks)
{
  resetCounters(); 

  while (countLM < ticks) 
  {
    analogWrite(RBM, 252); 
    analogWrite(LBM, 255); 
  }

  turnedRight = false; // Reset turn flag
  endDetected = allBlack(); // Check if all IR sensors detect black

  moveStop(); 
  
  gripClose();
}

// Function to turn right
void turnRight()
{
  resetCounters(); 
  turnLights(); 

  querySensors(); 
  moveStop(); 
  wait(150); 

  if(distanceLeft < 7 || turnedRight) // If too close to left wall or recently turned right
  {
    turnLights(); 
    basicTurnRight(); 
    turnedRight = true; // Set turn flag
  }
  else // If adjusting to left wall is needed
  {
    turnLights(); 
    adjustToWall(); 
  }

  wait(150); 

  querySensors(); 

  if (distanceFront > 15) // If enough space in front
  {
    wait(100);
    moveForwardInTicks(20); 
  }

  return wait(150); 
}

// Function to turn left
void turnLeft()
{
  resetCounters(); 
  leftLights();

  moveForwardInTicks(13); 
  wait(350); 

  basicTurnLeft(); 
  wait(150); 

  querySensors(); 

  if (distanceFront > 25) // If enough space in front
  {
    moveForwardInTicks(13); // Move forward
  }

  turnedRight = false; // Reset turn flag

  return wait(150); 
}

// Function to perform basic left turn
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

// Function to perform basic right turn
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

// Function to adjust to left wall while turning right
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

// Function to send ultrasonic pulse and calculate distance
float pulse(int proxTrig, int proxEcho)
{
  digitalWrite(proxTrig, HIGH); // Send ultrasonic pulse
  delayMicroseconds(10); // Wait for pulse to settle
  digitalWrite(proxTrig, LOW);

  float duration_us = pulseIn(proxEcho, HIGH); // Measure pulse duration

  return duration_us * .017; // Convert duration to distance in cm
}

// Function to set servo angle 
void setServoAngle(int angle, int servoPin)
{
  int pulseWidth = map(angle, 0, 180, minPulseWidth, maxPulseWidth); // Map angle to pulse width
  digitalWrite(servoPin, HIGH); // Start the pulse
  delayMicroseconds(pulseWidth); // Wait for the pulse width
  digitalWrite(servoPin, LOW); // End the pulse
  delay(20); // Wait for servo to settle
}

// Function to get distance from front ultrasonic sensor
float getFrontDistance()
{
    return round(pulse(frontTrig, frontEcho) * 100.0) / 100.0; // Round and return distance in cm
}

// Function to get distance from left ultrasonic sensor
float getLeftDistance()
{
    return round(pulse(leftTrig, leftEcho) * 100.0) / 100.0; // Round and return distance in cm
}

// Function to query ultrasonic sensors
float querySensors()
{
  distanceLeft = getLeftDistance(); // Get left distance
  distanceFront = getFrontDistance(); // Get front distance
}

// Function to query IR sensors
void queryIRSensors()
{
  // Set irValues[] to the actual values of IR sensors
  for (int i = 0; i < 8; i++)
  {
    irValues[i] = analogRead(irSensors[i]) > 800;
  }
}

// Function to check if all IR sensors detect black
boolean allBlack()
{
  short sum = 0; // Initialize sum counter
  queryIRSensors(); // Query IR sensors

  // Count number of sensors detecting black
  for (int i = 0; i < 8; i++)
  {
    if (irValues[i])
    {
      sum++;
    }
  }

  return sum == 8; // Return true if all sensors detect black
}

// Interrupt service routine for updating left motor encoder count
void updateLM()
{
  static unsigned long timer; // Timer for encoder update interval
  static bool lastState; // Last state of encoder pin

  noInterrupts(); // Disable interrupts

  if (millis() > timer) // Check if update interval has passed
  {
    bool state = digitalRead(encoderLM); // Read current state of encoder pin

    if (state != lastState) // If state has changed
    {
       countLM++; // Increment left motor encoder count
       lastState = state; // Update last state
    }

    timer = millis() + COUNTER_INTERVAL; // Update timer for next interval
  } 

  interrupts(); // Enable interrupts
}

// Interrupt service routine for updating right motor encoder count
void updateRM() 
{
  static unsigned long timer; // Timer for encoder update interval
  static bool lastState; // Last state of encoder pin

  noInterrupts(); // Disable interrupts

  if (millis() > timer) // Check if update interval has passed
  {
    bool state = digitalRead(encoderRM); // Read current state of encoder pin

    if (state != lastState) // If state has changed
    {
       countRM++; // Increment right motor encoder count
       lastState = state; // Update last state
    }

    timer = millis() + COUNTER_INTERVAL; // Update timer for next interval
  }

  interrupts(); // Enable interrupts
}

// Function to reset encoder counters
void resetCounters()
{
  countRM = 0; 
  countLM = 0; 
}

// Function to open gripper
void gripOpen()
{
  setServoAngle(90, servoGrip); // Set servo angle to open gripper
}

// Function to close gripper
void gripClose()
{
  setServoAngle(2, servoGrip); // Set servo angle to close gripper
}

// Function to wait for a specified amount of time
void wait(int timeToWait)
{
  long time = millis(); // Get current time

  while (millis() < time + timeToWait) // Wait until specified time has passed
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