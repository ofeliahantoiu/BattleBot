#include <Adafruit_NeoPixel.h> // Library for LEDs control

#define LFM 5 // LFM = Left Forward Motor A2
#define LBM 6 // LBM = Left Backward Motor A1
#define RFM 10 // RFM = Right Forward Motor B1
#define RBM 11 // RBM = Right Backward Motor B2

#define encoderRM 2 // Encoder Right Motor
#define encoderLM 3 // Encoder Left Motor
 
#define frontTrig 4 // Ultrasonic sensor trigger front
#define frontEcho 12 // Ultrasonic sensor echo front


#define leftTrig 8 // Ultrasonic sensor trigger left
#define leftEcho 7 // Ultrasonic sensor echo left
 
#define servoGrip 9 // Servo used for the gripper

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


float distanceFront, distanceLeft; // Variables to store distance left and distance front values

bool waitingStart = true; // The initial state of the robot, robot will wait until it detects object in front of it, then it will start sequence
bool startSequence = false; // Initialize the starting state as false, so that the robot will only start if distance condition met
bool endDetected = false; // Initialize the ending state as false, so that the robot will only release the object if it detects the end sign

bool turnedRight = false; // Variable for turning state. If the robot turn right or adjust itself to the right wall, this variable is set to true. 
                          //If the robot move forward, move backward, turn left or adjust itself to the left wall, it will be set to false

volatile int countRM = 0; // Variable to count the number of ticks the robot moved with right motor
volatile int countLM = 0; // Variable to count the number of ticks the robot moved with left motor

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
  Serial.begin(9600); // Initialize serial communication at a baud rate of 9600 bits per second. This helps to use Serial.print
  
  // Initialize the motors
  pinMode(LFM, OUTPUT);
  pinMode(LBM, OUTPUT);
  pinMode(RFM, OUTPUT);
  pinMode(RBM, OUTPUT);

  pinMode(servoGrip, OUTPUT);

  // Initialize the gripper
  for (int i = 0; i < 4; i++)
  {
    gripOpen();
  }

  gripOpen();

  // Initialize the ultrasonic sensors
  pinMode(frontTrig, OUTPUT);
  pinMode(frontEcho, INPUT);

  pinMode(leftTrig, OUTPUT);
  pinMode(leftEcho, INPUT);

  // Initialize the rotation sensors
  pinMode(encoderRM, INPUT);
  pinMode(encoderLM, INPUT);

  //  attachInterrupt(): This is a function used to attach an interrupt to a specific digital pin.
  //  digitalPinToInterrupt(encoderRM): This function translates the digital pin number (encoderRM. updateLM) to the corresponding interrupt number used by the microcontroller. 
  //  updateRM, updateLM: Name of the functions that will be called when the interrupt is triggered.
  //  CHANGE: This parameter specifies the type of change that triggers the interrupt.
  attachInterrupt(digitalPinToInterrupt(encoderRM), updateRM, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderLM), updateLM, CHANGE);

  pixels.begin(); // Initialize NeoPixel LEDs

  lightsOff(); // The initial state of LEDS are off
}

void loop() 
{
  // The code for awaiting the start;
  // If the robot sees an object in front of it, it starts
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

  // The start itself, the robot will move, pick up the object, turn left and move forward to enter the maze
  if (startSequence)
  {
    wait(200);

    moveForwardInTicks(80);
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

  // The end sequence, if the robot detects the end square, it will open the gripper, release object and move backwards a bit 
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

  // The main sequence
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

// Stop the motors before performing an action to prevent accidents or collisions
void moveStop()
{
  stopLights();
  digitalWrite(RFM, LOW);
  digitalWrite(LFM, LOW);
  digitalWrite(RBM, LOW);
  digitalWrite(LBM, LOW);
}

// The function defines the behaviour of the car when it is going forward
// it adjusts the car so that it is constantly around 8.2 cm away from the wall
void moveForward()
{
  forwardLights();
  querySensors();

  // Adjust the robot a bit to the left wall
  if(distanceLeft > 9.2 && distanceLeft < 300)
  {
    forwardLights();
    analogWrite(RFM, 199); // I'm a bit confused here,isn't the RFM supposed to have higher value than LFM? Cuz I think it try to adjust to left wall 
    analogWrite(LFM, 220);
    analogWrite(LBM, 0);
    analogWrite(RBM, 0);
  }
  // Adjust the robot a bit to the right wall
  else if (distanceLeft < 7.2)
  {
    forwardLights();
    analogWrite(RFM, 189);
    analogWrite(LFM, 220);
    analogWrite(LBM, 0);
    analogWrite(RBM, 0);
  }
  // Make the robot move forward
  else
  {
    forwardLights();
    analogWrite(RFM, 230);
    analogWrite(LFM, 255);
    analogWrite(LBM, 0);
    analogWrite(RBM, 0);
  }

  turnedRight = false;

  endDetected = allBlack();
}

// Move the robot forward in a desired ticks
void moveForwardInTicks(int ticks)
{
  forwardLights(); // Activate the light indicating that the robot is moving forward 
  resetCounters();

  while (countLM < ticks)
  {
    forwardLights();
    analogWrite(RFM, 230);
    analogWrite(LFM, 255);
    analogWrite(LBM, 0);
    analogWrite(RBM, 0);
  }

  turnedRight = false; // Indicate that the robot did not turn right
  endDetected = allBlack(); // Check if all infrared sensors detect black (indicating the end of a track or maze), updating a flag accordingly

  moveStop();
}

// Move the robot backward in a desired ticks
void moveBackwardInTicks(int ticks)
{
  resetCounters();

  while (countRM < ticks)
  {
    analogWrite(RBM, 225);
    analogWrite(LBM, 255);
    analogWrite(LFM, 0);
    analogWrite(RFM, 0);
  }

  turnedRight = false; 
  endDetected = allBlack();

  moveStop();
  
  setServoAngle(2, servoGrip); // Ensure that the gripper is closed 
}

// Execute a right turn
void turnRight()
{
  turnLights(); // Activate the light indicating that the robot is turning 
  querySensors();

  moveStop();
  wait(150);

  //  Check if the left distance is less than 10 centimeters or if the robot has already executed a right turn
  //  If either condition is met, the robot performs a basic right turn
  //  Otherwise, it adjusts its position relative to the wall using the adjustToWall() function
  if(distanceLeft < 10 || turnedRight)
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

  // Check if there is sufficient space in front of the robot to move forward after completing the turn
  if (distanceFront > 15)
  {
    wait(150);
    moveForwardInTicks(30);
  }

  return wait(150);
}

// Execute a left turn
void turnLeft()
{
  leftLights(); // Activate the light indicating that the robot is turning 
  moveForwardInTicks(40); // Move forward a bit before turning left to prevent being stuck 
  wait(350);

  leftLights();
  basicTurnLeft();
  wait(155);

  querySensors();

  // Check if there is sufficient space in front of the robot to move forward after completing the turn
  if (distanceFront > 25)
  {
    moveForwardInTicks(40);
  }

  turnedRight = false;

  return wait(150);
}

// Execute a basic left turn
void basicTurnLeft()
{
  leftLights();
  moveStop();
  resetCounters(); // Reset counters used to keep track of motor movements or encoder pulses, ensuring accurate measurement of the turn's completion

  while (countRM < 40) // Continue executing motor commands until the right motor's count reaches a predetermined value
  {
    leftLights();
    analogWrite(RFM, 220);
    analogWrite(LBM, 255);
    analogWrite(LFM, 0);
    analogWrite(RBM, 0);
  }

  moveStop();
}

// Execute a basic right turn
void basicTurnRight()
{
  turnLights();
  moveStop();
  resetCounters();

  while (countLM < 40)
  {
    turnLights();
    analogWrite(LFM, 255);
    analogWrite(RBM, 230);
    analogWrite(LBM, 0);
    analogWrite(RFM, 0);
  }

  moveStop();
}

// While performing a right turn, the car might need to be adjusted to the wall
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
// Measure the distance to an obstacle using ultrasonic sensors
float pulse(int proxTrig, int proxEcho)
{
  digitalWrite(proxTrig, HIGH); // Send a 10 microsecond pulse to the ultrasonic sensors by setting the trigger pin (proxTrig) to HIGH
  delayMicroseconds(10); // Ensure that the trigger pulse is sent properly
  digitalWrite(proxTrig, LOW); // After the delay, the trigger pin is set back to LOW to end the trigger pulse

  //Measure the duration of the echo pulse using the pulseIn function. It waits for the echo pin (proxEcho) to go HIGH and then measures the time until it goes LOW again
  float duration_us = pulseIn(proxEcho, HIGH);

  // The duration is converted to distance in centimeters by multiplying it by the speed of sound in air (approximately 0.034 centimeters per microsecond), 
  // and then dividing by 2 since the pulse travels to the obstacle and back
  return duration_us * .017;
}

// Function to set gripper angle 
void setServoAngle(int angle, int servoPin)
{
  int pulseWidth = map(angle, 0, 180, minPulseWidth, maxPulseWidth);
  digitalWrite(servoPin, HIGH); // Start the pulse
  delayMicroseconds(pulseWidth); // Wait for the pulse width
  digitalWrite(servoPin, LOW); // End the pulse
  delay(20); // wait for servo to settle
}

// Return forward distance in cm
float getFrontDistance()
{
    return round(pulse(frontTrig, frontEcho) * 100.0) / 100.0;
}

// Return left distance in cm
float getLeftDistance()
{
    return round(pulse(leftTrig, leftEcho) * 100.0) / 100.0;
}

float querySensors()
{
  distanceLeft = getLeftDistance(); // Retrieve the distance measured by the left sensor and assigns it to the variable distanceLeft
  distanceFront = getFrontDistance(); // Retrieve the distance measured by the front sensor and assigns it to the variable distanceFront
}

// Update an array irValues[] with the current readings from infrared (IR) sensors
void queryIRSensors()
{
  //The function sets irValues[] to the actual values
  for (int i = 0; i < 6; i++)
  {
    // If the reading is greater than 800, it sets the corresponding element in irValues[] to true, 
    // indicating that the sensor detects an obstacle. Otherwise, it sets it to false
    irValues[i] = analogRead(irSensors[i]) > 800;
  }
}

// Check if all the infrared (IR) sensors detect an obstacle, indicating that the surface is entirely black
boolean allBlack()
{
  short sum = 0;
  queryIRSensors();
  for (int i = 0; i < 6; i++)
  {
    if (irValues[i])
    {
      sum++;
    }
  }
  // After iterating through all the sensors, it compares the value of sum with 6, which represents the total number of sensors
  // If sum equals 6, it means that all sensors detect obstacles
  return sum == 6;
}

// Reset the counters countRM and countLM to zero
void resetCounters()
{
  countRM = 0; // Reset the counter for the right motor
  countLM = 0; // Reset the counter for the left motor
}

// This function is called whenever there is a change in the state of the right motor encoder pin 
// It increments the countRM variable by one to keep track of the number of encoder ticks on the right motor
void updateRM()
{
  noInterrupts();
  countRM++;
  interrupts();
}

// This function is called whenever there is a change in the state of the left motor encoder pin 
// It increments the countLM variable by one to keep track of the number of encoder ticks on the left motor
void updateLM()
{
  noInterrupts();
  countLM++;
  interrupts();
}

// Open the gripper by setting the servo motor connected to the grip mechanism to an angle of 90 degrees
void gripOpen()
{
  setServoAngle(90, servoGrip);
}

// Close the gripper by setting the servo motor connected to the grip mechanism to an angle of 2 degrees
void gripClose()
{
  setServoAngle(2, servoGrip);
}

// Wait for an amount of time in milliseconds
// Used to eliminate the need of using the delay() function
void wait(int timeToWait)
{
  long time = millis();

  while (millis() < time + timeToWait)
        ;
}

// Function to set specific LEDs to represent left movement
void leftLights()
{
  pixels.setPixelColor(0, YELLOW); 
  pixels.setPixelColor(1, YELLOW); 
  pixels.setPixelColor(2, WHITE);  
  pixels.setPixelColor(3, WHITE);  
  pixels.show(); // Update LEDs with new colors
}

// Function to set all LEDs to represent turning movement
void turnLights()
{
  pixels.setPixelColor(0, WHITE); 
  pixels.setPixelColor(1, WHITE); 
  pixels.setPixelColor(2, YELLOW); 
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
