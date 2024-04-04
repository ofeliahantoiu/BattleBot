// LFM = Left Forward Motor A2
// LBM = Left Backward Motor A1
// RFM = Right Forward Motor B1
// RBM = Right Backward Motor B2

#define LFM 5
#define LBM 6
#define RFM 10
#define RBM 11

#define encoderRight 2 //encoder Right Motor
#define encoderLeft 3 //encoder Left Motor
 
#define frontTrig 4 // Ultrasonic sensor trigger 
#define leftTrig 8

#define frontEcho 12 // Ultrasonic sensor echo
#define leftEcho 7

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

#define maxPulseLength 1000

#define thresholdDistance 15 // Desired distance from the wall in front

float distanceFront, distanceLeft;

bool waitingStart = true;
bool startSequence = false;
bool endDetected = false;

bool turnedRight = false;

volatile unsigned long countRight = 0;
volatile unsigned long countLeft = 0;

bool isGripOpen = false;

void setup() {
  Serial.begin(9600);

  pinMode(LFM, OUTPUT);
  pinMode(LBM, OUTPUT);
  pinMode(RFM, OUTPUT);
  pinMode(RBM, OUTPUT);

  pinMode(servoGrip, OUTPUT);
  setServoAngle(95, servoGrip);

  pinMode(frontTrig, OUTPUT);
  pinMode(leftTrig, OUTPUT);
  pinMode(frontEcho, INPUT);
  pinMode(leftEcho, INPUT);

  pinMode(encoderRight, INPUT);
  pinMode(encoderLeft, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderRight), updateRM, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderLeft), updateLM, CHANGE);
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

void loop() {
//  querySensors();

//  Debug for ultrasonic sensor
//  Serial.print("Front: ");
//  Serial.print(distanceFront);
  Serial.print(" | Left: ");
  Serial.println(distanceLeft);

  // Control the performance of the car
  switch (carState) 
  {
    case STATE_FORWARD:
      handleForwardState();
      break;

    case STATE_OBSTACLE_DETECTED:
      handleObstacleDetectedState();
      break;

    case STATE_TURN_LEFT:
      handleTurnLeftState();
      break;

    case STATE_TURN_RIGHT:
      handleTurnRightState();
      break;

    case STATE_STOP:
      handleStopState();
      break;

    case STATE_WAIT:
      handleWaitState();
      break;

    default:
      break;
  }
}

void updateRM()
{
  noInterrupts();
  countRight++;
  interrupts();
}

void updateLM()
{
  noInterrupts();
  countLeft++;
  interrupts();
}

//float querySensors()
//{
//  distanceLeft = lookLeft();
//  distanceFront = lookFront();
//}
//
//float pulse(int proxTrig, int proxEcho) {
//  digitalWrite(proxTrig, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(proxTrig, LOW);
//
//  float duration_us = pulseIn(proxEcho, HIGH);
//
//  return duration_us * 0.017;
//}
//
//float lookFront() {
//  wait(300);
//  return round(pulse(frontTrig, frontEcho) * 100.0) / 100.0;
//}
//
//float lookLeft() {
//  wait(300);
//  return round(pulse(leftTrig, leftEcho) * 100.0) / 100.0;
//}


// Function to wait until a certain pulse count is reached for the left wheel
void waitUntilPulseCountLeft(unsigned long count) 
{
  int previousPulseStateLeft = digitalRead(encoderLeft);
  unsigned long lastPulseTime = millis();

  while (countLeft < count) 
  {
    int pulseStateLeft = digitalRead(encoderLeft);

    if (pulseStateLeft != previousPulseStateLeft) 
    {
      // State change
      previousPulseStateLeft = pulseStateLeft;
      countLeft++;
      lastPulseTime = millis();

      if (millis() - lastPulseTime >= maxPulseLength) 
      {
        // No pulse state change for a while, must have hit a stop
        moveBackwards(); 
        wait(350);
        moveStop(); 
        countLeft = 0;
        return;
      }
    }
  }
}

// Function to wait until a certain pulse count is reached for the right wheel
void waitUntilPulseCountRight(unsigned long count) 
{
  int previousPulseStateRight = digitalRead(encoderRight);
  unsigned long lastPulseTime = millis();

  while (countRight < count) 
  {
    int pulseStateRight = digitalRead(encoderRight);

    if (pulseStateRight != previousPulseStateRight) 
    {
      // State change
      previousPulseStateRight = pulseStateRight;
      countRight++;
      lastPulseTime = millis();

      if (millis() - lastPulseTime >= maxPulseLength) 
      {
        // No pulse state change for a while, must have hit a stop
        moveBackwards(); 
        wait(350);
        moveStop(); 
        countRight = 0;
        return;
      }
    }
  }

  countRight = 0;
}

// Function to wait until a certain pulse count is reached for both wheels
void waitUntilPulseCountBoth(unsigned long count) 
{
  // Call both individual wait functions
  waitUntilPulseCountRight(count);
  waitUntilPulseCountLeft(count);
}

void basicTurnLeft() 
{
  analogWrite(LBM, 130);
  analogWrite(RFM, 250);
  analogWrite(LFM, 0);
  analogWrite(RBM, 0);

  countLeft = 0;
  waitUntilPulseCountLeft(105);
}

void basicTurnRight() 
{
  analogWrite(LFM, 250);
  analogWrite(RBM, 130);
  analogWrite(LBM, 0);
  analogWrite(RFM, 0);

  countRight = 0;
  waitUntilPulseCountRight(105);
} 

void handleForwardState() 
{
  distanceFront = getDistance(frontTrig, frontEcho);
  distanceLeft = getDistance(leftTrig, leftEcho); 

  if (distanceFront > thresholdDistance) 
  {
    moveForward();
    Serial.print("Distance Front: ");
    Serial.println(distanceFront);  // Debug statement to print distanceFront

    // Check if adjustment is needed
    if (distanceLeft < 5) 
    {
      // Turn right a bit
      adjustRight();
      wait(20);
      moveForward();
    } 
    else if (distanceLeft > 10) 
    {
      adjustLeft();
      moveForward();
    }
  } 
  else 
  {
    carState = STATE_OBSTACLE_DETECTED;
  }
}

void moveForwardInTicks(int ticks)
{
  resetCounters();

  analogWrite(RFM, 230);
  analogWrite(LFM, 255);
  digitalWrite(LBM, LOW);
  digitalWrite(RBM, LOW);

  waitUntilPulseCountBoth(ticks);

  turnedRight = false;
  endDetected = allBlack();

  moveStop();
  
  setServoAngle(2, servoGrip);
}

float getDistance(int trigPin, int echoPin) 
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  float duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2; // Convert duration to distance in cm
  return distance;
}

void handleObstacleDetectedState() 
{
  moveBackwardInTicks(20); // Move backward for a short distance
  if (distanceLeft > 30) 
  {
    carState = STATE_TURN_LEFT;
  } 
  else 
  {
    carState = STATE_TURN_RIGHT; // Move to the turn right state
  }
}

void handleTurnLeftState() 
{
  wait(200);
  basicTurnLeft(); // Turn left
  carState = STATE_STOP;
}

void handleTurnRightState() 
{
  wait(200);
  basicTurnRight(); // Turn right
  carState = STATE_STOP; // Move to the stop state after turning
}

void handleStopState() 
{
  moveStop();
  carState = STATE_WAIT; // Move to the wait state
  wait(150);
}

void handleWaitState() 
{
  carState = STATE_FORWARD; // Move back to the forward state for the next iteration
}

void adjustRight() 
{
  moveStop();
  wait(50); 

  analogWrite(LFM, 200);
  analogWrite(RBM, 120);
  analogWrite(LBM, 0);
  analogWrite(RFM, 0);

  wait(50); 
  moveStop();
}

void adjustLeft() 
{
  if (distanceLeft > 30) 
  {
    basicTurnLeft();
    wait(50); // Reduce delay
    moveStop();
  } 
  else 
  {
    // Adjust left slightly
    analogWrite(RFM, 200);
    analogWrite(LBM, 120);
    analogWrite(LFM, 0);
    analogWrite(RBM, 0);
    wait(50); // Reduce delay
    moveStop();
  }
}

void moveForward() 
{
  analogWrite(RFM, 230);
  analogWrite(LFM, 255);
  analogWrite(LBM, 0);
  analogWrite(RBM, 0);

  turnedRight = false;
  endDetected = allBlack();
}

void moveStop() 
{
  analogWrite(RFM, 0);
  analogWrite(LFM, 0);
  analogWrite(LBM, 0);
  analogWrite(RBM, 0);
}

void moveBackwards() 
{
  analogWrite(LBM, 255);
  analogWrite(RBM, 225);
  digitalWrite(LFM, LOW);
  digitalWrite(RFM, LOW);
}

void moveBackwardInTicks(int ticks) 
{
  resetCounters();

  while (countRight < ticks) 
  {
    analogWrite(LBM, 255);
    analogWrite(RBM, 225);
    digitalWrite(LFM, LOW);
    digitalWrite(RFM, LOW);
  }

  turnedRight = false;
  endDetected = allBlack();

  moveStop();

//  setServoAngle(0, servoGrip);
}

void setServoAngle(int angle, int servoPin) 
{
  int pulseWidth = map(angle, 0, 180, minPulseWidth, maxPulseWidth);
  digitalWrite(servoPin, HIGH); // Start the pulse
  delayMicroseconds(pulseWidth); // Wait for the pulse width
  digitalWrite(servoPin, LOW); // End the pulse
  delay(20); // wait for servo to settle
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
  countRight = 0;
  countLeft = 0;
}

// waits for an amount of time in milliseconds
// used to eliminate the need of using the delay() function
void wait(int timeToWait)
{
  long time = millis();

  while (millis() < time + timeToWait)
        ;
}