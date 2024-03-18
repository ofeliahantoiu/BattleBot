bool isGripping = false;
bool justTurned = false;

#define LFM 5
#define LBM 6
#define RFM 10
#define RBM 11

#define encoderRM 2 //encoder Right Motor
#define encoderLM 3 //encoder Left Motor
 
#define frontTrig 13 // Ultrasonic sensor trigger 
#define leftTrig 8
#define frontEcho 12 // Ultrasonic sensor echo
#define leftEcho 9

#define servoGrip 7 //servo used for the gripper

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

  pinMode(encoderRM, INPUT);
  pinMode(encoderLM, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderRM), updateRM, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderLM), updateLM, CHANGE);
}

enum CarState {
  STATE_FORWARD,
  STATE_OBSTACLE_DETECTED,
  STATE_TURN_RIGHT,
  STATE_TURN_LEFT,
  STATE_STOP,
  STATE_WAIT
};

CarState carState = STATE_FORWARD;

void loop() {
  querySensors();

  // Debug for ultrasonic sensor
  Serial.print("Front: ");
  Serial.print(distanceFront);
  Serial.print(" | Left: ");
  Serial.println(distanceLeft);

  // Control the performance of the car
  switch (carState) {
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

void handleForwardState() {
  querySensors();

  if (distanceFront > 12) {
    moveForward();

    // Check if adjustment is needed
    if (distanceLeft < 8) {
      // Turn right a bit
      adjustRight();
      moveForward();
    } else if (distanceLeft > 20) {
      adjustLeft();
      moveForward();
    }
  } else if (distanceFront < 8) {
    carState = STATE_OBSTACLE_DETECTED;
  }
}

void handleObstacleDetectedState() {
  moveBackwardInTicks(20); // Move backward for a short distance
  if (distanceLeft > 30) {
    carState = STATE_TURN_LEFT;
  } else {
    carState = STATE_TURN_RIGHT; // Move to the turn right state
  }
}

void handleTurnLeftState() {
  wait(350);
  basicTurnLeft(); // Turn left
  carState = STATE_STOP;
}

void handleTurnRightState() {
  wait(350);
  basicTurnRight(); // Turn right
  carState = STATE_STOP; // Move to the stop state after turning
}

void handleStopState() {
  moveStop();
  carState = STATE_WAIT; // Move to the wait state
  wait(150);
}

void handleWaitState() {
  carState = STATE_FORWARD; // Move back to the forward state for the next iteration
}

void adjustRight() {
  moveStop();
  wait(50); // Reduce delay

  analogWrite(LFM, 200);
  analogWrite(RBM, 120);
  analogWrite(LBM, 0);
  analogWrite(RFM, 0);

  wait(50); // Reduce delay
  moveStop();
}

void adjustLeft() {
  if (distanceLeft > 30) {
    basicTurnLeft();
    wait(50); // Reduce delay
    moveStop();
  } else {
    // Adjust left slightly
    analogWrite(RFM, 200);
    analogWrite(LBM, 120);
    analogWrite(LFM, 0);
    analogWrite(RBM, 0);
    wait(50); // Reduce delay
    moveStop();
  }
}

void moveForward() {
  analogWrite(RFM, 240);
  analogWrite(LFM, 255);
  analogWrite(LBM, 0);
  analogWrite(RBM, 0);

  turnedRight = false;
  endDetected = allBlack();
}

void moveStop() {
  analogWrite(RFM, 0);
  analogWrite(LFM, 0);
  analogWrite(LBM, 0);
  analogWrite(RBM, 0);
}

void moveBackwardInTicks(int ticks) {
  resetCounters();

  while (countRM < ticks) {
    analogWrite(LBM, 255);
    analogWrite(RBM, 225);
    digitalWrite(LFM, LOW);
    digitalWrite(RFM, LOW);
  }

  turnedRight = false;
  endDetected = allBlack();

  moveStop();

  setServoAngle(0, servoGrip);
}

void basicTurnLeft() {
  analogWrite(LBM, 130);
  analogWrite(RFM, 250);
  analogWrite(LFM, 0);
  analogWrite(RBM, 0);
}

void basicTurnRight() {
  analogWrite(LFM, 250);
  analogWrite(RBM, 130);
  analogWrite(LBM, 0);
  analogWrite(RFM, 0);
}

float pulse(int proxTrig, int proxEcho) {
  digitalWrite(proxTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(proxTrig, LOW);

  float duration_us = pulseIn(proxEcho, HIGH);

  return duration_us * 0.017;
}

float lookFront() {
  wait(300);
  return round(pulse(frontTrig, frontEcho) * 100.0) / 100.0;
}

float lookLeft() {
  wait(300);
  return round(pulse(leftTrig, leftEcho) * 100.0) / 100.0;
}

void setServoAngle(int angle, int servoPin) {
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
