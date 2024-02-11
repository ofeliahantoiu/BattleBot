// LFM = Left Forward Motor 
// LBM = Left Backward Motor
// RFM = Right Forward Motor
// RBM = Right Backward Motor

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


int distanceFront, distanceRight, distanceLeft;;

void setup() {
  
  pinMode(LFM, OUTPUT);
  pinMode(LBM, OUTPUT);
  pinMode(RFM, OUTPUT);
  pinMode(RBM, OUTPUT);

  pinMode(servoSensor, OUTPUT);
  pinMode(servoGrip, OUTPUT);

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
}

void loop() {
  
  distanceFront = getDistance();
  distanceRight = lookRight();
  distanceLeft = lookLeft();

  
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

void moveStop(){
  digitalWrite(RFM, LOW);
  digitalWrite(LFM, LOW);
  digitalWrite(RBM, LOW);
  digitalWrite(LBM, LOW);
}

void moveForward(){
    digitalWrite(LFM, HIGH);
    digitalWrite(RFM, HIGH);
    digitalWrite(LBM, LOW);
    digitalWrite(RBM, LOW);
}

void moveBackward(){
  digitalWrite(LBM, HIGH);
  digitalWrite(RBM, HIGH);
  digitalWrite(LFM, LOW);
  digitalWrite(RFM, LOW);
}

void turnRight(){
  digitalWrite(LFM, HIGH);
  digitalWrite(RBM, HIGH);
  digitalWrite(LBM, LOW);
  digitalWrite(RFM, LOW);

  delay(500);

  moveForward();
}


void turnLeft(){
  digitalWrite(LBM, HIGH);
  digitalWrite(RFM, HIGH);
  digitalWrite(LFM, LOW);
  digitalWrite(RBM, LOW);

  delay(500);

  moveForward();
}

//Function to calculate distance in cm
int getDistance(){
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
