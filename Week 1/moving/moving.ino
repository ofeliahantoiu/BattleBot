const int motorA1 = 9;
const int motorA2 = 6;
const int motorB1 = 5;
const int motorB2 = 3;

const int pws = 88;

int distanceThreshold = 0;
int cm = 0;

void setup() {

  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

}

void loop() {
  
  

  distanceThreshold = 5;
  cm = 0.01723 * readDistance(7, 8);
  
  
}

void forward(int pws){

  digitalWrite(motorA2, pws);
  digitalWrite(motorB1, pws);
  
}

void backward(int pws){

  digitalWrite(motorA1, pws);
  digitalWrite(motorB2, pws);
  

}

void right(int pws){

  digitalWrite(motorA2, pws);
  digitalWrite(motorB2, pws);

}

long readDistance(int trig, int echo){  
  pinMode(trig, OUTPUT);  

  // Clear the trigger  
  digitalWrite(trig, LOW);  
  delayMicroseconds(2);

  // Sets the trigger pin to HIGH state for 10 microseconds  
  digitalWrite(trig, HIGH);  
  delayMicroseconds(10);  
  digitalWrite(trig, LOW);

  pinMode(echo, INPUT);  
  // Reads the echo pin, and returns the sound wave travel time in microseconds  
  return pulseIn(echo, HIGH);
  }
