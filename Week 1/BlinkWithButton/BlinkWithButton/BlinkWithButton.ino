const int redLed = 13;
const int greenLed = 11;

const int button1 = 2;
const int button2 = 3;
const int button3 = 4;

int buttonState1, buttonState2, buttonState3 = 0;

bool ledState;

void setup() {
  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);

  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  pinMode(button3, INPUT);
}

void loop() {
  
  digitalWrite(redLed, LOW);  
  delay(1000);                      
  digitalWrite(redLed, HIGH);   
  delay(1000);

  int buttonState1 = digitalRead(button1);
  int buttonState2 = digitalRead(button2);
  int buttonState3 = digitalRead(button3);

  
  if (buttonState1 == LOW || buttonState2 == LOW || buttonState3 == LOW) {
    
    digitalWrite(greenLed, HIGH);
  } else {
    
    digitalWrite(greenLed, LOW);
  }
}


  


