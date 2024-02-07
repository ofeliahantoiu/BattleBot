
int ledRed = 13;
int ledYellow = 12;
int ledGreen = 11;


// the setup function runs once when you press reset or power the board

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(ledRed, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(ledYellow, OUTPUT);
}

void loop() {
  digitalWrite(ledRed, LOW);  
  delay(3000);                      
  digitalWrite(ledRed, HIGH);   
  delay(3000);      

  digitalWrite(ledGreen, LOW);  
  delay(4000);                      
  digitalWrite(ledGreen, HIGH);   
  delay(4000);    

  digitalWrite(ledYellow, LOW);  
  delay(1000);                      
  digitalWrite(ledYellow, HIGH);   
  delay(1000);  
}
