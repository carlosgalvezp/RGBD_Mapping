/*************************************************************
Motor Shield 1-Channel DC Motor Demo
by Randy Sarafan

For more information see:
http://www.instructables.com/id/Arduino-Motor-Shield-Tutorial/

*************************************************************/
#define A_DIR 12
#define A_PWM 3
#define A_BRK 9

#define B_DIR 13
#define B_PWM 11
#define B_BRK 8

void setup() {
  
  //Setup Channel A
  pinMode(A_DIR, OUTPUT); //Initiates Motor Channel A pin
  pinMode(A_BRK, OUTPUT); //Initiates Brake Channel A pin
  
  //Setup Channel B
  pinMode(B_DIR, OUTPUT); //Initiates Motor Channel B pin
  pinMode(B_BRK, OUTPUT); //Initiates Brake Channel B pin
  
}

void loop(){
  
  //forward @ full speed
  digitalWrite(A_DIR, HIGH); //Establishes forward direction of Channel A
  digitalWrite(A_BRK,LOW);   //Disengage the Brake for Channel A
  analogWrite(A_PWM,100);   //Spins the motor on Channel A at full speed
  
  delay(5000);
  
  digitalWrite(A_BRK, HIGH); //Eengage the Brake for Channel A

  delay(1000);
  
  //backward @ half speed
  digitalWrite(A_DIR, LOW); //Establishes backward direction of Channel A
  digitalWrite(A_BRK,LOW);   //Disengage the Brake for Channel A
  analogWrite(A_PWM,100);   //Spins the motor on Channel A at full speed
  
  delay(5000);
  
  digitalWrite(A_BRK, HIGH); //Eengage the Brake for Channel A
  
  delay(1000);
  
  // CHANEL B
    //forward @ full speed
  digitalWrite(B_DIR, HIGH); //Establishes forward direction of Channel A
  digitalWrite(B_BRK,LOW);   //Disengage the Brake for Channel A
  analogWrite(B_PWM,100);   //Spins the motor on Channel A at full speed
  
  delay(5000);
  
  digitalWrite(B_BRK, HIGH); //Eengage the Brake for Channel A

  delay(1000);
  
  //backward @ half speed
  digitalWrite(B_DIR, LOW); //Establishes backward direction of Channel A
  digitalWrite(B_BRK,LOW);   //Disengage the Brake for Channel A
  analogWrite(B_PWM,100);   //Spins the motor on Channel A at full speed
  
  delay(5000);
  
  digitalWrite(B_BRK, HIGH); //Eengage the Brake for Channel A
  
  delay(1000);
  
}

