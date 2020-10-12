//#include <CytronMotorDriver.h>
#include <Servo.h>

Servo GripperServo;        
Servo VerticalServo;
Servo ClawServo; 
Servo ArmServo;
int angle = 0;

//Pin declaration for IR Sensors
#define LEFTSEN A3
#define RIGHTSEN A0
#define FRONTSEN A2
#define BACKSEN A1
#define BACKSEN2 A5
#define ARMSEN A4

#define ENCODERright_CHANNEL_A 2 //right
#define ENCODERright_CHANNEL_B 11
#define ENCODERleft_CHANNEL_A 3 //left
#define ENCODERleft_CHANNEL_B 12

//Distance variables
int Rdis = 0;
int Ldis = 0;
int Fdis = 0;
int Bdis = 0;
int Bdis2 = 0;
int Armdis = 0;

//Pin declaration for Motors
int RightDir = 4;
int RightEn = 5;
int LeftEn = 6;
int LeftDir = 7;

long encoder_countRIGHT = 0;
long encoder_countLEFT = 0;
long turnStartCCW = 0;
long turnStartCW = 0;

int SetpointRightForward = 50;
int SetpointLeftForward = 55;
int SetpointLeftBackward = 65;
int FrontWall1 = 66;
int FrontWall2 = 270;
int SideWall = 485;

float Kp1 = 8;
float Ki1 = 0;
float Kd1 = 0;
float Kp2 = 3;
float Ki2 = 0;
float Kd2 = 5;
float Kp3 = 30;
float Ki3 = 0;
float Kd3 = 15;


float Error = 0;
int Correction = 0;
float Integral = 0;
float Derivative = 0;
float LastError = 0;

int LeftTurnSpeed = 0;
int RightTurnSpeed = 0;
int EncoderBasePWM = 210;
int WallBasePWM = 210;

int delayTime = 400;

void setup(){
    //Motor Driver Pin Setup
  pinMode(RightEn,OUTPUT);
  pinMode(RightDir,OUTPUT);
  pinMode(LeftEn,OUTPUT);
  pinMode(LeftDir,OUTPUT);
  
  //Encoder Setup
  pinMode(ENCODERright_CHANNEL_A, INPUT_PULLUP);
  pinMode(ENCODERright_CHANNEL_B, INPUT_PULLUP);

  pinMode(ENCODERleft_CHANNEL_A, INPUT_PULLUP);
  pinMode(ENCODERleft_CHANNEL_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODERright_CHANNEL_A),encoderISRright,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODERleft_CHANNEL_A),encoderISRleft,RISING);
  Serial.begin(250000);

  // IR SENSOR SETUP
  pinMode(FRONTSEN,INPUT);
  pinMode(LEFTSEN,INPUT);
  pinMode(RIGHTSEN,INPUT);
  pinMode(BACKSEN,INPUT);
  pinMode(BACKSEN2,INPUT);
  pinMode(ARMSEN,INPUT);

  GripperServo.attach(10);      
  VerticalServo.attach(9);
  ClawServo.attach(12);      
  ArmServo.attach(11);
}


void encoderISRright()

{
  if (digitalRead(ENCODERright_CHANNEL_B) ==HIGH)
  {
    encoder_countRIGHT--;
  }
  else
  {
    encoder_countRIGHT++;
  }
}

void encoderISRleft()

{
  if (digitalRead(ENCODERleft_CHANNEL_B) ==HIGH)
  {
    encoder_countLEFT++;
  }
  else
  {
    encoder_countLEFT--;
  }
}


void RightForward() {
  
  //When no wall in front, continue Forward
    Rdis = ((-1.3662*(analogRead(RIGHTSEN)))+645.12);
    Error = Rdis - SetpointRightForward;
    Integral = (Error + Integral);
    Derivative = (Error - LastError);
    Serial.print("IR Distance: ");
    Serial.println(Rdis);
    Serial.print("Error: ");
    Serial.println(Error);

//if Error is less than 0, rotate robot CW until straight
  if (Error < 0) 
  {
    Correction = Kp1 * Error + Kd1 * Derivative + Ki1 * Integral;
    LeftTurnSpeed = WallBasePWM + Correction;
    RightTurnSpeed = WallBasePWM - Correction;
  }
  
//if Error is greater than 0, rotate robot CCW until straight
  else if (Error > 0) 
  {
    int speed = Kp1 * Error + Kd1 * Derivative + Ki1 * Integral;
    LeftTurnSpeed = WallBasePWM + (speed);
    RightTurnSpeed = WallBasePWM - (speed);
  }
  
  else 
  {
    LeftTurnSpeed = WallBasePWM;
    RightTurnSpeed = WallBasePWM;
  }
      
  if (LeftTurnSpeed > 255) {
    LeftTurnSpeed = 255;
    }
  if (RightTurnSpeed > 255) {
    RightTurnSpeed = 255;
    }
  digitalWrite(4,HIGH);  
  analogWrite(5,LeftTurnSpeed);
  digitalWrite(7,HIGH);  
  analogWrite(6,RightTurnSpeed);
    
  LastError = Error;
  //delay(100);
}

void PIDForward() {
    Error = encoder_countRIGHT - encoder_countLEFT;
    Integral = (Error + Integral);
    Derivative = (Error - LastError);
    Serial.print("Error: ");
    Serial.println(Error);
//if Error is less than 0, rotate robot CW until straight
    if (Error < 0) 
    {
    Correction = Kp3 * Error + Kd3 * Derivative + Ki3 * Integral;
      LeftTurnSpeed = EncoderBasePWM - Correction;
      RightTurnSpeed = EncoderBasePWM + Correction;
    }
//if Error is greater than 0, rotate robot CCW until straight
    else if (Error > 0) 
    {
      int speed = Kp3 * Error + Kd3 * Derivative + Ki3 * Integral;
      LeftTurnSpeed = EncoderBasePWM - (speed);
      RightTurnSpeed = EncoderBasePWM + (speed);
    }
    else 
    {
      LeftTurnSpeed = EncoderBasePWM;
      RightTurnSpeed = EncoderBasePWM;
    } 
    //Driving motors Forward based on PID values
    digitalWrite(4,HIGH);  
    analogWrite(5,LeftTurnSpeed);
    digitalWrite(7,HIGH);  
    analogWrite(6,RightTurnSpeed);
    LastError = Error;
}



void StopMotors()
{
  digitalWrite(4,HIGH);  
  analogWrite(5,0);
  digitalWrite(7,HIGH);  
  analogWrite(6,0);
}


void RotateCCW()
{
  turnStartCCW = encoder_countRIGHT; 
  while(encoder_countRIGHT < (turnStartCCW + 490)){
    digitalWrite(4,LOW);  
    analogWrite(5,200);
    digitalWrite(7,HIGH);  
    analogWrite(6,200);
    Serial.print("RIGHT: ");
    Serial.println(encoder_countRIGHT);
  }
  StopMotors();
  delay(delayTime);
}


void RotateCW()
{
  digitalWrite(4,HIGH);  
  analogWrite(5,200);
  digitalWrite(7,LOW);  
  analogWrite(6,200);
}

void LowerGripper ()
{
for(angle = 140; angle>=10; angle-=1)    //angle = 180; angle>=1; angle-=1
  {                                  
    GripperServo.write(angle);                 //command to rotate the servo to the specified angle
    delay(5);                       
  } 
  
    for(angle = 180; angle>=1; angle-=1)     // command to move from 180 degrees to 0 degrees 
  {                                
    VerticalServo.write(angle);              //command to rotate the servo to the specified angle
    delay(5);                       
  } 
}

void GrabandRaise() 
{
//If miner is detected, in this case they are in the chanel from phase A/B map 

  for(angle = 0; angle < 180; angle += 1)    //angle = 180; angle>=1; angle-=1
  {                                  
    GripperServo.write(angle);                 //command to rotate the servo to the specified angle
    delay(5);                       
  } 
  delay(1000);

   for(angle = 0; angle < 180; angle += 1)     // command to move from 180 degrees to 0 degrees 
  {                                
    VerticalServo.write(angle);              //command to rotate the servo to the specified angle
    delay(5);                       
  } 
}

void loop()
{
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  if (Fdis >= FrontWall2 && Ldis >= SideWall) {
  while (Fdis > 320)
  {
    PIDForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);

//RIGHT FRONT WALL FOLLOW PID ******************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > 59)
  {
    RightForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);

//GRIPPER GRABS MINER, THEN LIFTS UP MINER*******************************************************

  for(angle = 0; angle < 180; angle += 1)    //angle = 180; angle>=1; angle-=1
  {                                  
    GripperServo.write(angle);                 //command to rotate the servo to the specified angle
    delay(5);                       
  } 
  delay(1000);

   for(angle = 0; angle < 180; angle += 1)     // command to move from 180 degrees to 0 degrees 
  {                                
    VerticalServo.write(angle);              //command to rotate the servo to the specified angle
    delay(5);                       
  } 

//START TO HEAD BACK WITH MINER NOW 
  turnStartCCW = encoder_countRIGHT; 
  while(encoder_countRIGHT < (turnStartCCW + 1200)){
    RotateCCW();
    Serial.print("RIGHT: ");
    Serial.println(encoder_countRIGHT);
  }
  StopMotors();
  delay(delayTime);
  }
  
  else if (ARMSEN <= 50 ) { // obstacle detected, robot will pick up rotate 180 degrees and drop the obstacle before turning back to original position

   GrabandRaise();

  }
  else if (Armdis >= 55 && Rdis <= 55 && Fdis >= 55 ) {  // no front wall and obstacle detected but right wall detected 

    PIDForward() ;

  }
  
  else if (Armdis >= 55 && Rdis <=55 && Fdis <=54) {  // if front wall detected 
   RotateCCW() ;
  }
}


    
