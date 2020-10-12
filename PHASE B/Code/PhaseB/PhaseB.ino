#include <CytronMotorDriver.h>
#include <Servo.h>

Servo GripperServo;        
Servo VerticalServo;
int angle = 0;

//Pin declaration for IR Sensors
#define LEFTSEN A3
#define RIGHTSEN A0
#define FRONTSEN A2
#define BACKSEN A1
#define BACKSEN2 A5

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
int FrontWall3 = 485;

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

  GripperServo.attach(10);      
  VerticalServo.attach(9);
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


void LeftForward() {
  //When no wall in front, continue Forward
    Ldis = ((-1.3662*(analogRead(LEFTSEN)))+645.12);
    Error = Ldis - SetpointLeftForward;
    Integral = (Error + Integral);
    Derivative = (Error - LastError);
    Serial.print("IR Distance: ");
    Serial.println(Ldis);
    Serial.print("Error: ");
    Serial.println(Error);
//if Error is less than 0, rotate robot CW until straight
  if (Error < 0) 
  {
    Correction = Kp1 * Error + Kd1 * Derivative + Ki1 * Integral;  
    LeftTurnSpeed = WallBasePWM - Correction;
    RightTurnSpeed = WallBasePWM + Correction;
  }
//if Error is greater than 0, rotate robot CCW until straight
  else if (Error > 0) 
  {
    int speed = Kp1 * Error + Kd1 * Derivative + Ki1 * Integral;
    LeftTurnSpeed = WallBasePWM - (speed);
    RightTurnSpeed = WallBasePWM + (speed);
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


void LeftBackward() {
  //When no wall in front, continue Forward
    Bdis = ((-1.3662*(analogRead(BACKSEN)))+645.12);
    Error = Bdis - SetpointLeftBackward;
    Integral = (Error + Integral);
    Derivative = (Error - LastError);
    Serial.print("IR Distance: ");
    Serial.println(Bdis);
    Serial.print("Error: ");
    Serial.println(Error);
//if Error is less than 0, rotate robot CW until straight
  if (Error < 0) 
  {
    Correction = Kp2 * Error + Kd2 * Derivative + Ki2 * Integral;  
    LeftTurnSpeed = WallBasePWM - Correction;
    RightTurnSpeed = WallBasePWM + Correction;
  }
//if Error is greater than 0, rotate robot CCW until straight
  else if (Error > 0) 
  {
    int speed = Kp2 * Error + Kd2 * Derivative + Ki2 * Integral;
    LeftTurnSpeed = WallBasePWM - (speed);
    RightTurnSpeed = WallBasePWM + (speed);
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
  digitalWrite(4,LOW);  
  analogWrite(6,LeftTurnSpeed);
  digitalWrite(7,LOW);  
  analogWrite(5,RightTurnSpeed);
    
  LastError = Error;
  //delay(100);
}

void LeftBackwardRamp() {
  //When no wall in front, continue Forward
    Bdis = ((-1.3662*(analogRead(BACKSEN)))+645.12);
    Error = Bdis - 55;
    Integral = (Error + Integral);
    Derivative = (Error - LastError);
    Serial.print("IR Distance: ");
    Serial.println(Bdis);
    Serial.print("Error: ");
    Serial.println(Error);
//if Error is less than 0, rotate robot CW until straight
  if (Error < 0) 
  {
    Correction = Kp2 * Error + Kd2 * Derivative + Ki2 * Integral;  
    LeftTurnSpeed = WallBasePWM - Correction;
    RightTurnSpeed = WallBasePWM + Correction;
  }
//if Error is greater than 0, rotate robot CCW until straight
  else if (Error > 0) 
  {
    int speed = Kp2 * Error + Kd2 * Derivative + Ki2 * Integral;
    LeftTurnSpeed = WallBasePWM - (speed);
    RightTurnSpeed = WallBasePWM + (speed);
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
  digitalWrite(4,LOW);  
  analogWrite(6,LeftTurnSpeed);
  digitalWrite(7,LOW);  
  analogWrite(5,RightTurnSpeed);
    
  LastError = Error;
}

void LeftBackward14() {
  //When no wall in front, continue Forward
    Bdis = ((-1.3662*(analogRead(BACKSEN)))+645.12);
    Error = Bdis - 50;
    Integral = (Error + Integral);
    Derivative = (Error - LastError);
    Serial.print("IR Distance: ");
    Serial.println(Bdis);
    Serial.print("Error: ");
    Serial.println(Error);
//if Error is less than 0, rotate robot CW until straight
  if (Error < 0) 
  {
    Correction = Kp2 * Error + Kd2 * Derivative + Ki2 * Integral;  
    LeftTurnSpeed = WallBasePWM - Correction;
    RightTurnSpeed = WallBasePWM + Correction;
  }
//if Error is greater than 0, rotate robot CCW until straight
  else if (Error > 0) 
  {
    int speed = Kp2 * Error + Kd2 * Derivative + Ki2 * Integral;
    LeftTurnSpeed = WallBasePWM - (speed);
    RightTurnSpeed = WallBasePWM + (speed);
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
  digitalWrite(4,LOW);  
  analogWrite(6,LeftTurnSpeed);
  digitalWrite(7,LOW);  
  analogWrite(5,RightTurnSpeed);
    
  LastError = Error;
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


void PIDBackward() {
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
    digitalWrite(4,LOW);  
    analogWrite(5,LeftTurnSpeed);
    digitalWrite(7,LOW);  
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
  digitalWrite(4,LOW);  
  analogWrite(5,200);
  digitalWrite(7,HIGH);  
  analogWrite(6,200);
}


void RotateCW()
{
  digitalWrite(4,HIGH);  
  analogWrite(5,200);
  digitalWrite(7,LOW);  
  analogWrite(6,200);
}



void loop()
{

//START RIGHT FRONT***************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > FrontWall2)
  {
    RightForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);
  
//FIRST TURN (CCW)*******************************************
  turnStartCCW = encoder_countRIGHT; 
  while(encoder_countRIGHT < (turnStartCCW + 490)){
    RotateCCW();
    Serial.print("RIGHT: ");
    Serial.println(encoder_countRIGHT);
  }
  StopMotors();
  delay(delayTime);
  
//FORWARD ENCODER PID FOR 2 BLOCKS********************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > FrontWall1)
  {
    PIDForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);

//SECOND TURN (CW)******************************************************
  turnStartCW = encoder_countLEFT; 
  while(encoder_countLEFT < (turnStartCW + 500)){
    RotateCW();
    Serial.print("LEFT: ");
    Serial.println(encoder_countLEFT);
  }
  StopMotors();
  delay(delayTime);

//FORWARD ENCODER PID FOR 2 BLOCKS**************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > FrontWall1)
  {
    PIDForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);

//THIRD TURN (CCW)************************************************************
  turnStartCCW = encoder_countRIGHT; 
  while(encoder_countRIGHT < (turnStartCCW + 490)){
    RotateCCW();
    Serial.print("RIGHT: ");
    Serial.println(encoder_countRIGHT);
  }
  StopMotors();
  delay(delayTime);

//RIGHT FRONT FOLLOW PID ******************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > FrontWall1)
  {
    RightForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);

//FOURTH TURN(CCW)**************************************************************
  turnStartCCW = encoder_countRIGHT; 
  while(encoder_countRIGHT < (turnStartCCW + 490)){
    RotateCCW();
    Serial.print("RIGHT: ");
    Serial.println(encoder_countRIGHT);
  }
  StopMotors();
  delay(delayTime);

//RIGHT FRONT WALL FOLLOW PID ******************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > FrontWall2)
  {
    RightForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);

//FORWARD ENCODER PID FOR 2 BLOCKS**************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > FrontWall1)
  {
    PIDForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);

//5TH TURN (CW)******************************************************
  turnStartCW = encoder_countLEFT; 
  while(encoder_countLEFT < (turnStartCW + 500)){
    RotateCW();
    Serial.print("LEFT: ");
    Serial.println(encoder_countLEFT);
  }
  StopMotors();
  delay(delayTime);

//FORWARD ENCODER PID FOR 2 BLOCKS**************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > FrontWall1)
  {
    PIDForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);
  
//6TH TURN (CW)******************************************************
  turnStartCW = encoder_countLEFT; 
  while(encoder_countLEFT < (turnStartCW + 500)){
    RotateCW();
    Serial.print("LEFT: ");
    Serial.println(encoder_countLEFT);
  }
  StopMotors();
  delay(delayTime);

/*
//FORWARD ENCODER PID FOR 2 BLOCKS**************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > FrontWall3)
  {
    PIDForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);
*/
//RUN RIGHT WALL FOLLOW FOR 2 SECONDS********************
  int starttime6 = millis();
  int endtime6 = starttime6;
  while ((endtime6 - starttime6) <=1050) 
  {
    PIDForward();
    int loopcount6 = loopcount6+1;
    endtime6 = millis();
    } 
  StopMotors();
  delay(delayTime);
  
//7TH TURN (CCW)********************************************************************
  turnStartCCW = encoder_countRIGHT; 
  while(encoder_countRIGHT < (turnStartCCW + 490)){
    RotateCCW();
    Serial.print("RIGHT: ");
    Serial.println(encoder_countRIGHT);
  }
  StopMotors();
  delay(delayTime);

//FORWARD ENCODER PID FOR 2 BLOCKS**************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > 75)
  {
    PIDForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);

//8TH TURN (CCW)*************************************************************************
  turnStartCCW = encoder_countRIGHT; 
  while(encoder_countRIGHT < (turnStartCCW + 490)){
    RotateCCW();
    Serial.print("RIGHT: ");
    Serial.println(encoder_countRIGHT);
  }
  StopMotors();
  delay(delayTime);
/*
//BACKWARD ENCODER PID FOR 2 BLOCKS**************************************************
  Ldis = ((-1.3662*(analogRead(LEFTSEN)))+645.12);
  while (Ldis > 50)
  {
    PIDBackward();
    Ldis = ((-1.3662*(analogRead(LEFTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);*/

//LEFT BACKWARD WALL FOLLOW PID (DOWN SHORT RAMP)_________________________START OF RAMP__________________
  Bdis2 = ((-1.3662*(analogRead(BACKSEN2)))+645.12);
  while (Bdis2 > 50)
  {
    LeftBackwardRamp();
    Bdis2 = ((-1.3662*(analogRead(BACKSEN2)))+645.12);
    Serial.println(Bdis2);
    }
  StopMotors();
  delay(delayTime);
/*
//RUN RIGHT WALL FOLLOW FOR 2 SECONDS********************
  int starttime4 = millis();
  int endtime4 = starttime4;
  while ((endtime4 - starttime4) <=250) 
  {
    PIDBackward();
    int loopcount4 = loopcount4+1;
    endtime4 = millis();
    } */

//9TH TURN (CW)
  turnStartCW = encoder_countLEFT; 
  while(encoder_countLEFT < (turnStartCW + 500)){
    RotateCW();
    Serial.print("LEFT: ");
    Serial.println(encoder_countLEFT);
  }
  StopMotors();
  delay(delayTime);
/*
//BACKWARD ENCODER PID FOR 2 BLOCKS**************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis < FrontWall2)
  {
    PIDBackward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);*/

//LEFT BACKWARD WALL FOLLOW PID (DOWN LONG RAMP)*************************************
  Bdis2 = ((-1.3662*(analogRead(BACKSEN2)))+645.12);
  while (Bdis2 > 58)
  {
    LeftBackwardRamp();
    Bdis2 = ((-1.3662*(analogRead(BACKSEN2)))+645.12);
    Serial.println(Bdis2);
    }
  StopMotors();
  delay(delayTime);

/*
//RUN RIGHT WALL FOLLOW FOR 2 SECONDS********************
  int starttime3 = millis();
  int endtime3 = starttime3;
  while ((endtime3 - starttime3) <=150) 
  {
    PIDBackward();
    int loopcount3 = loopcount3+1;
    endtime3 = millis();
    } */

//10TH TURN (CW)
  turnStartCW = encoder_countLEFT; 
  while(encoder_countLEFT < (turnStartCW + 500)){
    RotateCW();
    Serial.print("LEFT: ");
    Serial.println(encoder_countLEFT);
  }
  StopMotors();
  delay(delayTime);

//LEFT WALL FOLLOW (FORWARDS FACING)***************************************************
  Bdis2 = ((-1.3662*(analogRead(BACKSEN2)))+645.12);
  while (Bdis2 > 250 )
  {
    LeftBackward();
    Bdis2 = ((-1.3662*(analogRead(BACKSEN2)))+645.12);
    }
  StopMotors();
  delay(delayTime);
/*
//TURN 180****************************
  turnStartCCW = encoder_countRIGHT; 
  while(encoder_countRIGHT < (turnStartCCW + 1200)){
    RotateCCW();
    Serial.print("RIGHT: ");
    Serial.println(encoder_countRIGHT);
  }
  StopMotors();
  delay(delayTime);
  
//LEFT FRONT FOLLOW PID ******************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > FrontWall2)
  {
    LeftForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);*/
  
//12TH TURN (CCW)*************************************************************************
  turnStartCCW = encoder_countRIGHT; 
  while(encoder_countRIGHT < (turnStartCCW + 490)){
    RotateCCW();
    Serial.print("RIGHT: ");
    Serial.println(encoder_countRIGHT);
  }
  StopMotors();
  delay(delayTime);

//FORWARD ENCODER PID FOR 2 BLOCKS*******************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > FrontWall1)
  {
    PIDForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);

//12TH TURN (CCW)*************************************************************************
  turnStartCCW = encoder_countRIGHT; 
  while(encoder_countRIGHT < (turnStartCCW + 490)){
    RotateCCW();
    Serial.print("RIGHT: ");
    Serial.println(encoder_countRIGHT);
  }
  StopMotors();
  delay(delayTime);

//FORWARD ENCODER PID FOR 2 BLOCKS*******************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > FrontWall1)
  {
    PIDForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);
  
//13TH TURN (CCW)*************************************************************************
  turnStartCCW = encoder_countRIGHT; 
  while(encoder_countRIGHT < (turnStartCCW + 470)){
    RotateCCW();
    Serial.print("RIGHT: ");
    Serial.println(encoder_countRIGHT);
  }
  StopMotors();
  delay(delayTime);

//LEFT WALL FOLLOW (FORWARDS FACING)***************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis < 430)
  {
    LeftBackward14();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);

//14TH TURN (CCW)*************************************************************************
  turnStartCCW = encoder_countRIGHT; 
  while(encoder_countRIGHT < (turnStartCCW + 490)){
    RotateCCW();
    Serial.print("RIGHT: ");
    Serial.println(encoder_countRIGHT);
  }
  StopMotors();
  delay(delayTime);

//FORWARD ENCODER PID FOR 2 BLOCKS**************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > FrontWall3)
  {
    PIDForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);

//RIGHT FRONT WALL FOLLOW PID ******************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > 66)
  {
    RightForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);

//15TH TURN (CCW)*************************************************************************
  turnStartCCW = encoder_countRIGHT; 
  while(encoder_countRIGHT < (turnStartCCW + 490)){
    RotateCCW();
    Serial.print("RIGHT: ");
    Serial.println(encoder_countRIGHT);
  }
  StopMotors();
  delay(delayTime);

//FORWARD ENCODER PID FOR 2 BLOCKS**************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > FrontWall2)
  {
    PIDForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);

//16TH TURN (CCW)*************************************************************************
  turnStartCCW = encoder_countRIGHT; 
  while(encoder_countRIGHT < (turnStartCCW + 490)){
    RotateCCW();
    Serial.print("RIGHT: ");
    Serial.println(encoder_countRIGHT);
  }
  StopMotors();
  delay(delayTime); 
  

//OPEN GRIPPER AND LOWER VERTICAL SERVO**************************************************
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


//FORWARD ENCODER PID FOR 2 BLOCKS*****************UNTIL HIT MINER*************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
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

//START TO HEAD BACK WITH MINER NOW _______________________________________________________________________GOING BACK WITH MINRER NOW_______
//17TH TURN FULL 180 (CCW)*************************************************************************
  turnStartCCW = encoder_countRIGHT; 
  while(encoder_countRIGHT < (turnStartCCW + 1200)){
    RotateCCW();
    Serial.print("RIGHT: ");
    Serial.println(encoder_countRIGHT);
  }
  StopMotors();
  delay(delayTime);

//RIGHT FRONT FOLLOW PID ******************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > FrontWall2)
  {
    RightForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);

//FORWARD ENCODER PID FOR 2 BLOCKS*****************UNTIL WALL*************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > FrontWall1)
  {
    PIDForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);

//18TH TURN (CW)***************************************************************
  turnStartCW = encoder_countLEFT; 
  while(encoder_countLEFT < (turnStartCW + 500)){
    RotateCW();
    Serial.print("LEFT: ");
    Serial.println(encoder_countLEFT);
  }
  StopMotors();
  delay(delayTime);

//FORWARD ENCODER PID FOR 2 BLOCKS**************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > FrontWall1)
  {
    PIDForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);

//19TH TURN (CW)***************************************************************
  turnStartCW = encoder_countLEFT; 
  while(encoder_countLEFT < (turnStartCW + 500)){
    RotateCW();
    Serial.print("LEFT: ");
    Serial.println(encoder_countLEFT);
  }
  StopMotors();
  delay(delayTime);


//LEFT FRONT FOLLOW PID ******************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > FrontWall2)
  {
    LeftForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);

//FORWARD ENCODER PID FOR 2 BLOCKS**************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > FrontWall1)
  {
    PIDForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);

//20TH TURN (CCW)*************************************************************************
  turnStartCCW = encoder_countRIGHT; 
  while(encoder_countRIGHT < (turnStartCCW + 490)){
    RotateCCW();
    Serial.print("RIGHT: ");
    Serial.println(encoder_countRIGHT);
  }
  StopMotors();
  delay(delayTime);

//RIGHT FRONT WALL FOLLOW***************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > FrontWall1)
  {
    RightForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);

//21ST TURN (CCW)*************************************************************************
  turnStartCCW = encoder_countRIGHT; 
  while(encoder_countRIGHT < (turnStartCCW + 490)){
    RotateCCW();
    Serial.print("RIGHT: ");
    Serial.println(encoder_countRIGHT);
  }
  StopMotors();
  delay(delayTime);

//FORWARD ENCODER PID FOR 2 BLOCKS**************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > FrontWall3)
  {
    PIDForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);

//22ND TURN (CW)***************************************************************
  turnStartCW = encoder_countLEFT; 
  while(encoder_countLEFT < (turnStartCW + 500)){
    RotateCW();
    Serial.print("LEFT: ");
    Serial.println(encoder_countLEFT);
  }
  StopMotors();
  delay(delayTime);

//FORWARD ENCODER PID FOR 2 BLOCKS**************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > FrontWall1)
  {
    PIDForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);

//23RD TURN (CCW)*************************************************************************
  turnStartCCW = encoder_countRIGHT; 
  while(encoder_countRIGHT < (turnStartCCW + 490)){
    RotateCCW();
    Serial.print("RIGHT: ");
    Serial.println(encoder_countRIGHT);
  }
  StopMotors();
  delay(delayTime);

//RIGHT FRONT WALL FOLLOW (ALONG BOT OF RAMP)***************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > FrontWall1)
  {
    RightForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);

//24TH TURN (CCW)*************************************************************************
  turnStartCCW = encoder_countRIGHT; 
  while(encoder_countRIGHT < (turnStartCCW + 490)){
    RotateCCW();
    Serial.print("RIGHT: ");
    Serial.println(encoder_countRIGHT);
  }
  StopMotors();
  delay(delayTime);

//RIGHT FRONT WALL FOLLOW (UP LONG RAMP)***************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > FrontWall1)
  {
    RightForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);

//25TH TURN (CCW)*************************************************************************
  turnStartCCW = encoder_countRIGHT; 
  while(encoder_countRIGHT < (turnStartCCW + 490)){
    RotateCCW();
    Serial.print("RIGHT: ");
    Serial.println(encoder_countRIGHT);
  }
  StopMotors();
  delay(delayTime);

//RUN RIGHT WALL FOLLOW FOR 2 SECONDS********************
  int starttime = millis();
  int endtime = starttime;
  while ((endtime - starttime) <=6000) 
  {
    RightForward();
    int loopcount = loopcount+1;
    endtime = millis();
    } 
  
//RIGHT FRONT WALL FOLLOW (UP SHORT RAMP)***************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis < FrontWall2)
  {
    RightForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);

//26TH TURN (CCW)*************************************************************************
  turnStartCCW = encoder_countRIGHT; 
  while(encoder_countRIGHT < (turnStartCCW + 490)){
    RotateCCW();
    Serial.print("RIGHT: ");
    Serial.println(encoder_countRIGHT);
  }
  StopMotors();
  delay(delayTime);

//FORWARD ENCODER PID FOR 2 BLOCKS**************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > FrontWall1)
  {
    PIDForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);

//27TH TURN (CW)***************************************************************
  turnStartCW = encoder_countLEFT; 
  while(encoder_countLEFT < (turnStartCW + 500)){
    RotateCW();
    Serial.print("LEFT: ");
    Serial.println(encoder_countLEFT);
  }
  StopMotors();
  delay(delayTime);

//FORWARD ENCODER PID FOR 2 BLOCKS**************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > FrontWall1)
  {
    PIDForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);

//28TH TURN (CCW)*************************************************************************
  turnStartCCW = encoder_countRIGHT; 
  while(encoder_countRIGHT < (turnStartCCW + 490)){
    RotateCCW();
    Serial.print("RIGHT: ");
    Serial.println(encoder_countRIGHT);
  }
  StopMotors();
  delay(delayTime);

//FORWARD ENCODER PID FOR 2 BLOCKS**************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > FrontWall1)
  {
    PIDForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);

//29TH TURN (CCW)*************************************************************************
  turnStartCCW = encoder_countRIGHT; 
  while(encoder_countRIGHT < (turnStartCCW + 490)){
    RotateCCW();
    Serial.print("RIGHT: ");
    Serial.println(encoder_countRIGHT);
  }
  StopMotors();
  delay(delayTime);

//FORWARD ENCODER PID FOR 2 BLOCKS**************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > FrontWall3)
  {
    PIDForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);

//LEFT FRONT FOLLOW PID ******************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > FrontWall1)
  {
    LeftForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);

//30TH TURN (CW)***************************************************************
  turnStartCW = encoder_countLEFT; 
  while(encoder_countLEFT < (turnStartCW + 500)){
    RotateCW();
    Serial.print("LEFT: ");
    Serial.println(encoder_countLEFT);
  }
  StopMotors();
  delay(delayTime);

//LEFT FRONT FOLLOW PID ******************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > FrontWall1)
  {
    LeftForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);

//31ST TURN (CW)***************************************************************
  turnStartCW = encoder_countLEFT; 
  while(encoder_countLEFT < (turnStartCW + 500)){
    RotateCW();
    Serial.print("LEFT: ");
    Serial.println(encoder_countLEFT);
  }
  StopMotors();
  delay(delayTime);

//FORWARD ENCODER PID FOR 2 BLOCKS**************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > FrontWall3)
  {
    PIDForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);

//32ND TURN (CCW)*************************************************************************
  turnStartCCW = encoder_countRIGHT; 
  while(encoder_countRIGHT < (turnStartCCW + 490)){
    RotateCCW();
    Serial.print("RIGHT: ");
    Serial.println(encoder_countRIGHT);
  }
  StopMotors();
  delay(delayTime);

//FORWARD ENCODER PID FOR 2 BLOCKS**************************************************
  Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
  while (Fdis > FrontWall1)
  {
    PIDForward();
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    }
  StopMotors();
  delay(delayTime);

//33RD TURN (CW)***************************************************************
  turnStartCW = encoder_countLEFT; 
  while(encoder_countLEFT < (turnStartCW + 500)){
    RotateCW();
    Serial.print("LEFT: ");
    Serial.println(encoder_countLEFT);
  }
  StopMotors();
  delay(delayTime);

//FINAL MOVE, PID FORWARD FOR 3 BLOCKS THEN COMPLETE STOP, END OF PHASE B**********************

  int starttime1 = millis();
  int endtime1 = starttime1;
  while ((endtime1 - starttime1) <=2500) 
  {
    LeftForward();
    int loopcount1 = loopcount1+1;
    endtime1 = millis();
    } 
  StopMotors();
  while(1){}
  
}
