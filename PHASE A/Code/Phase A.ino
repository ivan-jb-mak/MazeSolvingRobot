#include <CytronMotorDriver.h>  //Library included to use Cytron motor shield

//Defining Varibles for Sensor Distances
int Rdis = 0;
int Ldis = 0;
int Fdis = 0;

//Pin declaration for IR Sensors and Motors
int RightDir = 4;
int RightEn = 5;
int LeftEn = 6;
int LeftDir = 7;
#define RIGHTSEN A1
#define LEFTSEN A2
#define FRONTSEN A3

int Setpoint = 56; //Setpoint value used for Wall Following PID

//PID gains, must tune and change depending on enviornment 
float Kp = 50;
float Ki = 0;
float Kd = 0;

//Defining variables for PID controller 
float Error = 0;
int Correction = 0;
float Integral = 0;
float Derivative = 0;
float LastError = 0;
int LeftTurnSpeed = 0;
int RightTurnSpeed = 0;
int BasePWM = 150;


//Setting Up Sensor and Motor Pins as Output/Input 
void setup(){
  
  Serial.begin(250000);

 //Motor Driver Pin Setup
  pinMode(RightEn,OUTPUT);
  pinMode(RightDir,OUTPUT);
  pinMode(LeftEn,OUTPUT);
  pinMode(LeftDir,OUTPUT);

// IR SENSOR SETUP
  pinMode(LEFTSEN,INPUT);
  pinMode(RIGHTSEN,INPUT);
  pinMode(FRONTSEN, INPUT);
}


//Main "loop" that runs once,  must start robot at the start of the maze to work
void loop(){

//First while loop to move robot forward (with WF_PID) until left IR sensor does not detect a wall

  Ldis = ((analogRead(LEFTSEN)*0.13)); //Read the left sensor distance
  while (Ldis > 45){ //while the left sensors reads that the distance is greater than the value 45, continue going forward with Wall following PID
    WF_PID(); 
    Ldis = ((analogRead(LEFTSEN)*0.13)); //Read the left sensor again
    }
    RotateCCW(); //when left distance is not greater than 45, excecute rotateCCW function
  
  Fdis = ((analogRead(FRONTSEN)*0.13));
  while (Fdis < 56){
    Forward();
    Fdis = ((analogRead(FRONTSEN)*0.13));
    }
    RotateCW();

  Fdis = ((analogRead(FRONTSEN)*0.13));
  while (Fdis < 56){
    Forward();
    Fdis = ((analogRead(FRONTSEN)*0.13));
    }
    RotateCCW();

   Fdis = ((analogRead(FRONTSEN)*0.13));
  while (Fdis < 56){
    WF_PID();
    Fdis = ((analogRead(FRONTSEN)*0.13));
    }
    RotateCCW();

  Fdis = ((analogRead(FRONTSEN)*0.13));
  while (Fdis < 56){
    WF_PID();
    Fdis = ((analogRead(FRONTSEN)*0.13));
    }
    RotateCW();

   Fdis = ((analogRead(FRONTSEN)*0.13));
  while (Fdis < 56){
    Forward();
    Fdis = ((analogRead(FRONTSEN)*0.13));
    }
    RotateCW2();
    
   Ldis = ((analogRead(LEFTSEN)*0.13));
  while (Ldis > 45){
    Forward();
    Ldis = ((analogRead(LEFTSEN)*0.13));
    }
    RotateCCW();

   Fdis = ((analogRead(FRONTSEN)*0.13));
  while (Fdis < 53){
    Forward();
    Fdis = ((analogRead(FRONTSEN)*0.13));
    }
    RotateCW1();

  Forward1();
    
   Fdis = ((analogRead(FRONTSEN)*0.13));
  while (Fdis < 56){
    WF_PID();
    Fdis = ((analogRead(FRONTSEN)*0.13));
    }
    RotateCW3();

  Forward1();

    Fdis = ((analogRead(FRONTSEN)*0.13));
  while (Fdis < 56){
    WF_PID();
    Fdis = ((analogRead(FRONTSEN)*0.13));
    }
    RotateCW();

  //Forward1();

    Rdis = ((analogRead(RIGHTSEN)*0.13));
  while (Rdis < 45){
    WF_PID();
    Rdis = ((analogRead(RIGHTSEN)*0.13));
    }
    RotateCW();

    Fdis = ((analogRead(FRONTSEN)*0.13));
  while (Fdis < 58){
    Forward();
    Fdis = ((analogRead(FRONTSEN)*0.13));
    }
    RotateCW();

   Forward1();

    Fdis = ((analogRead(FRONTSEN)*0.13));
  while (Fdis < 56){
    Forward();
    Fdis = ((analogRead(FRONTSEN)*0.13));
    }
    RotateCCW();

   Fdis = ((analogRead(FRONTSEN)*0.13));
  while (Fdis < 56){
    Forward();
    Fdis = ((analogRead(FRONTSEN)*0.13));
    }
    RotateCCW();

   Fdis = ((analogRead(FRONTSEN)*0.13));
  while (Fdis < 56){
    WF_PID();
    Fdis = ((analogRead(FRONTSEN)*0.13));
    }
    RotateCW();
}


//PID Wall Following function to call when driving along a long wall
void WF_PID() {
  
    Rdis = ((analogRead(RIGHTSEN)*0.13)); //Creating distance variable from analog output from IR sensor
    Error = Rdis - Setpoint; //Error variable for PID loop
    Integral = (Error + Integral); //Intergral Error for PID loop
    Derivative = (Error - LastError); //Derivative Error for PID loop

//Error is less than 0, rotate robot CW until straight
    if (Error < 0) {
    Correction = Kp * Error + Kd * Derivative + Ki * Integral; //Correction is the value for the output of PID controller
    
      LeftTurnSpeed = BasePWM - Correction;
      RightTurnSpeed = BasePWM + Correction;
        }

//Error is greater than 0, rotate robot CCW until straight
    else if (Error > 0) {
      int speed = Kp * Error + Kd * Derivative + Ki * Integral; //speed is the value for the output of the PID controller 

      LeftTurnSpeed = BasePWM - (speed);
      RightTurnSpeed = BasePWM + (speed);
    }

//Error is 0, continue robot forward 
    else {
      LeftTurnSpeed = BasePWM;
      RightTurnSpeed = BasePWM;
      }

//Printing output motors values to check if PID is working correctly 
    /*Serial.print("Left: ");
    Serial.println(LeftTurnSpeed);
    Serial.print("Right: ");
    Serial.println(RightTurnSpeed);*/

//Driving motors Forward based on PID values
    digitalWrite(4,HIGH);  
    analogWrite(5,LeftTurnSpeed);
    digitalWrite(7,HIGH);  
    analogWrite(6,RightTurnSpeed);
    
    LastError = Error; //recycling error term for integral and derivative
    //delay(100);
}

      
//Functions to call when driving motors Forward, Rotating CCW and CW

//Rotate clockwise
void RotateCW(){
   digitalWrite(4,HIGH);  
   analogWrite(5,100);
   digitalWrite(7,LOW);  
   analogWrite(6,100);
   delay(650);
   }
   
//Another Rotate clockwise 
void RotateCW1(){
   digitalWrite(4,HIGH);  
   analogWrite(5,100);
   digitalWrite(7,LOW);  
   analogWrite(6,100);
   delay(600);
}

//Another Rotate clockwise
void RotateCW2(){
   digitalWrite(4,HIGH);  
   analogWrite(5,100);
   digitalWrite(7,LOW);  
   analogWrite(6,100);
   delay(550);
}

//Another Rotate clockwise
void RotateCW3(){
   digitalWrite(4,HIGH);  
   analogWrite(5,100);
   digitalWrite(7,LOW);  
   analogWrite(6,100);
   delay(450);
}

//Rotate coutner clockwise
void RotateCCW(){
   digitalWrite(4,LOW);  
   analogWrite(5,100);
   digitalWrite(7,HIGH);  
   analogWrite(6,100);
   delay(650);
}

//Move Forward
void Forward(){
   digitalWrite(4,HIGH);  
   analogWrite(5,60);
   digitalWrite(7,HIGH);  
   analogWrite(6,65);
}

//Another Move Forward
void Forward1(){
   digitalWrite(4,HIGH);  
   analogWrite(5,100);
   digitalWrite(7,HIGH);  
   analogWrite(6,110);
   delay(100);
}





  
