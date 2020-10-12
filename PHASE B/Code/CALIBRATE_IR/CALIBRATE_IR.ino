
//Distance variables
int Rdis = 0;
int Ldis = 0;
int Fdis = 0;
int Bdis = 0;
int Bdis2 = 0;

//Pin declaration for IR Sensors
#define LEFTSEN A3
#define RIGHTSEN A0
#define FRONTSEN A2
#define BACKSEN A1
#define BACKSEN2 A5


void setup(){
  Serial.begin(250000);

// IR SENSOR SETUP
  pinMode(FRONTSEN,INPUT);
  pinMode(LEFTSEN,INPUT);
  pinMode(RIGHTSEN,INPUT);
  pinMode(BACKSEN,INPUT);
  pinMode(BACKSEN2,INPUT);
}


//Main loop to print out distance

void loop() {
    Fdis = ((-1.3662*(analogRead(FRONTSEN)))+645.12);
    Rdis = ((-1.3662*(analogRead(RIGHTSEN)))+645.12);
    Ldis = ((-1.3662*(analogRead(LEFTSEN)))+645.12);
    Bdis = ((-1.3662*(analogRead(BACKSEN)))+645.12);
    Bdis2 = ((-1.3662*(analogRead(BACKSEN2)))+645.12);
    //Bdis = analogRead(BACKSEN);
    //Fdis = analogRead(FRONTSEN);
    //Rdis = analogRead(RIGHTSEN);
    //Ldis = analogRead(LEFTSEN);
    //Bdis = analogRead(BACKSEN);
    
    
    Serial.print("Front Distance: ");
    Serial.println(Fdis);
    Serial.print("Right Distance: ");
    Serial.println(Rdis);
    Serial.print("Left Distance: ");
    Serial.println(Ldis);
    Serial.print("Back Distance: ");
    Serial.println(Bdis);
    Serial.print("Back Distance 2: ");
    Serial.println(Bdis2);
 
    delay(100);
}
