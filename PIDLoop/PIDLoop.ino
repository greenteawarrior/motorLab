// Emily Wang and Claire Diehl
// POE Lab 2 : Controlling a motor

// Stepper motor things (shoutout to Adafruit and their "StepperTest" example code)
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_StepperMotor *myMotor = AFMS.getStepper(512, 1);

// Potentiometer things
const int potPin = 3;    // this is the input pin for the potentiometer
const int potStop0 = 0; //potStop0 and potStop1 is the movable angle range when turning potentiometer
const int potStop1 = 270;
const float p2g = 0.263; // = 270/1023 converstion factor from potentiometer measurement (potVal) to degrees
float potVal = 0;       // variable store the potentiometer's analogRead() measurement 

// Angular position things
int actualAngle; //potVal converted to angles (degrees); this is the current angular position of the motor
int desiredAngle; //specified by user, any number between 130 and 270
//change back to const int if desired

// Motor movement variables
boolean motorDir; //FORWARD or BACKWARD
int error; // subtraction: desiredAngle - currentAngle

float stepsToNextAngleProp; //proportional control calculation
float stepsToNextAngleDer; //derivative control calculation
float stepsToNextAngle;

// Control constants
float kP = .5; //proportional constant term
float kD = 50; //derivative constant term

// Derivative Control Terms
float errorArray[4]={0,0,0,0};
float errorDerivative;
float total;
int i; //for loop things
long oldTime;
long newTime;
long passedTime;
long currentTime;

void setup() {
  Serial.begin(9600);
  Serial.println('Beginning');
  
  AFMS.begin();  // create with the default frequency 1.6KHz
  myMotor->setSpeed(10);  // 10 rpm   
  
  potVal = analogRead(potPin);   
  actualAngle = potVal * p2g;
  Serial.println(actualAngle);
  Serial.println("");
  desiredAngle = actualAngle + 90;
}

void loop() {
  // Read value of potentiometer and convert it to angles and obtain an error value.
  potVal = analogRead(potPin);   
  actualAngle = potVal * p2g;
  error = desiredAngle - actualAngle;
  
  // Proportional Control
  stepsToNextAngleProp = kP * error;
  stepsToNextAngleProp = abs(stepsToNextAngleProp);
  
  // Derivative Control
  newTime=millis();
  passedTime = newTime - oldTime;
  if (passedTime >= 20){
    errorArray[0]=errorArray[1];
    errorArray[1]=errorArray[2];
    errorArray[2]=errorArray[3];
    errorArray[3]=error;
    Serial.println("ErrorArray");
    Serial.println(errorArray[0]);
    Serial.println(errorArray[1]);
    Serial.println(errorArray[2]);
    Serial.println(errorArray[3]);
    Serial.println("");
    for (i=0;i<4;i++){
      total=total+errorArray[i];
      //Serial.println("arrayTotal");
      //Serial.println(total);
    oldTime = newTime;
    }
    errorDerivative=total/(400);
    total = 0;
  }
  stepsToNextAngleDer = kD * errorDerivative;
  stepsToNextAngleDer = abs(stepsToNextAngleDer);
  
  // Add the proportional and derivative things
  stepsToNextAngle = stepsToNextAngleProp + stepsToNextAngleDer;
 
  // Move the motor!!
  if (error >= 0){
    myMotor->step(stepsToNextAngle, FORWARD, SINGLE); 
    Serial.println("FORWARD");
  }

  else if (error <= 0) {
    myMotor->step(stepsToNextAngle, BACKWARD, SINGLE);  
    Serial.println("BACKWARD");
  }
    
  else if (error == 0) {
    myMotor->step(0, BACKWARD, SINGLE);      
  }
  
  // Print everything!
  currentTime = millis();
  Serial.println("currentTime:");
  Serial.println(currentTime);
  
  //Serial.println("potVal:");
  //Serial.println(potVal);
  Serial.println("actualAngle");
  Serial.println(actualAngle);
  //Serial.println("desiredAngle");
  //Serial.println(desiredAngle); //currently 285
  //Serial.println("error");
  //Serial.println(error);
  //Serial.println("stepsToNextAngleProp:");
  //Serial.println(stepsToNextAngleProp);
  //Serial.println("stepsToNextAngleDer:");
  //Serial.println(stepsToNextAngleDer);
  //Serial.println("stepsToNextAngle:");
  //Serial.println(stepsToNextAngle);
  Serial.println();
  
}
 

