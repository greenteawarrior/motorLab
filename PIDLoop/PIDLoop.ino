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

int actualAngle; //potVal converted to angles (degrees); this is the current angular position of the motor
const int desiredAngle = 200; //specified by user, any number between 130 and 270
boolean motorDir; //FORWARD or BACKWARD
int error; // subtraction: desiredAngle - currentAngle
float stepsToNextAngle; //

float kP = .5; //proportional constant term


void setup() {
  Serial.begin(9600);
  Serial.println('Beginning');
  
  AFMS.begin();  // create with the default frequency 1.6KHz
  myMotor->setSpeed(10);  // 10 rpm   
}

void loop() {
  potVal = analogRead(potPin);    // read value from potentiometer
  
  actualAngle = potVal * p2g;
  error = desiredAngle - actualAngle;
  stepsToNextAngle = kP * error;
  stepsToNextAngle = abs(stepsToNextAngle);
  
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
  
  Serial.println("potVal:");
  Serial.println(potVal);
  Serial.println("actualAngle");
  Serial.println(actualAngle);
  Serial.println("desiredAngle");
  Serial.println(desiredAngle);
  Serial.println("error");
  Serial.println(error);
  Serial.println("stepsToNextAngle:");
  Serial.println(stepsToNextAngle);
  Serial.println();
  
}
 
