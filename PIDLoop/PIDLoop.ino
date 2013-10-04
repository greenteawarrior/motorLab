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

float actualAngle; //potVal converted to angles (degrees); this is the current angular position of the motor
float desiredAngle = 100; //specified by user
boolean motorDir; //FORWARD or BACKWARD
float error; // subtraction: desiredAngle - currentAngle

void setup() {
  Serial.begin(9600);
  Serial.println('Beginning');
  
  AFMS.begin();  // create with the default frequency 1.6KHz
  myMotor->setSpeed(10);  // 10 rpm   
  
  
}

void loop() {
  potVal = analogRead(potPin);    // read value from potentiometer
  Serial.println("potVal:");
  Serial.println(potVal);
  
  actualAngle = potVal * p2g;
  error = desiredAngle - actualAngle;
  Serial.println("actualAngle");
  Serial.println(actualAngle);
  Serial.println("desiredAngle");
  Serial.println(desiredAngle);
  Serial.println("error");
  Serial.println(error);
  Serial.println();
  
  
  

}
 
