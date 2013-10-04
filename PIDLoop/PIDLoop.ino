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
const float p2g = 3.79; // = 1023/270 converstion factor from potentiometer measurement (potVal) to degrees
int potVal = 0;       // variable store the potentiometer's analogRead() measurement 

int actualAngle; //potVal converetd to angles (degrees)
int desiredAngle; //specified by user
int error; // subtraction: desiredAngle - currentAngle

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
  
  
  
  

}
 
