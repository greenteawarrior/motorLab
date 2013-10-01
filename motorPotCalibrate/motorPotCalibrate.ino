// still a work in progress

// Emily Wang and Claire Diehl
// POE Lab 2 : Controlling a motor

// Calibrating the motor and potentiometer
// aka. mapping potentiometer voltages to 
// motor position (angles in degrees)

// Potentiometer things
const int potPin = 3;    // this is the input pin for the potentiometer
int potVal = 0;       // variable store the voltage from the potentiometer

// Motor position things
int motorStep = 100; //vary this for each trial to get calibration data


// Stepper motor things 
// (shoutout to Adafruit and their "StepperTest" example code)
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Connect a stepper motor with 512 steps per 
// revolution (1.8 degree) to motor port #1 (M3 and M4)
Adafruit_StepperMotor *myMotor = AFMS.getStepper(512, 1);

void setup() {
  Serial.begin(9600);
  Serial.println('Beginning');
  
  AFMS.begin();  // create with the default frequency 1.6KHz
  myMotor->setSpeed(10);  // 10 rpm   
  
}

void loop() {
  potVal = analogRead(potPin);    // read value from potentiometer
  Serial.println(potVal);
  
  //Single coil steps
  myMotor->step(motorStep, FORWARD, SINGLE); 
  myMotor->step(motorStep, BACKWARD, SINGLE);
  Serial.println('motorStep'); 
  Serial.println('Record the angle!');
  
  //FIGURE OUT WHICH DIRECTION IS WHICH (forwards or backwards)
  //print out a compass to put on to the output shaft
