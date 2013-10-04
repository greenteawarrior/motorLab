// Emily Wang and Claire Diehl
// POE Lab 2 : Controlling a motor

// Stepper motor things 
// (shoutout to Adafruit and their "StepperTest" example code)
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Connect a stepper motor with 512 steps per revolution (1.8 degree) to motor port #1
Adafruit_StepperMotor *myMotor = AFMS.getStepper(512, 1);


// Potentiometer things
const int potPin = 3;    // this is the input pin for the potentiometer
int potVal = 0;       // variable store the voltage from the potentiometer

// control loop variables - do we want integers or floats?
int actualAngle; //potPos converted into angles (degrees or radians?)
int desiredAngle;
int error; //subtraction: desired angle - current angle

int Kp; //proportional constant term
int angV; //instantaneous angular velocity of the motor
int Kd; //derivative constant term
int PDsum; //sum: propotional gain + derivative gain


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
 
