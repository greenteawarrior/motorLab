// Emily Wang and Claire Diehl
// POE Lab 2 : Controlling a motor


// Potentiometer things
const int potPin = 3;    // this is the input pin for the potentiometer
int potVal = 0;       // variable store the voltage from the potentiometer


// control loop variables - do we want integers or floats?
int potPos; //position of potentiometer
int motorPos; //potPos converted into angles (degrees or radians?)
int error; //subtraction: desired angle - current angle
int Kp; //proportional constant term
int angV; //instantaneous angular velocity of the motor
int Kd; //derivative constant term
int PDsum; //sum: propotional gain + derivative gain


void setup() {
  Serial.begin(9600);
}

void loop() {
  val = analogRead(potPin);    // read value from potentiometer
  Serial.println(val);
}
 
