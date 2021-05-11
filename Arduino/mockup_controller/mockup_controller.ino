#include <ESP32Servo.h>

//Instantiate the servo objects
Servo J1;
Servo J2;
Servo J3;
Servo J4;
Servo J5;
Servo J6;

//Set the pins that each servo is plugged into
const int J1_pin = 18;
const int J2_pin = 32;
const int J3_pin = 33;
const int J4_pin = 25;
const int J5_pin = 26;
const int J6_pin = 27;


void setup() {
  // put your setup code here, to run once:
  J1.attach(J1_pin);
  J2.attach(J2_pin);
  J3.attach(J3_pin);
  J4.attach(J4_pin);
  J5.attach(J5_pin);
  J6.attach(J6_pin);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  J1.write(90);
}
