#include <Servo.h>

//Instantiate the servo objects
Servo J1;
Servo J2;
Servo J3;
Servo J4;
Servo J5;
Servo J6;

//Set the pins that each servo is plugged into
const int J1_pin = 2;
const int J2_pin = 3;
const int J3_pin = 4;
const int J4_pin = 5;
const int J5_pin = 6;
const int J6_pin = 7;

//Set the home config servo positions
// Array of {minValue, homePosValue, maxValue}
const int J1_pos[3] = {0, 90, 180};
const int J2_pos[3] = {40, 90, 180};
const int J3_pos[3] = {20, 120, 160};
const int J4_pos[3] = {0, 95, 180};
const int J5_pos[3] = {0, 85, 180};
const int J6_pos[3] = {0, 78, 180};

int jointQ[6] = {0, 0, 0, 0, 0, 0};

void setup() {
  // put your setup code here, to run once:
  J1.attach(J1_pin);
  J2.attach(J2_pin);
  J3.attach(J3_pin);
  J4.attach(J4_pin);
  J5.attach(J5_pin);
  J6.attach(J6_pin);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  J1.write(min(max(J1_pos[0], (jointQ[0] + J1_pos[1])), J1_pos[2]));
  J2.write(min(max(J2_pos[0], (jointQ[1] + J2_pos[1])), J2_pos[2]));
  J3.write(min(max(J3_pos[0], (jointQ[2] + J3_pos[1])), J3_pos[2]));
  J4.write(min(max(J4_pos[0], (jointQ[3] + J4_pos[1])), J4_pos[2]));
  J5.write(min(max(J5_pos[0], (-jointQ[4] + J5_pos[1])), J5_pos[2]));
  J6.write(min(max(J6_pos[0], (jointQ[5] + J6_pos[1])), J6_pos[2]));

  delay(100);
}
