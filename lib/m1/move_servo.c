#include "Arduino.h"
#include <Servo.h>  // loads the Servo library

// Servo pin
#define SERVO_PIN 9

// Parameters for servo control as well as instantiation
#define SERVO_START_ANGLE 90
#define SERVO_UP_LIMIT 180
#define SERVO_DOWN_LIMIT 0
static Servo myServo;

void MoveServo(); // forward declaration 


void setup() {
  Serial.begin(9600);  // set up serial connection at 9600 Baud

  //Set up servo
  myServo.attach(SERVO_PIN);
  myServo.write(SERVO_START_ANGLE);
}

void loop() { // Replace your old void loop with this
  // MoveServo(); // state machine to move servo back and forth
  
  // 0, 45, 90, 135, 180 degrees
  myServo.write(0); // set 
  delay(5000); 
  myServo.write(45); // set
  delay(5000); 
  myServo.write(90); // set
  delay(5000); 
  myServo.write(135); // set
  delay(5000); 
  myServo.write(180); // set
  delay(5000); 
  

}

void MoveServo() {
    static int state = 0;
    static int servoAngle = SERVO_START_ANGLE;
    Serial.println(servoAngle);
    switch(state) {
    case 0: // servo moving in positive direction
      servoAngle++; 
      if (servoAngle >= SERVO_UP_LIMIT) {
        state = 1;
      }
      break;
    case 1: // servo moving in negative direction
      servoAngle--; 
      if (servoAngle <= SERVO_DOWN_LIMIT) {
        state = 0;
      }
      break;
  }
  myServo.write(servoAngle); // send angle to the servo 
  // the .write() function expects an integer between 0 and 180 degrees
}