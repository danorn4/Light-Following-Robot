#include "Arduino.h"
#define TRIGGER_PIN 12
#define ECHO_PIN 10



void setup() {
  Serial.begin(9600); // set up serial connection
  pinMode(TRIGGER_PIN, OUTPUT);   
// pulse sent out through TRIGGER_PIN    
  pinMode(ECHO_PIN, INPUT); 
	// return signal read through ECHO_PIN
}

long getDurationRaw() {
  long duration;
  // Clear the TRIGGER_PIN
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);

  // Set the TRIGGER_PIN on HIGH state 
  // for 10 micro seconds
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  
  // Reads the ECHO_PIN, returns the sound 
  //wave travel time in microseconds
  duration = pulseIn(ECHO_PIN, HIGH, 10000);
  return(duration);
}

float getDistanceRaw() {
  float duration = (float)getDurationRaw();
  // duration is time for sonar to travel to 
  // object and back
  duration = duration / 2;  
  // divide by 2 for travel time to object 
  float c = 343;  // speed of sound in m/s
  c = c * 100 / 1e6;  
// speed of sound in cm/microseconds
  // Calculate the distance in centimeters
  float distance = duration * c;
  return(distance);
}

float getDistanceSmoothed() {
  static float distanceSmoothed = getDistanceRaw();
  float distance = getDistanceRaw();
  float alpha = 0.9; // alpha-filter constant
  if (distance != 0) {
// this is an example of a measurement gate:
// sensor returns a 0 when it times out 
// (i.e., no measurement) ignore those measurements
  
// alpha filter all good measurements     
distanceSmoothed = alpha*distanceSmoothed +(1-alpha)*distance;
  }
  return(distanceSmoothed);
}

void loop() {
float distance = getDistanceSmoothed();
Serial.println(distance);  
}


