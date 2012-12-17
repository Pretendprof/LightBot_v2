// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!

#include <AFMotor.h>

AF_DCMotor motor4(4);
AF_DCMotor motor2(2, MOTOR12_1KHZ);

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Motor test!");

  // turn on motor
  motor4.setSpeed(200);
  motor2.setSpeed(200);
 
  motor4.run(RELEASE);
  motor2.run(RELEASE);
}

void loop() {
  uint8_t i;
  
  Serial.print("tick");
  
  motor4.run(FORWARD);
 // for (i=0; i<255; i++) {
    motor4.setSpeed(150);  
    //delay(10);
 //}
 
 /*for (i=255; i!=0; i--) {
    motor4.setSpeed(i);  
    delay(10);
 }
 */
 motor2.run(BACKWARD);
 // for (i=0; i<255; i++) {
    motor2.setSpeed(150);  
    delay(300);
 //}
 /*
  for (i=255; i!=0; i--) {
    motor2.setSpeed(i);  
    delay(10);
 }
 /* 
  Serial.print("tock");

  motor.run(BACKWARD);
  for (i=0; i<255; i++) {
    motor.setSpeed(i);  
    delay(10);
 }
 
  for (i=255; i!=0; i--) {
    motor.setSpeed(i);  
    delay(10);
 }
 
  
*/

  motor4.run(RELEASE);
  motor2.run(RELEASE);
  delay(10);

  motor2.run(BACKWARD);
 // for (i=0; i<255; i++) {
    motor2.setSpeed(200);  
    delay(500);
    
  motor2.run(RELEASE);
    
  motor4.run(FORWARD);
 // for (i=0; i<255; i++) {
    motor4.setSpeed(200);  
    delay(500);
    
  Serial.print("tech");
  motor4.run(RELEASE);
 
  delay(500);
}
