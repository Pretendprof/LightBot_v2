#include <AFMotor.h>

/*
This is my first attempt to create a light controled robot
It is guided by 3 photoresistors coupled to a left and right motor
 
 */

// These constants won't change:
const int sensorPinC = A0;    // Center Sensor
const int sensorPinL = A1;    // Left Sensor
const int sensorPinR = A2;    // Right Sensor
const int led1 = A3;
const int led2 = A4;

//const int MotorPinL = 9;      // pin that the Left motor is attached to
//const int MotorPinR = 3;      // pin that the Right motor is attached to
AF_DCMotor motor4(3);          // pin that the Left motor is attached to
AF_DCMotor motor2(2, MOTOR12_1KHZ);  // pin that the Right motor is attached to


// variables:
// Sensor values for Center
int sensorValueC = 0;         
int sensorMinC = 1023;        
int sensorMaxC = 0;           

// Sensor values for Left
int sensorValueL = 0;         
int sensorMinL = 1023;        
int sensorMaxL = 0;   

// Sensor values for Right
int sensorValueR = 0;         
int sensorMinR = 1023;        
int sensorMaxR = 0;   

// Motor values
int MotorValueL;
int MotorValueR;
int sensorVelocityL;
int sensorVelocityR;
int sensorMemL;
int sensorMemR;
int sensorValueCR;
int sensorValueCL;

void setup() {
  Serial.begin(9600);
  // turn on LED to signal the start of the calibration period:
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  //pinMode(led1, OUTPUT);
  //pinMode(led2, OUTPUT);
  pinMode(sensorPinC, INPUT);
  pinMode(sensorPinL, INPUT);
  pinMode(sensorPinR, INPUT);
  
  //pinMode(MotorPinL, OUTPUT);
  //pinMode(MotorPinR, OUTPUT);


  // calibrate during the first five seconds 
  while (millis() < 5000) {
    //Calibrate Center sensor
    sensorValueC = analogRead(sensorPinC);
    // record the maximum sensor value
    if (sensorValueC > sensorMaxC) {
      sensorMaxC = sensorValueC;
    }
    // record the minimum sensor value
    if (sensorValueC < sensorMinC) {
      sensorMinC = sensorValueC;
    }
   
    //Calibrate Left sensor
    sensorValueL = analogRead(sensorPinL);
    // record the maximum sensor value
    if (sensorValueL > sensorMaxL) {
      sensorMaxL = sensorValueL;
    }
    // record the minimum sensor value
    if (sensorValueL < sensorMinL) {
      sensorMinL = sensorValueL;
    }
   
    //Calibrate Right sensor
    sensorValueR = analogRead(sensorPinR);
    // record the maximum sensor value
    if (sensorValueR > sensorMaxR) {
      sensorMaxR = sensorValueR;
    }
    // record the minimum sensor value
    if (sensorValueR < sensorMinR) {
      sensorMinR = sensorValueR;
    }
  }

  // signal the end of the calibration period
  digitalWrite(13, LOW);

  //Set sensor mem value for finding x dot
  sensorMemL = analogRead(sensorPinL);
  // apply the calibration to the Left sensor reading
  sensorMemL = map(sensorMemL, sensorMinL, sensorMaxL, 0, 255);
  // in case the sensor value is outside the range seen during calibration
  sensorMemL = constrain(sensorMemL, 0, 255);
  
  sensorMemR = analogRead(sensorPinR);
  sensorMemR = map(sensorMemR, sensorMinR, sensorMaxR, 0, 255);
  // in case the sensor value is outside the range seen during calibration
  sensorMemR = constrain(sensorMemR, 0, 255);


  // turn on motor
  motor4.setSpeed(150);
  motor2.setSpeed(150);
 
  motor4.run(RELEASE);
  motor2.run(RELEASE); 
 
 //digitalWrite(led1, HIGH);
 //digitalWrite(led2, HIGH);
  //MotorValueL = 0;
  //MotorValueR = 0;
}

void loop() {
  // read the sensors:
  sensorValueC = analogRead(sensorPinC);
  sensorValueL = analogRead(sensorPinL);
  sensorValueR = analogRead(sensorPinR);
  
  sensorValueCR = sensorValueC;
  sensorValueCL = sensorValueC;

  // apply the calibration to the Center sensor reading
  sensorValueCL = map(sensorValueC, sensorMinC, sensorMaxC, 255, -255);
  // in case the sensor value is outside the range seen during calibration
  sensorValueCL = constrain(sensorValueCL, 255, -255);
  
  // apply the calibration to the Left sensor reading
  sensorValueL = map(sensorValueL, sensorMinL, sensorMaxL, 0, 255);
  // in case the sensor value is outside the range seen during calibration
  sensorValueL = constrain(sensorValueL, 0, 255);
  Serial.print(sensorValueL);
  Serial.print("\t");
  Serial.print(sensorMemL);
  Serial.print("\t");
  
   // apply the calibration to the Right sensor reading
  sensorValueR = map(sensorValueR, sensorMinR, sensorMaxR, 0, 255);
  
  // in case the sensor value is outside the range seen during calibration
  sensorValueR = constrain(sensorValueR, 0, 255);
  Serial.print(sensorValueR);
  Serial.print("\t");
  Serial.print(sensorMemR);
  Serial.print("\t");
  
    //Calculate Center Sensor Value:
   
 
  // Calculate Left Motor speed
  sensorVelocityL = (sensorValueL - sensorMemL);
  Serial.print(sensorVelocityL);
  Serial.print("\t");
  
  MotorValueL = (((sensorValueL * sensorVelocityL) / 2) + (2 * (sensorValueL - sensorValueR)) / 10) + (sensorValueCL); //Consider adding MotorValueR/2 to constant at end)
  //MotorValueL = abs(MotorValueL);
  Serial.print(MotorValueL);
  Serial.print("\t");
  MotorValueL = constrain(MotorValueL, -5000, 15000);
  
  if (MotorValueL >= 4000)
    {
      motor4.run(FORWARD);
      motor4.setSpeed(255);
      delay(5);
      motor4.run(RELEASE);
      MotorValueL = map(MotorValueL, 4000, 15000, 0, 255);
      MotorValueL = constrain(MotorValueL, 150, 255);
      //Serial.print(MotorValueL);
      //Serial.print("\t");  
      motor4.run(FORWARD);
      motor4.setSpeed(MotorValueL);
    }
  
 else //if(MotorValueL < 4000)
    {
      motor4.run(BACKWARD);
      motor4.setSpeed(255);
      delay(5);
      motor4.run(RELEASE);
     
      MotorValueL = map(MotorValueL, -5000, 4000, 0, 255);
      MotorValueL = constrain(MotorValueL, 150, 255);
      //Serial.print(MotorValueL);
      //Serial.print("\t");  
      motor4.run(BACKWARD);
      motor4.setSpeed(MotorValueL);
    
  }
    
  /*
  Serial.print(MotorValueL);
  Serial.print("\t");
  MotorValueL = map(MotorValueL, -5000, 15000, 0, 255);
  //Serial.print(MotorValueL);
  //Serial.print("\t");
  MotorValueL = constrain(MotorValueL, 80, 160);
  Serial.print(MotorValueL);
  Serial.print("\t");  
  motor4.run(FORWARD);
  motor4.setSpeed(MotorValueL);  
  */
  sensorValueCR = map(sensorValueC, sensorMinC, sensorMaxC, -255, 255);
  sensorValueCR = constrain(sensorValueCR, -255, 255);
 
  sensorVelocityR = (sensorValueR - sensorMemR);
  Serial.print(sensorVelocityR);
  Serial.print("\t");
  // Calculate Right Motor speed
  MotorValueR = (((sensorValueR * sensorVelocityR) / 2) + (2 * (sensorValueR - sensorValueL)) / 10) + (sensorValueCR);
  Serial.println(MotorValueR);
  MotorValueR = constrain(MotorValueR, -5000, 15000);
 
  //MotorValueL = (((sensorValueL * sensorVelocityL) / 2) + (2 * (sensorValueL - sensorValueR)) / 10) + (sensorValueCL);
  
  if (MotorValueR >= 4000)
    {
      motor2.run(BACKWARD);  
      motor2.setSpeed(255);
      delay(5);
      motor2.run(RELEASE);
      MotorValueR = map(MotorValueR, 4000, 15000, 0, 255);
      MotorValueR = constrain(MotorValueR, 150, 255);  
      motor2.run(BACKWARD);  
      motor2.setSpeed(MotorValueR);
    }
   
 else //(MotorValueR < -5000)
    {
      motor2.run(FORWARD); 
      motor2.setSpeed(255);
      delay(5);
      motor2.run(RELEASE);
            
      MotorValueR = map(MotorValueR, -5000, 4000, 0, 255);
      MotorValueR = constrain(MotorValueR, 150, 255);  
      motor2.run(FORWARD);  
      motor2.setSpeed(MotorValueR);
      
    }
 
  /*
  Serial.print(MotorValueR);
  Serial.print("\t");
  MotorValueR = map(MotorValueR, -5000, 15000, 0, 255);
  //Serial.print(MotorValueR);
  //Serial.print("\t");
  
  MotorValueR = constrain(MotorValueR, 80, 160);  
  Serial.print(MotorValueR);
  Serial.print("\t");
  Serial.print(sensorValueL);
  Serial.print("\t");
  Serial.print(sensorValueR);
  Serial.println("\t");
  
  motor2.run(BACKWARD);
  motor2.setSpeed(MotorValueR); 
  */
  
/*
  // Calculate Left Motor speed
  MotorValueL = (sensorValueL + sensorValueC) + (sensorValueL - sensorValueR) - sqrt(MotorValueR); //Consider adding MotorValueR/2 to constant at end)
  MotorValueL = abs(MotorValueL);
  MotorValueL = constrain(MotorValueL, 120, 200); 
  motor4.run(FORWARD);
  motor4.setSpeed(MotorValueL);  
  //analogWrite(MotorPinL, MotorValueL);
  
  //Serial.print(MotorValueL);
  //Serial.print("\t");
  
  //delay(50); 
  
  // Calculate Right Motor speed
  MotorValueR = (sensorValueR + sensorValueC) + (sensorValueR - sensorValueL) - sqrt(MotorValueL);
  MotorValueR = abs(MotorValueR);
  MotorValueR = constrain(MotorValueR, 120, 200);  
  motor2.run(BACKWARD);
  motor2.setSpeed(MotorValueR); 
*/
  //analogWrite(MotorPinR, MotorValueR);
  /*
  Serial.print(sensorValueC);
  Serial.print("\t");
  Serial.print(sensorValueL);
  Serial.print("\t");
  Serial.print(sensorValueR);
  Serial.print("\t");
  Serial.print(MotorValueL);
  Serial.print("\t");
  Serial.println(MotorValueR);
  //Serial.print("\t");
  */
  
  
  sensorMemL = analogRead(sensorPinL);
  sensorMemL = map(sensorMemL, sensorMinL, sensorMaxL, 0, 255);
  // in case the sensor value is outside the range seen during calibration
  sensorMemL = constrain(sensorMemL, 0, 255);
  
  sensorMemR = analogRead(sensorPinR);
  sensorMemR = map(sensorMemR, sensorMinR, sensorMaxR, 0, 255);
  // in case the sensor value is outside the range seen during calibration
  sensorMemR = constrain(sensorMemR, 0, 255);
  delay(150);
}
