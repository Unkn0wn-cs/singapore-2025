#include <Arduino.h>
#include <AFMotor.h>
#include <QuadratureEncoder.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "C:\Users\samue\OneDrive\Documents\GitHub\singapore25\Codes\Movement\Encoder\Ecoder measured movement\lib\move\move.h"
#include <Pixy2.h>

unsigned long lastMilli = 0;

const int enable34 = 23; // L293D pin 9
const int input3   = 41; // L293D pin 10
const int input4   = 31; // L293D pin 15

AF_DCMotor motor1(1); // Motor 1 on the Adafruit Motor Shield
AF_DCMotor motor2(2); // Motor 2 on the Adafruit Motor Shield
AF_DCMotor motor3(3); // Motor 3 on the Adafruit Motor Shield
AF_DCMotor motor4(4); // Motor 4 on the Adafruit Motor Shield

Encoders leftEncoder(A15, A13);	// Create an Encoder object name leftEncoder, using digitalpin 2 & 3
Encoders rightEncoder(A14 , A12); // Encoder object name rightEncoder using analog pin A0 and A1 

Move move(motor1, motor2, motor3, motor4, leftEncoder, rightEncoder);

int pwm1 = 220;
int pwm2 = 230;
int pwm3 = 230;
int pwm4 = 220;

void setup() {
  Serial.begin(9600);

  pinMode(enable34, OUTPUT);
  pinMode(input3, OUTPUT);
  pinMode(input4, OUTPUT);

  digitalWrite(enable34, HIGH); // Enable motor driver    

    digitalWrite(input3, LOW);
  digitalWrite(input4, HIGH);
}

void loop() {
  int ppr = 800;
  static int cases = 0;
  move.begin(pwm1,pwm2,pwm3,pwm4);

  // put your main code here, to run repeatedly:
// switch(cases){
//   case 0:
//     if (move.accelerateForward(move.mmToPulses(1000, 60, ppr), 210, 225, 20, 40, 0, 1)) cases++;
//     break;
//   case 1:
//     if(move.stopForMillis(500))cases++;
//     break;
//   case 2:
//     if (move.accelerateBackward(move.mmToPulses(1010, 60, ppr), 210, 225, 20, 40, 0, 1)) cases++;
//     break;
//   case 3:
//     if(move.stopForMillis(500)) cases = 10;
//     break;
//   case 4:
//     if(move.accelerateRight(move.mmToPulses(500, 60, ppr), 210, 230, 80)) cases++;
//     break;
//   case 5:
//     if(move.stopForMillis(500)) cases++;
//     break;
//   case 6:
//     if(move.accelerateLeft(move.mmToPulses(500, 60, ppr), 210, 230, 80)) cases++;
//     break;
//   case 7:
//     if (move.stopForMillis(500)) 
//     break;
// }
switch(cases){
  case 0:
    if (move.forward(move.mmToPulses(1000, 60, ppr))) cases++;
    break;
  case 1:
    if(move.stopForMillis(500))cases++;
    break;
  case 2:
    if (move.backward(move.mmToPulses(1010, 60, ppr))) cases++;
    break;
  case 3:
    if(move.stopForMillis(500)) cases = 10;
    break;
  case 4:
    if(move.accelerateRight(move.mmToPulses(500, 60, ppr), 210, 230, 80)) cases++;
    break;
  case 5:
    if(move.stopForMillis(500)) cases++;
    break;
  case 6:
    if(move.accelerateLeft(move.mmToPulses(500, 60, ppr), 210, 230, 80)) cases++;
    break;
  case 7:
    if (move.stopForMillis(500)) 
    break;
}
// switch(cases){
//   // case 0:
//   //   if (move.accelerateForward(move.mmToPulses(500, 60, ppr), 150, 230, 80)) cases++;
//   //   break;
//   // case 1:
//   //   if(move.stopForMillis(500))cases++;
//   //   break;
//   // case 2:
//   //   if(move.accelerateBackward(move.mmToPulses(500, 60, ppr), 150, 230, 80)) cases++;
//   //   break;
//   // case 3:
//   //   if(move.stopForMillis(500)) cases++;
//   //   break;
//   case 4:
//     if(move.accelerateRight(move.mmToPulses(500, 60, ppr), 150, 230, 80, 40, 160, 0.7, 250, 250, 250, 250)) cases++;
//     break;
//   case 5:
//           if(move.stopForMillis(500)) cases++;
//     break;
//   case 6:
//     if(move.accelerateLeft(move.mmToPulses(500, 60, ppr), 150, 230, 80)) cases++;
//     break;
//   case 7:
//     if (move.stopForMillis(500)) 
//     break;
// }
  
  if(millis()-lastMilli > 50){ 
    
    long currentLeftEncoderCount = leftEncoder.getEncoderCount();
    long currentRightEncoderCount = rightEncoder.getEncoderCount();
    

    Serial.print(currentLeftEncoderCount / ppr);
    Serial.print(" , ");
    Serial.println(currentRightEncoderCount / ppr);
    
    lastMilli = millis();
    // if (currentLeftEncoderCount > currentRightEncoderCount + 1 && pwm1<240 && pwm2>200){
    //   pwm1 ++;
    //   pwm2 --;
    //   pwm3 --;
    //   pwm4 ++;
    // }else if (currentLeftEncoderCount < currentRightEncoderCount - 1 && pwm2<240 && pwm1>200){
    //   pwm1 --;
    //   pwm2 ++;
    //   pwm3 ++;
    //   pwm4 --;
    // }
  }
}
