#include <Arduino.h>
#include <AFMotor.h>
#include <QuadratureEncoder.h>
#include <Wire.h>
#include "C:\Users\samue\OneDrive\Documents\GitHub\singapore25\Codes\Movement\Encoder\Ecoder measured movement\lib\move\move.h"

// Define motors & encoders
AF_DCMotor motor1(1); // Motor 1 on the Adafruit Motor Shield
AF_DCMotor motor2(2); // Motor 2 on the Adafruit Motor Shield
AF_DCMotor motor3(3); // Motor 3 on the Adafruit Motor Shield
AF_DCMotor motor4(4); // Motor 4 on the Adafruit Motor Shield
Encoders encoderLeft(A15, A13);  // Motor 3
Encoders encoderRight(A14, A12); // Motor 4

Move move(motor1, motor2, motor3, motor4, encoderLeft, encoderRight);

const int pwm = 200; // Maximum PWM value (full speed)
const unsigned long interval = 1000; // Interval for turning motors on and off (in milliseconds)
unsigned long lastMilli = 0;

void setup() {
  Serial.begin(9600);
  move.begin(pwm, pwm, pwm, pwm); // Initialize motors with PWM values
}
 
void loop() {
  static int routine = 0;
  static int state = 0;
  // static int i = 0;
  // const long PULSES = 6000;
  static int mili = 500;
  int diameter = 60; // Diameter of the wheel in mm
  const long ppr = 1200; // Number of pulses for each movement step

  //move.mmToPulses(100.0, diameter, ppr);     ----------------------------         mm , diameter in mm, pulses per revolutio

  switch (routine) {// Routines --------------
    case 0: // Routine 0 ----------------------------------------------------
      switch (state) {
        case 0:
          if (move.forward(move.mmToPulses(300.0, diameter, ppr))) state++;
          break;
        case 1:
          if (move.stopForMillis(mili)) state++;
          break;
        case 2:
          if (move.backward(move.mmToPulses(300.0, diameter, ppr))) state++;
          break;
        case 3:
          if (move.stopForMillis(mili)) state++;
          break;
        case 4:
          routine = 1; // Move to next routine
          state = 0; // Reset state for the next routine
          break;
      }
    break;
    case 1: // Routine 1 ----------------------------------------------------
      switch (state) {  
      case 0:
        if (move.forward(move.mmToPulses(330.0, diameter, ppr))) state++;
        break;
      case 1:
        if (move.stopForMillis(mili)) state++;
        break;
      case 2:
        if (move.backward(move.mmToPulses(330.0, diameter, ppr))) state++;
        break;
      case 3:
        if (move.stopForMillis(mili)) state++;
        break;
      case 4:
        routine = 2; // Move to next routine
        state = 0; // Reset state for the next routine
        break;
      }
    break;
    case 2: // Routine 2 ----------------------------------------------------
      switch (state) {
      case 0:
        if (move.right(move.mmToPulses(180.0, diameter, ppr))) state++;
        break;
      case 1:
        if (move.stopForMillis(mili)) state++;
        break;
      case 2:
        if (move.forward(move.mmToPulses(300.0, diameter, ppr))) state++;
        break;
      case 3:
        if (move.stopForMillis(mili)) state++;
        break;
      case 4:
        routine = 3; // Move to next routine
        state = 0; // Reset state for the next routine
        break;
      }
    break;
    case 3: // Routine 3 ----------------------------------------------------
      switch (state) {
      case 0:
        if (move.backward(move.mmToPulses(310.0, diameter, ppr))) state++;
        break;
      case 1:
        if (move.stopForMillis(mili)) state++;
        break;
      case 2:
        if (move.left(move.mmToPulses(220.0, diameter, ppr))) state++;
        break;
      case 3:
        if (move.stopForMillis(mili)) state++;
        break;
      case 4:
        routine = 4; // Move to next routine
        state = 0; // Reset state for the next routine
        break;
      }
      break;
    case 4: // Routine 4 ----------------------------------------------------
      switch (state) {
      case 0:
        if (move.forward(move.mmToPulses(400.0, diameter, ppr))) state++;
        break;
      case 1:
        if (move.stopForMillis(mili)) state++;
        break;
      case 2:
        if (move.backward(move.mmToPulses(410.0, diameter, ppr))) state++;
        break;
      case 3:
        if (move.stopForMillis(mili)) state++;
        break;
      case 4:
        routine = 5; // Move to next routine
        state = 0; // Reset state for the next routine
        break;
      }
      break;
    case 5: // Routine 5 ----------------------------------------------------
      switch (state) {
      case 0:
        if (move.right(move.mmToPulses(180.0, diameter, ppr))) state++;
        break;
      case 1:
        if (move.stopForMillis(mili)) state++;
        break;
      case 2:
        if (move.forward(move.mmToPulses(400.0, diameter, ppr))) state++;
        break;
      case 3:
        if (move.stopForMillis(mili)) state++;
        break;
      case 4:
        if (move.backward(move.mmToPulses(410.0, diameter, ppr))) state++;
        break;
      case 5:
        if (move.stopForMillis(mili)) state++;
        break;
      case 6:
        if (move.left(move.mmToPulses(210.0, diameter, ppr))) state++;
        break;
      case 7:
        if (move.stopForMillis(mili)) state++;
        break;
      case 8:
        routine = 4; // Move to next routine
        state = 0; // Reset state for the next routine
        break;
      }
      break;
  }

    long currentLeftEncoderCount1 = encoderLeft.getEncoderCount();
    long currentRightEncoderCount1 = encoderRight.getEncoderCount();
    Serial.print((currentLeftEncoderCount1) / 600.0);   
    Serial.print(" , ");
    Serial.print((currentRightEncoderCount1) / 600.0);
    Serial.print(" , ");
    // Serial.print(state);
    Serial.print("\n");
}