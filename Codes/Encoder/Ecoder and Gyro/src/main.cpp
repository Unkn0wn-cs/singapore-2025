#include <Arduino.h>
#include <AFMotor.h>
#include <QuadratureEncoder.h>
#include "Move.h"

// Define motors & encoders
AF_DCMotor motor1(1); // Motor 1 on the Adafruit Motor Shield
AF_DCMotor motor2(2); // Motor 2 on the Adafruit Motor Shield
AF_DCMotor motor3(3); // Motor 3 on the Adafruit Motor Shield
AF_DCMotor motor4(4); // Motor 4 on the Adafruit Motor Shield
Encoders encoderLeft(A15, A13);  // Motor 3
Encoders encoderRight(A14, A12); // Motor 4

Move move(motor1, motor2, motor3, motor4, encoderLeft, encoderRight);

const int pwm = 255; // Maximum PWM value (full speed)
const unsigned long interval = 1000; // Interval for turning motors on and off (in milliseconds)
unsigned long lastMilli = 0;

void setup() {
  Serial.begin(9600);
  motor1.setSpeed(pwm);
  motor2.setSpeed(pwm);
  motor3.setSpeed(pwm);
  motor4.setSpeed(pwm);
  move.begin();
}
 
void loop() {
  static int state = 0;
  const long PULSES = 60000;

  switch (state) {
    case 0:
      if (move.forward(PULSES)) state++;
      break;
    case 1:
      if (move.right(PULSES)) state++;
      break;
    case 2:
      if (move.backward(PULSES)) state++;
      break;
    case 3:
      if (move.left(PULSES)) state++;
      break;
    case 4:
      move.rotateCW(); // Not encoder regulated, just for demonstration
      delay(1000);
      move.stop();
      state = 0;
      break;
  }
}