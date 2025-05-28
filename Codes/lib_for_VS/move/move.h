#pragma once
#include <AFMotor.h>
#include <QuadratureEncoder.h>

class Move {
  public:
    Move(AF_DCMotor& m1, AF_DCMotor& m2, AF_DCMotor& m3, AF_DCMotor& m4,
         Encoders& encLeft, Encoders& encRight)
      : motor1(m1), motor2(m2), motor3(m3), motor4(m4),
        encoderLeft(encLeft), encoderRight(encRight) {}

    void begin() {
      // Set initial speeds if needed
      motor1.setSpeed(200);
      motor2.setSpeed(200);
      motor3.setSpeed(200);
      motor4.setSpeed(200);
    }

    // Call before starting a new movement
    void startMove() {
      startLeft = encoderLeft.getEncoderCount();
      startRight = encoderRight.getEncoderCount();
      moving = true;
    }

    // Forward
    bool forward(long pulses) {
      if (!moving) startMove();
      setMotors(FORWARD, FORWARD, FORWARD, FORWARD);
      return checkDone(pulses);
    }

    // Backward
    bool backward(long pulses) {
      if (!moving) startMove();
      setMotors(BACKWARD, BACKWARD, BACKWARD, BACKWARD);
      return checkDone(pulses);
    }

    // Left (strafe)
    bool left(long pulses) {
      if (!moving) startMove();
      setMotors(BACKWARD, FORWARD, FORWARD, BACKWARD);
      return checkDone(pulses);
    }

    // Right (strafe)
    bool right(long pulses) {
      if (!moving) startMove();
      setMotors(FORWARD, BACKWARD, BACKWARD, FORWARD);
      return checkDone(pulses);
    }

    // Diagonal: Forward-Left
    bool forwardLeft(long pulses) {
      if (!moving) startMove();
      setMotors(RELEASE, FORWARD, FORWARD, RELEASE);
      return checkDone(pulses);
    }

    // Diagonal: Forward-Right
    bool forwardRight(long pulses) {
      if (!moving) startMove();
      setMotors(FORWARD, RELEASE, RELEASE, FORWARD);
      return checkDone(pulses);
    }

    // Diagonal: Backward-Left
    bool backwardLeft(long pulses) {
      if (!moving) startMove();
      setMotors(RELEASE, BACKWARD, BACKWARD, RELEASE);
      return checkDone(pulses);
    }

    // Diagonal: Backward-Right
    bool backwardRight(long pulses) {
      if (!moving) startMove();
      setMotors(BACKWARD, RELEASE, RELEASE, BACKWARD);
      return checkDone(pulses);
    }

    // Rotation (not encoder regulated)
    void rotateCW() {
      setMotors(FORWARD, BACKWARD, FORWARD, BACKWARD);
      moving = false;
    }
    void rotateCCW() {
      setMotors(BACKWARD, FORWARD, BACKWARD, FORWARD);
      moving = false;
    }

    // Stop all motors
    void stop() {
      setMotors(RELEASE, RELEASE, RELEASE, RELEASE);
      moving = false;
    }

  private:
    AF_DCMotor& motor1;
    AF_DCMotor& motor2;
    AF_DCMotor& motor3;
    AF_DCMotor& motor4;
    Encoders& encoderLeft;  // Motor 3
    Encoders& encoderRight; // Motor 4

    long startLeft = 0;
    long startRight = 0;
    bool moving = false;

    void setMotors(uint8_t m1, uint8_t m2, uint8_t m3, uint8_t m4) {
      motor1.run(m1);
      motor2.run(m2);
      motor3.run(m3);
      motor4.run(m4);
    }

    bool checkDone(long pulses) {
      long deltaLeft = abs(encoderLeft.getEncoderCount() - startLeft);
      long deltaRight = abs(encoderRight.getEncoderCount() - startRight);
      if (deltaLeft >= pulses && deltaRight >= pulses) {
        stop();
        return true;
      }
      return false;
    }
};