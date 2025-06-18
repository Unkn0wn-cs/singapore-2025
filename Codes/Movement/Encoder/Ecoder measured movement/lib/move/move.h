#pragma once
#include <AFMotor.h>
#include <QuadratureEncoder.h>

class Move {
  public:
    Move(AF_DCMotor& m1, AF_DCMotor& m2, AF_DCMotor& m3, AF_DCMotor& m4,
         Encoders& encLeft, Encoders& encRight)
      : motor1(m1), motor2(m2), motor3(m3), motor4(m4),
        encoderLeft(encLeft), encoderRight(encRight) {}

    void begin(int pwm = 255, int pwm2 = 255, int pwm3 = 255, int pwm4 = 255) {
      // Set initial speeds if needed
      motor1.setSpeed(pwm);
      motor2.setSpeed(pwm2);
      motor3.setSpeed(pwm3);
      motor4.setSpeed(pwm4);
    } 

    // Call before starting a new movement
    void startMove() {
      startLeft = encoderLeft.getEncoderCount();
      startRight = encoderRight.getEncoderCount();
      moving = true; 
    }

    //Movement without encoder regulation
    void simpleForward() {
      motor1.run(FORWARD);
      motor2.run(FORWARD);
      motor3.run(FORWARD);
      motor4.run(FORWARD);
    }

    void simpleBackward() {
      motor1.run(BACKWARD);
      motor2.run(BACKWARD);
      motor3.run(BACKWARD);
      motor4.run(BACKWARD);
    }

    void simpleLeft() {
      motor1.run(BACKWARD);
      motor2.run(FORWARD);
      motor3.run(BACKWARD);
      motor4.run(FORWARD);
    }

    void simpleRight() {
      motor1.run(FORWARD);
      motor2.run(BACKWARD);
      motor3.run(FORWARD);
      motor4.run(BACKWARD);
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
      setMotors(BACKWARD, FORWARD, BACKWARD, FORWARD);
      return checkDone(pulses);
    }

    // Right (strafe)
    bool right(long pulses) {
      if (!moving) startMove();
      setMotors(FORWARD, BACKWARD, FORWARD, BACKWARD);
      return checkDone(pulses);
    }

    // Diagonal: Forward-Left
    bool forwardLeft(long pulses) {
      if (!moving) startMove();
      setMotors(FORWARD, RELEASE, FORWARD, RELEASE);
      return checkDone(pulses);
    }

    // Diagonal: Forward-Right
    bool forwardRight(long pulses) {
      if (!moving) startMove();
      setMotors(RELEASE, FORWARD, RELEASE, FORWARD);
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
    void rotateCW(int pwm, int pwm2, int pwm3, int pwm4) {
      setMotors(BACKWARD, FORWARD, FORWARD, BACKWARD);
      motor1.setSpeed(pwm);
      motor2.setSpeed(pwm2);
      motor3.setSpeed(pwm3);
      motor4.setSpeed(pwm4);      
      moving = false;
    }
    void rotateCCW(int pwm, int pwm2, int pwm3, int pwm4) {
      setMotors(FORWARD, BACKWARD, BACKWARD, FORWARD);
      motor1.setSpeed(pwm);
      motor2.setSpeed(pwm2);
      motor3.setSpeed(pwm3);
      motor4.setSpeed(pwm4);      
      moving = false;
    }

    // Stop all motors
    void stop() {
      setMotors(RELEASE, RELEASE, RELEASE, RELEASE);
      moving = false;
    }

    // Stop all motors for a given number of loop repetitions (non-blocking)
    // Call repeatedly in your main loop; returns true when done stopping
    bool stopFor(unsigned long repetitions) {
      static unsigned long stopCounter = 0;
      static bool stopping = false;

      if (!stopping) {
        setMotors(RELEASE, RELEASE, RELEASE, RELEASE);
        moving = false;
        stopCounter = 0;
        stopping = true;
      }

      if (stopCounter < repetitions) {
        stopCounter++;
        return false; // Still stopping
      } else {
        stopping = false;
        return true; // Done stopping
      }
    }

    bool stopForMillis(unsigned long durationMs) {
      static unsigned long startTime = 0;
      static bool stopping = false;

      if (!stopping) {
        setMotors(RELEASE, RELEASE, RELEASE, RELEASE);
        moving = false;
        startTime = millis();
        stopping = true;
      }

      if (millis() - startTime < durationMs) {
        return false; // Still stopping
      } else {
        stopping = false;
        return true; // Done stopping
      }
    }

    long mmToPulses(float mm, float wheelDiameterMM, int pulsesPerRevolution) {
      // Calculate wheel circumference in mm
      float circumference = 3.14159265f * wheelDiameterMM;
      // Pulses per mm
      float pulsesPerMM = pulsesPerRevolution / circumference;
      // Total pulses for the given mm
      return static_cast<long>(mm * pulsesPerMM + 0.5f); // rounded
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
      if (deltaLeft >= pulses || deltaRight >= pulses) {
        stop();
        return true;
      }
      return false;
    }
};