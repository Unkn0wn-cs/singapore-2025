#pragma once
#include <AFMotor.h>
#include <QuadratureEncoder.h>


class Move {
  public:
// Timeout for all movement functions (in milliseconds)
    unsigned long moveTimeoutMs = 4000; // Default 3 seconds, can be changed by user

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

    long getStartLeft()  { return startLeft; }
    long getStartRight()  { return startRight; }

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
      if (!moving) {
        startMove();
        moveStartTime = millis();
      }
      setMotors(FORWARD, FORWARD, FORWARD, FORWARD);
      return checkDoneWithTimeout(pulses);
    }

    // Backward
    bool backward(long pulses) {
      if (!moving) {
        startMove();
        moveStartTime = millis();
      }
      setMotors(BACKWARD, BACKWARD, BACKWARD, BACKWARD);
      return checkDoneWithTimeout(pulses);
    }

    // Left (strafe)
    bool left(long pulses) {
      if (!moving) {
        startMove();
        moveStartTime = millis();
      }
      setMotors(BACKWARD, FORWARD, BACKWARD, FORWARD);
      return checkDoneWithTimeout(pulses);
    }

    // Right (strafe)
    bool right(long pulses) {
      if (!moving) {
        startMove();
        moveStartTime = millis();
      }
      setMotors(FORWARD, BACKWARD, FORWARD, BACKWARD);
      return checkDoneWithTimeout(pulses);
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

    // Calculates the distance in mm for a given number of pulses
    float pulsesToMM(long pulses, float wheelDiameterMM, int pulsesPerRevolution) {
        // Calculate wheel circumference in mm
        float circumference = 3.14159265f * wheelDiameterMM;
        // mm per pulse
        float mmPerPulse = circumference / pulsesPerRevolution; 
        // Total mm for the given pulses
        return pulses * mmPerPulse;
    }

    // Accelerate all motors from initialPWM to targetPWM in 'steps' increments
    // Call this repeatedly in your main loop
    
    // ...existing code...

bool accelerateToPWM(int initialPWM, int targetPWM, int steps = 20, unsigned long stepDelay = 20, bool reverse = false) {
  static int currentStep = 0;
  static unsigned long lastStepTime = 0;
  static int pwm = initialPWM;
  if (currentStep == 0) {
    pwm = initialPWM;
    lastStepTime = millis();
    setAllSpeeds(pwm);
  }
  if ((!reverse && pwm >= targetPWM) || (reverse && pwm <= targetPWM)) {
    setAllSpeeds(targetPWM);
    currentStep = 0;
    return true;
  }
  if (millis() - lastStepTime >= stepDelay) {
    if (!reverse)
      pwm = initialPWM + ((targetPWM - initialPWM) * currentStep) / steps;
    else
      pwm = initialPWM - ((initialPWM - targetPWM) * currentStep) / steps;
    setAllSpeeds(pwm);
    currentStep++;
    lastStepTime = millis();
  }
  if (currentStep > steps) {
    setAllSpeeds(targetPWM);
    currentStep = 0;
    return true;
  }
  return false;
}

// Accelerate and then decelerate forward
 bool accelerateForward(long pulses, int initialPWM, int targetPWM, int steps = 20, unsigned long stepDelay = 20, int finalPWM = 0, float decelPercent = 0.7, int variation1 = 0, int variation2 = 0, int variation3 = 0, int variation4 = 0) {
      static bool decelerating = false;
      long currentPulses = encoderRight.getEncoderCount();
      long decelStart = pulses * decelPercent;

      if (!moving) {
        startMove();
        moveStartTime = millis();
        setMotors(FORWARD, FORWARD, FORWARD, FORWARD);
        setIndividualSpeeds(initialPWM + variation1, initialPWM + variation2, initialPWM + variation3, initialPWM + variation4);
        delay(20);
        decelerating = false;
      }
      setMotors(FORWARD, FORWARD, FORWARD, FORWARD);

      if (!decelerating && currentPulses >= decelStart) {
        decelerating = true;
      }

      if (!decelerating) {
        accelerateToPWM(initialPWM, targetPWM, steps, stepDelay, false);
      } else {
        accelerateToPWM(targetPWM, finalPWM, steps, stepDelay, true);
      }

      return checkDoneWithTimeout(pulses);
    }


// Accelerate and then decelerate backward with variations
    bool accelerateBackward(long pulses, int initialPWM, int targetPWM, int steps = 20, unsigned long stepDelay = 20, int finalPWM = 0, float decelPercent = 0.7, int variation1 = 0, int variation2 = 0, int variation3 = 0, int variation4 = 0) {
      static bool decelerating = false;
      long currentPulses = encoderRight.getEncoderCount();
      long decelStart = pulses * decelPercent;

      if (!moving) {
        startMove();
        moveStartTime = millis();
        setMotors(BACKWARD, BACKWARD, BACKWARD, BACKWARD);
        setIndividualSpeeds(initialPWM + variation1, initialPWM + variation2, initialPWM + variation3, initialPWM + variation4);
        delay(20);
        decelerating = false;
      }
      setMotors(BACKWARD, BACKWARD, BACKWARD, BACKWARD);

      if (!decelerating && currentPulses >= decelStart) {
        decelerating = true;
      }

      if (!decelerating) {
        accelerateToPWM(initialPWM, targetPWM, steps, stepDelay, false);
      } else {
        accelerateToPWM(targetPWM, finalPWM, steps, stepDelay, true);
      }

      return checkDoneWithTimeout(pulses);
    }
// Accelerate and then decelerate left with variations
    bool accelerateLeft(long pulses, int initialPWM, int targetPWM, int steps = 20, unsigned long stepDelay = 20, int finalPWM = 0, float decelPercent = 0.7, int variation1 = 0, int variation2 = 0, int variation3 = 0, int variation4 = 0) {
      static bool decelerating = false;
      long currentPulses = encoderLeft.getEncoderCount();
      long decelStart = pulses * decelPercent;

      if (!moving) {
        startMove();
        moveStartTime = millis();
        setMotors(BACKWARD, FORWARD, BACKWARD, FORWARD);
        setIndividualSpeeds(initialPWM + variation1, initialPWM + variation2, initialPWM + variation3, initialPWM + variation4);
        delay(20);
        decelerating = false;
      }
      setMotors(BACKWARD, FORWARD, BACKWARD, FORWARD);

      if (!decelerating && currentPulses >= decelStart) {
        decelerating = true;
      }

      if (!decelerating) {
        accelerateToPWM(initialPWM, targetPWM, steps, stepDelay, false);
      } else {
        accelerateToPWM(targetPWM, finalPWM, steps, stepDelay, true);
      }

      return checkDoneWithTimeout(pulses);
    }

// Accelerate and then decelerate right with variations
    bool accelerateRight(long pulses, int initialPWM, int targetPWM, int steps = 20, unsigned long stepDelay = 20, int finalPWM = 0, float decelPercent = 0.7, int variation1 = 0, int variation2 = 0, int variation3 = 0, int variation4 = 0) {
      static bool decelerating = false;
      long currentPulses = encoderRight.getEncoderCount();
      long decelStart = pulses * decelPercent;

      if (!moving) {
        startMove();
        moveStartTime = millis();
        setMotors(FORWARD, BACKWARD, FORWARD, BACKWARD);
        setIndividualSpeeds(initialPWM + variation1, initialPWM + variation2, initialPWM + variation3, initialPWM + variation4);
        delay(20);
        decelerating = false;
      }
      setMotors(FORWARD, BACKWARD, FORWARD, BACKWARD);

      if (!decelerating && currentPulses >= decelStart) {
        decelerating = true;
      }

      if (!decelerating) {
        accelerateToPWM(initialPWM, targetPWM, steps, stepDelay, false);
      } else {
        accelerateToPWM(targetPWM, finalPWM, steps, stepDelay, true);
      }

      return checkDoneWithTimeout(pulses);
    }

  // New: checkDone with timeout
    bool checkDoneWithTimeout(long pulses) {
      long deltaLeft = abs(encoderLeft.getEncoderCount() - startLeft);
      long deltaRight = abs(encoderRight.getEncoderCount() - startRight);
      if (deltaLeft >= pulses || deltaRight >= pulses) {
        stop();
        return true;
      }
      if (millis() - moveStartTime > moveTimeoutMs) {
        stop();
        return true;
      }
      return false;
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
    unsigned long moveStartTime = 0; // For movement timeout


    void setMotors(uint8_t m1, uint8_t m2, uint8_t m3, uint8_t m4) {
      motor1.run(m1);
      motor2.run(m2);
      motor3.run(m3);
      motor4.run(m4);
    }


    // Helper to set all motor speeds
    void setAllSpeeds(int pwm) {
      motor1.setSpeed(pwm);
      motor2.setSpeed(pwm);
      motor3.setSpeed(pwm);
      motor4.setSpeed(pwm);
    }

    void setIndividualSpeeds(int pwm1, int pwm2, int pwm3, int pwm4) {
      motor1.setSpeed(pwm1); 
      motor2.setSpeed(pwm2);
      motor3.setSpeed(pwm3);
      motor4.setSpeed(pwm4);
    }
};