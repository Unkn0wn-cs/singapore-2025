// Decelerate to a lower PWM
bool decelerateToPWM(int initialPWM, int targetPWM, int steps = 20, unsigned long stepDelay = 20) {
  static int currentStep = 0;
  static unsigned long lastStepTime = 0;
  static int pwm = initialPWM;
  if (currentStep == 0) {
    pwm = initialPWM;
    lastStepTime = millis();
    setAllSpeeds(pwm);
  }
  if (pwm <= targetPWM) {
    setAllSpeeds(targetPWM);
    currentStep = 0;
    return true; // Done decelerating
  }
  if (millis() - lastStepTime >= stepDelay) {
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
  return false; // Still decelerating
}

// Accelerate to a PWM
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
  long currentPulses = getEncoderPulses(); // Replace with your encoder read function
  long decelStart = pulses * decelPercent;

  if (!moving) {
    startMove();
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
    accelerateToPWM(targetPWM, finalPWM, steps, stepDelay, true); // decelerate (reverse)
  }

  return checkDone(pulses);
}

// Accelerate and then decelerate backward
bool accelerateBackward(long pulses, int initialPWM, int targetPWM, int steps = 20, unsigned long stepDelay = 20, int finalPWM = 0, float decelPercent = 0.7) {
  static bool decelerating = false;
  long currentPulses = getEncoderPulses(); // Replace with your encoder read function
  long decelStart = pulses * decelPercent;

  if (!moving) {
    startMove();
    setMotors(BACKWARD, BACKWARD, BACKWARD, BACKWARD);
    setAllSpeeds(initialPWM);
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
    accelerateToPWM(targetPWM, finalPWM, steps, stepDelay, true); // decelerate (reverse)
  }

  return checkDone(pulses);
}