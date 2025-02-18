#include <QuadratureEncoder.h>
#include <AFMotor.h>

#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #if defined(__AVR__)
    #include <avr/io.h>
  #endif
  #include "WProgram.h"
#endif


Encoders leftEncoder(A8 ,A9);	// Create an Encoder object name leftEncoder, using digitalpin 2 & 3
AF_DCMotor motor2(2); // create motor #2, 64KHz pwm

const int targetRPM = 60; // Target speed in RPM
const int pulsesPerRevolution = 600; // Number of pulses per revolution
const unsigned long interval = 1000; // Interval for speed calculation in milliseconds

unsigned long lastTime = 0;
long lastEncoderCount = 0;

void setup() {
  Serial.begin(9600);
  motor2.setSpeed(0); // Start with motor off
  motor2.run(FORWARD); // Set motor direction
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= interval) {
    long currentEncoderCount = leftEncoder.getEncoderCount();
    long pulses = currentEncoderCount - lastEncoderCount;
    lastEncoderCount = currentEncoderCount;

    // Calculate RPM
    float rpm = (pulses / (float)pulsesPerRevolution) * (60000.0 / interval);

    // Print RPM to Serial Monitor
    Serial.print("Current RPM: ");
    Serial.println(rpm);

    // Adjust motor speed based on RPM
    int pwmValue = map(rpm, 0, targetRPM, 0, 255);
    pwmValue = constrain(pwmValue, 0, 255);
    motor2.setSpeed(pwmValue);

    // Print PWM value to Serial Monitor
    Serial.print("PWM Value: ");
    Serial.println(pwmValue);

    lastTime = currentTime;
  }
}