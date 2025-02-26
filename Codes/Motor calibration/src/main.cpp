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

//INSERT VALUES HERE
int encoderA1 = A10;
int encoderB1 = A11;
int encoderA2 = A8;
int encoderB2 = A9;
const int targetRPM = 300; // Target speed in RPM
int motorN1 = 1;
int motorN2 = 2;

// Create Encoder objects
Encoders leftEncoder(encoderA1, encoderB1);
Encoders rightEncoder(encoderA2, encoderB2);

// Create motor objects
AF_DCMotor motor1(motorN1); // create motor #1, 64KHz pwm
AF_DCMotor motor2(motorN2); // create motor #2, 64KHz pwm

const int pulsesPerRevolution = 600; // Number of pulses per revolution
const unsigned long interval = 1000; // Interval for speed calculation in milliseconds

unsigned long lastTime = 0;
long lastEncoderCount1 = 0;
long lastEncoderCount2 = 0;
float pwmValue1 = 80.0; // Initial PWM value for motor 1 (50% duty cycle)
float pwmValue2 = 80.0; // Initial PWM value for motor 2 (50% duty cycle)
float change = 20.0; // Change in PWM value

void setup() {
  Serial.begin(9600);
  motor1.setSpeed(pwmValue1); // Start with initial PWM value
  motor1.run(FORWARD); // Set motor direction
  motor2.setSpeed(pwmValue2); // Start with initial PWM value
  motor2.run(FORWARD); // Set motor direction
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= interval) {
    long currentEncoderCount1 = leftEncoder.getEncoderCount();
    long currentEncoderCount2 = rightEncoder.getEncoderCount();
    long pulses1 = currentEncoderCount1 - lastEncoderCount1;
    long pulses2 = currentEncoderCount2 - lastEncoderCount2;
    lastEncoderCount1 = currentEncoderCount1;
    lastEncoderCount2 = currentEncoderCount2;

    // Calculate RPM for motor 1
    float rpm1 = (pulses1 / (float)pulsesPerRevolution) * (60000.0 / interval);
    // Calculate RPM for motor 2
    float rpm2 = (pulses2 / (float)pulsesPerRevolution) * (60000.0 / interval);

    // Print RPM to Serial Monitor
    Serial.print("Current RPM Motor 1: ");
    Serial.println(rpm1);
    Serial.print("Current RPM Motor 2: ");
    Serial.println(rpm2);

    // Adjust motor speed based on RPM for motor 1
    if (rpm1 < targetRPM) {
      pwmValue1 += change; // Increase PWM value
    } else if (rpm1 > targetRPM) {
      pwmValue1 -= change; // Decrease PWM value
    }
    pwmValue1 = constrain(pwmValue1, 0.0, 255.0);
    motor1.setSpeed(pwmValue1);

    // Adjust motor speed based on RPM for motor 2
    if (rpm2 < targetRPM) {
      pwmValue2 += change; // Increase PWM value
    } else if (rpm2 > targetRPM) {
      pwmValue2 -= change; // Decrease PWM value
    }
    pwmValue2 = constrain(pwmValue2, 0.0, 255.0);
    motor2.setSpeed(pwmValue2);

    // Print PWM values to Serial Monitor
    Serial.print("PWM Value Motor 1: ");
    Serial.println((pwmValue1) * 1.0);
    Serial.print("PWM Value Motor 2: ");
    Serial.println((pwmValue2) * 1.0);

    lastTime = currentTime;
    if (change >= 2){
      change = change - 1;
    } else if (change <= 1 && change > 0.1)
    {
      change = change - 0.1;
    }
  }
}