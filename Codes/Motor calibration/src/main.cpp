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
int encoderA = A10;
int encoderB = A11;
const int targetRPM = 300; // Target speed in RPM
int motorN = 1;



// Create an Encoder object named leftEncoder, using digital pins A8 & A9
Encoders leftEncoder(encoderA, encoderB);
AF_DCMotor motor2(motorN); // create motor #2, 64KHz pwm

const int pulsesPerRevolution = 600; // Number of pulses per revolution
const unsigned long interval = 1000; // Interval for speed calculation in milliseconds

unsigned long lastTime = 0;
long lastEncoderCount = 0;
float pwmValue = 80.0; // Initial PWM value (50% duty cycle)
float change = 20.0; // Change in PWM value


void setup() {
  Serial.begin(9600);
  motor2.setSpeed(pwmValue); // Start with initial PWM value
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
    if (rpm < targetRPM) {
      pwmValue += change; // Increase PWM value
    } else if (rpm > targetRPM) {
      pwmValue -= change; // Decrease PWM value
    }
    pwmValue = constrain(pwmValue, 0.0, 255.0);
    motor2.setSpeed(pwmValue);

    // Print PWM value to Serial Monitor
    Serial.print("PWM Value: ");
    Serial.println((pwmValue) * 1.0);

    lastTime = currentTime;
    if (change >= 2){
      change = change - 1;
    } else if (change <= 1 && change > 0.1)
    {
      change = change - 0.1;
    }
    
    
  }
}