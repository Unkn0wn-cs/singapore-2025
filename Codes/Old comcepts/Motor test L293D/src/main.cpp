#include <Arduino.h>
#include <AFMotor.h>
#include <QuadratureEncoder.h>
// #include <EnableInterrupt.h>

// Define motors & encoders
AF_DCMotor motor1(1); // Motor 1 on the Adafruit Motor Shield
AF_DCMotor motor2(2); // Motor 2 on the Adafruit Motor Shield
Encoders leftEncoder(A8 ,A9);	// Create an Encoder object name leftEncoder, using digitalpin 2 & 3
Encoders rightEncoder(A10,A11); // Encoder object name rightEncoder using analog pin A0 and A1 

const int pwmValueA = 225; // Maximum PWM value (full speed)
const int pwmValueB = 211; // Maximum PWM value (full speed)
const unsigned long interval = 1000; // Interval for turning motors on and off (in milliseconds)
unsigned long lastMilli = 0;

unsigned long lastTime = 0;
bool motorsOn = false;


void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= interval) {
    // if (motorsOn) {
    //   // Turn motors off
    //   motor1.setSpeed(0);
    //   motor1.run(RELEASE);
    //   motor2.setSpeed(0);
    //   motor2.run(RELEASE);
    //   motorsOn = false;
    //   Serial.println("Motors off");
    // } else {
      // Turn motors on
      motor1.setSpeed(224);
      motor1.run(FORWARD);
      motor2.setSpeed(246.5);
      motor2.run(FORWARD);
      motorsOn = true;
      Serial.println("Motors on");
    // }
    lastTime = currentTime;

    if(millis()-lastMilli > 50){ 
    
      long currentLeftEncoderCount = leftEncoder.getEncoderCount();
      long currentRightEncoderCount = rightEncoder.getEncoderCount();
      
      Serial.print((currentLeftEncoderCount)/600.0);
      Serial.print(" , ");
      Serial.println((currentRightEncoderCount)/600.0);
      
      lastMilli = millis();
    }
  }
}
