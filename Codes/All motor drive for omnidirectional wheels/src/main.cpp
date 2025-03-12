#include <Arduino.h>
#include <AFMotor.h>
#include <QuadratureEncoder.h>

// Define motors & encoders
AF_DCMotor motor1(1); // Motor 1 on the Adafruit Motor Shield
AF_DCMotor motor2(2); // Motor 2 on the Adafruit Motor Shield
AF_DCMotor motor3(3); // Motor 3 on the Adafruit Motor Shield
AF_DCMotor motor4(4  ); // Motor 4 on the Adafruit Motor Shield
Encoders leftEncoder(A8, A9); // Create an Encoder object name leftEncoder, using digitalpin 2 & 3
Encoders rightEncoder(A10, A11); // Encoder object name rightEncoder using analog pin A0 and A1

const int pwmValueA = 225; // Maximum PWM value (full speed)
const int pwmValueB = 225; // Maximum PWM value (full speed)
const unsigned long interval = 1000; // Interval for turning motors on and off (in milliseconds)
unsigned long lastMilli = 0;

unsigned long lastTime = 0;
int step = 1;

class Move {
  public:
    void forward() {
      motor1.run(FORWARD);
      motor2.run(FORWARD);
      motor3.run(FORWARD);
      motor4.run(FORWARD);
    }

    void backward() {
      motor1.run(BACKWARD);
      motor2.run(BACKWARD);
      motor3.run(BACKWARD);
      motor4.run(BACKWARD);
    }

    void left() {
      motor1.run(FORWARD);
      motor2.run(FORWARD);
      motor3.run(BACKWARD);
      motor4.run(BACKWARD);
    }

    void right() {
      motor1.run(BACKWARD);
      motor2.run(BACKWARD);
      motor3.run(FORWARD);
      motor4.run(FORWARD);
    }

    void stop() {
      motor1.run(RELEASE);
      motor2.run(RELEASE);
      motor3.run(RELEASE);
      motor4.run(RELEASE);
    }
};

Move move;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  motor1.setSpeed(pwmValueA);
  motor2.setSpeed(pwmValueB);
  motor3.setSpeed(pwmValueA);
  motor4.setSpeed(pwmValueB);
}



void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= interval) {
    if (step == 1) {
      // Turn motors off
      move.forward();
      Serial.println("Motors on");
    } else if (step == 2) {
      // Turn motors on
      move.stop();
      Serial.println("Motors off");
    } else if (step == 3){
      move.backward();
      Serial.println("Motors back");
    } else if (step == 4) {
      move.stop();
      Serial.println("Motors off");
    } else if (step == 5) {
      move.left();
      Serial.println("Motors left");
    }else if (step == 6) {
      move.stop();
      Serial.println("Motors off");
    } else if (step == 7) {
      move.right();
      Serial.println("Motors right");
    } else if (step == 8) {
      move.stop();
      Serial.println("Motors off");}

    lastTime = currentTime;
    if (step < 8) {
      step ++;
    } 
  }

  if (millis() - lastMilli > 50) {
    long currentLeftEncoderCount = leftEncoder.getEncoderCount();
    long currentRightEncoderCount = rightEncoder.getEncoderCount();

    Serial.print((currentLeftEncoderCount) / 600.0);
    Serial.print(" , ");
    Serial.println((currentRightEncoderCount) / 600.0);

    lastMilli = millis();
  }
}

