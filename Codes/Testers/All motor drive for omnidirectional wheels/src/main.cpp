#include <Arduino.h>
#include <AFMotor.h>
#include <QuadratureEncoder.h>

// Define motors & encoders
AF_DCMotor motor1(1); // Motor 1 on the Adafruit Motor Shield
AF_DCMotor motor2(2); // Motor 2 on the Adafruit Motor Shield
AF_DCMotor motor3(3); // Motor 3 on the Adafruit Motor Shield
AF_DCMotor motor4(4); // Motor 4 on the Adafruit Motor Shield
Encoders leftEncoder1(46, 47); // Encoder for motor1
Encoders leftEncoder2(36, 37); // Encoder for motor2
Encoders rightEncoder1(46, 47); // Encoder for motor3
Encoders rightEncoder2(36, 37); // Encoder for motor4

const int pwmValueA = 225; // Maximum PWM value (full speed)
const int pwmValueB = 225; // Maximum PWM value (full speed)
const unsigned long interval = 1000; // Interval for turning motors on and off (in milliseconds)
unsigned long lastMilli = 0;

unsigned long lastTime = 0;
int step = 1;

const long TARGET_PULSES = 600; // Set your desired pulse count per movement

// Track encoder counts at the start of each step
long startLeftEncoder1 = 0;
long startLeftEncoder2 = 0;
long startRightEncoder1 = 0;
long startRightEncoder2 = 0;

class Move {
  public:
    void forward(int in = 1) {
      // while (in <= leftEncoder1){
      motor1.run(FORWARD);
      motor2.run(FORWARD);
      motor3.run(FORWARD);
      motor4.run(FORWARD);
      // }
    }
    void backward() {
      motor1.run(BACKWARD);
      motor2.run(BACKWARD);
      motor3.run(BACKWARD);
      motor4.run(BACKWARD);
    }

    void left() {
      motor1.run(BACKWARD);
      motor2.run(FORWARD);
      motor3.run(BACKWARD);
      motor4.run(FORWARD);
    }

    void right() {
      motor1.run(FORWARD);
      motor2.run(BACKWARD);
      motor3.run(FORWARD);
      motor4.run(BACKWARD);
    }

    void stop() {
      motor1.run(RELEASE);
      motor2.run(RELEASE);
      motor3.run(RELEASE);
      motor4.run(RELEASE);
    }

    void rotate(bool clockwise) {
      if (clockwise) {
        motor1.run(BACKWARD);
        motor2.run(FORWARD);
        motor3.run(FORWARD);
        motor4.run(BACKWARD);
      } else {
        motor1.run(FORWARD);
        motor2.run(BACKWARD);
        motor3.run(BACKWARD);
        motor4.run(FORWARD);
      }
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
      move.forward();
      Serial.println("Motors on");
    } else if (step == 2) {
      move.stop();
      Serial.println("Motors off");
    } else if (step == 3) {
      move.backward();
      Serial.println("Motors back");
    } else if (step == 4) {
      move.stop();
      Serial.println("Motors off");
    } else if (step == 5) {
      move.left();
      Serial.println("Motors left");
    } else if (step == 6) {
      move.stop();
      Serial.println("Motors off");
    } else if (step == 7) {
      move.right();
      Serial.println("Motors right");
    } else if (step == 8) {
      move.stop();
      Serial.println("Motors off");
    } else if (step == 9) {
      move.rotate(true); // Rotate clockwise
      Serial.println("Motors rotate clockwise");
    } else if (step == 10) {
      move.stop();
      Serial.println("Motors off");
    } else if (step == 11) {
      move.rotate(false); // Rotate counterclockwise
      Serial.println("Motors rotate counterclockwise");
    } else if (step == 12) {
      move.stop();
      Serial.println("Motors off");
    }

    lastTime = currentTime;
    if (step < 12) {
      step++;
    } else {
      step = 1;
    }
  }

  if (millis() - lastMilli > 500) {
    long currentLeftEncoderCount1 = leftEncoder1.getEncoderCount();
    long currentLeftEncoderCount2 = leftEncoder2.getEncoderCount();
    long currentRightEncoderCount1 = rightEncoder1.getEncoderCount();
    long currentRightEncoderCount2 = rightEncoder2.getEncoderCount();

    Serial.print((currentLeftEncoderCount1) / 600.0);   
    Serial.print(" , ");
    Serial.print((currentLeftEncoderCount2) / 600.0);
    Serial.print(" , ");
    Serial.print((currentRightEncoderCount1) / 600.0);
    Serial.print(" , ");
    Serial.println((currentRightEncoderCount2) / 600.0);

    lastMilli = millis();
  }
}

