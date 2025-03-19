#include <Arduino.h>
#include <QuadratureEncoder.h>

// Define motor pins
const int motor1Pin1 = 2; // Motor 1 direction pin 1 (positive)
const int motor1Pin2 = 3; // Motor 1 direction pin 2 (negative)
const int motor1PWM = 255; // Motor 1 PWM pin
const int motor2Pin1 = 4; // Motor 2 direction pin 1 (positive)
const int motor2Pin2 = 5; // Motor 2 direction pin 2 (negative)
const int motor2PWM = 255; // Motor 2 PWM pin
const int motor3Pin1 = 6; // Motor 3 direction pin 1 (positive)
const int motor3Pin2 = 7; // Motor 3 direction pin 2 (negative)
const int motor3PWM = 255; // Motor 3 PWM pin
const int motor4Pin1 = 8; // Motor 4 direction pin 1 (positive)
const int motor4Pin2 = 9; // Motor 4 direction pin 2 (negative)
const int motor4PWM = 255; // Motor 4 PWM pin

// Define encoders
Encoders leftEncoder1(A8, A9); // Encoder for motor1 (A8: A, A9: B)
Encoders leftEncoder2(A10, A11); // Encoder for motor2 (A10: A, A11: B)
Encoders rightEncoder1(A12, A13); // Encoder for motor3 (A12: A, A13: B)
Encoders rightEncoder2(A14, A15); // Encoder for motor4 (A14: A, A15: B)

const int pwmValueA = 225; // Maximum PWM value (full speed)
const int pwmValueB = 225; // Maximum PWM value (full speed)
const unsigned long interval = 1000; // Interval for turning motors on and off (in milliseconds)
unsigned long lastMilli = 0;
unsigned long lastTime = 0;
int step = 1;

class Move {
  public:
    void forward() {
      setMotor(motor1Pin1, motor1Pin2, motor1PWM, pwmValueA, true);
      setMotor(motor2Pin1, motor2Pin2, motor2PWM, pwmValueB, true);
      setMotor(motor3Pin1, motor3Pin2, motor3PWM, pwmValueA, true);
      setMotor(motor4Pin1, motor4Pin2, motor4PWM, pwmValueB, true);
    }

    void backward() {
      setMotor(motor1Pin1, motor1Pin2, motor1PWM, pwmValueA, false);
      setMotor(motor2Pin1, motor2Pin2, motor2PWM, pwmValueB, false);
      setMotor(motor3Pin1, motor3Pin2, motor3PWM, pwmValueA, false);
      setMotor(motor4Pin1, motor4Pin2, motor4PWM, pwmValueB, false);
    }

    void left() {
      setMotor(motor1Pin1, motor1Pin2, motor1PWM, pwmValueA, true);
      setMotor(motor2Pin1, motor2Pin2, motor2PWM, pwmValueB, true);
      setMotor(motor3Pin1, motor3Pin2, motor3PWM, pwmValueA, false);
      setMotor(motor4Pin1, motor4Pin2, motor4PWM, pwmValueB, false);
    }

    void right() {
      setMotor(motor1Pin1, motor1Pin2, motor1PWM, pwmValueA, false);
      setMotor(motor2Pin1, motor2Pin2, motor2PWM, pwmValueB, false);
      setMotor(motor3Pin1, motor3Pin2, motor3PWM, pwmValueA, true);
      setMotor(motor4Pin1, motor4Pin2, motor4PWM, pwmValueB, true);
    }

    void stop() {
      analogWrite(motor1PWM, 0);
      analogWrite(motor2PWM, 0);
      analogWrite(motor3PWM, 0);
      analogWrite(motor4PWM, 0);
    }

    void rotate(bool clockwise) {
      if (clockwise) {
        setMotor(motor1Pin1, motor1Pin2, motor1PWM, pwmValueA, true);
        setMotor(motor2Pin1, motor2Pin2, motor2PWM, pwmValueB, false);
        setMotor(motor3Pin1, motor3Pin2, motor3PWM, pwmValueA, true);
        setMotor(motor4Pin1, motor4Pin2, motor4PWM, pwmValueB, false);
      } else {
        setMotor(motor1Pin1, motor1Pin2, motor1PWM, pwmValueA, false);
        setMotor(motor2Pin1, motor2Pin2, motor2PWM, pwmValueB, true);
        setMotor(motor3Pin1, motor3Pin2, motor3PWM, pwmValueA, false);
        setMotor(motor4Pin1, motor4Pin2, motor4PWM, pwmValueB, true);
      }
    }

  private:
    void setMotor(int pin1, int pin2, int pwmPin, int speed, bool forward) {
      digitalWrite(pin1, forward ? HIGH : LOW);
      digitalWrite(pin2, forward ? LOW : HIGH);
      analogWrite(pwmPin, speed);
    }
};

Move move;

void setup() {
  Serial.begin(9600);

  // Initialize motor pins
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
  pinMode(motor3Pin1, OUTPUT);
  pinMode(motor3Pin2, OUTPUT);
  pinMode(motor3PWM, OUTPUT);
  pinMode(motor4Pin1, OUTPUT);
  pinMode(motor4Pin2, OUTPUT);
  pinMode(motor4PWM, OUTPUT);
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
    }
  }

  if (millis() - lastMilli > 50) {
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

