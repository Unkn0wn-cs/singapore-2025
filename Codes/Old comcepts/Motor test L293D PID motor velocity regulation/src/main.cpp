#include <Arduino.h>
#include <PID_v1.h>
#include <AFMotor.h>

// Motor shield setup
AF_DCMotor motor1(1); // Motor 1 connected to M1
AF_DCMotor motor2(2); // Motor 2 connected to M2

// Encoder pins
const int encoder1PinA = A8;
const int encoder1PinB = A9;
const int encoder2PinA = A10;
const int encoder2PinB = A11;

// PID parameters
double kp = 2.0, ki = 5.0, kd = 1.0;
double setpoint = 100.0; // Desired velocity

// Encoder counts
volatile long encoder1Count = 0;
volatile long encoder2Count = 0;

// PID variables
double input1, output1, input2, output2;

// PID controllers
PID motor1PID(&input1, &output1, &setpoint, kp, ki, kd, DIRECT);
PID motor2PID(&input2, &output2, &setpoint, kp, ki, kd, DIRECT);

// Function declarations
void encoder1ISR();
void encoder2ISR();
double calculateVelocity(long encoderCount);
void setMotorSpeed(AF_DCMotor &motor, double speed);

void encoder1ISR() {
  if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) {
    encoder1Count++;
  } else {
    encoder1Count--;
  }
}

void encoder2ISR() {
  if (digitalRead(encoder2PinA) == digitalRead(encoder2PinB)) {
    encoder2Count++;
  } else {
    encoder2Count--;
  }
}

double calculateVelocity(long encoderCount) {
  // Simple velocity calculation (pulses per time unit)
  return encoderCount;
}

void setMotorSpeed(AF_DCMotor &motor, double speed) {
  int pwm = constrain(abs(speed), 0, 255);
  motor.setSpeed(pwm);
  if (speed > 0) {
    motor.run(FORWARD);
  } else {
    motor.run(BACKWARD);
  }
}

void setup() {
  Serial.begin(9600);

  // Encoder pins setup
  pinMode(encoder1PinA, INPUT);
  pinMode(encoder1PinB, INPUT);
  pinMode(encoder2PinA, INPUT);
  pinMode(encoder2PinB, INPUT);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), encoder2ISR, CHANGE);

  // Initialize PID controllers
  motor1PID.SetMode(AUTOMATIC);
  motor2PID.SetMode(AUTOMATIC);

  // Turn motors on
  motor1.run(FORWARD);
  motor2.run(FORWARD);
}

void loop() {
  // Calculate velocities
  input1 = calculateVelocity(encoder1Count);
  input2 = calculateVelocity(encoder2Count);

  // Compute PID outputs
  motor1PID.Compute();
  motor2PID.Compute();

  // Set motor speeds
  setMotorSpeed(motor1, output1);
  setMotorSpeed(motor2, output2);

  // Reset encoder counts for next calculation
  encoder1Count = 0;
  encoder2Count = 0;

  // Small delay to simulate background task
  delay(100);  
}



