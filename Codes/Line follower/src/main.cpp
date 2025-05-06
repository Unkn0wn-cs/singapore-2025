#include <Arduino.h>
#include <Wire.h> // Include the Wire library for I2C communication
#include <C:\Users\samue\OneDrive\Documents\GitHub\singapore25\Codes\lib\Adafruit Motor Shield library\AFMotor.h>
#include <C:\Users\samue\OneDrive\Documents\GitHub\singapore25\Codes\lib\QuadratureEncoder\QuadratureEncoder.h>

// Define motors
AF_DCMotor motor1(1); // Motor 1 on the Adafruit Motor Shield (left motor)
AF_DCMotor motor2(2); // Motor 2 on the Adafruit Motor Shield (right motor)

// I2C address for the line-following sensor
const int sensorAddress = 0x3C; // Replace with your sensor's I2C address

// Function to read sensor values via I2C
void readSensorValues(int sensorValues[4]) {
  Wire.beginTransmission(sensorAddress); // Start communication with the sensor
  Wire.write(0x00); // Command to request sensor data (adjust as needed)
  Wire.endTransmission();

  Wire.requestFrom(sensorAddress, 4); // Request 4 bytes of data
  for (int i = 0; i < 4; i++) {
    if (Wire.available()) {
      sensorValues[i] = Wire.read();
    }
  }
}

// Function to calculate error based on sensor readings
float calculateError(int sensorValues[4]) {
  int weights[4] = {-3, -1, 1, 3}; // Weights for each sensor
  int sum = 0, totalWeight = 0;

  for (int i = 0; i < 4; i++) {
    sum += sensorValues[i] * weights[i];
    totalWeight += sensorValues[i];
  }

  if (totalWeight == 0) {
    return 0; // No line detected
  }

  return (float)sum / totalWeight;
}

// Function to set motor speeds
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  motor1.setSpeed(leftSpeed);
  motor2.setSpeed(rightSpeed);

  // Always run motors forward
  motor1.run(FORWARD);
  motor2.run(FORWARD);
}

// PID constants
float Kp = 2.0;  // Proportional gain
float Ki = 0.0;  // Integral gain
float Kd = 1.0;  // Derivative gain

// PID variables
float error = 0, previousError = 0, integral = 0, derivative = 0;
float correction = 0;

// Motor speed
int baseSpeed = 150; // Base speed for motors
int leftMotorSpeed = 0, rightMotorSpeed = 0;

void setup() {
  // Initialize I2C communication
  Wire.begin();

  // Set initial motor speeds to 0
  motor1.setSpeed(0);
  motor2.setSpeed(0);

  // Set motor directions to forward
  motor1.run(FORWARD);
  motor2.run(FORWARD);

  // Start serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // Read sensor values via I2C
  int sensorValues[4];
  readSensorValues(sensorValues);

  // Calculate error based on sensor readings
  error = calculateError(sensorValues);

  // Calculate PID terms
  integral += error;
  derivative = error - previousError;
  correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // Adjust motor speeds based on correction
  leftMotorSpeed = baseSpeed - correction;
  rightMotorSpeed = baseSpeed + correction;

  // Constrain motor speeds to valid range
  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

  // Set motor speeds
  setMotorSpeed(leftMotorSpeed, rightMotorSpeed);

  // Update previous error
  previousError = error;

  // Small delay for stability
  delay(10);
}

