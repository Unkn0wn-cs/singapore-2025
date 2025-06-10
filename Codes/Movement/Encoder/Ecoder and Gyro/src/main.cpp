#include <Arduino.h>
#include <AFMotor.h>
#include <QuadratureEncoder.h>
#include <Wire.h>
#include "C:\Users\samue\OneDrive\Documents\GitHub\singapore25\Codes\Movement\Encoder\Ecoder measured movement\lib\move\move.h"
#include "cali.h"
#include "GyroTurn.h"
#include "C:\Users\samue\OneDrive\Documents\GitHub\singapore25\Codes\libraries\MPU6050\MPU6050.h"
#include "C:\Users\samue\OneDrive\Documents\GitHub\singapore25\Codes\libraries\MPU6050\MPU6050.cpp"

// Define motors & encoders
AF_DCMotor motor1(1); // Motor 1 on the Adafruit Motor Shield
AF_DCMotor motor2(2); // Motor 2 on the Adafruit Motor Shield
AF_DCMotor motor3(3); // Motor 3 on the Adafruit Motor Shield
AF_DCMotor motor4(4); // Motor 4 on the Adafruit Motor Shield
Encoders encoderLeft(A15, A13);  // Motor 3
Encoders encoderRight(A14, A12); // Motor 4

Move move(motor1, motor2, motor3, motor4, encoderLeft, encoderRight);

MPU6050 sensor;
// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
int ax, ay, az;
int gx, gy, gz;

long tiempo_prev;
float dt;
float ang_x, ang_y, ang_z; // Added ang_z for Z-axis rotation
float ang_x_prev, ang_y_prev, ang_z_prev; // Added ang_z_prev for Z-axis rotation
// GyroTurn turner(sensor, move);

const int pwm = 200; // Maximum PWM value (full speed)
const unsigned long interval = 1000; // Interval for turning motors on and off (in milliseconds)
unsigned long lastMilli = 0;
  int diameter = 60; // Diameter of the wheel in mm
  const long ppr = 1200; // Number of pulses for each movement step

void setup() { 
    Wire.begin();           // THIS IS FOR THE MPU NOT THE SERIAL MONOTOR
  Serial.begin(9600);
  move.begin((pwm), pwm, pwm, pwm); // Initialize motors with PWM values
  sensor.initialize();
  // Gyro::calibrateMPU6050(sensor, 100); // Automatic calibration
  // turner.resetHeading(0); // Set current heading as 0
  // turner.setTarget(0);  
}
 
void loop() {
  static int state = 0;
  // const long PULSES = 400;
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();

  // Calcular los ángulos con acelerometro
  float accel_ang_x = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / 3.14);
  float accel_ang_y = atan(-ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / 3.14);

  // Calcular angulo de rotación con giroscopio y filtro complemento
  ang_x = 0.98 * (ang_x_prev + (gx / 131.0) * dt) + 0.02 * accel_ang_x;
  ang_y = 0.98 * (ang_y_prev + (gy / 131.0) * dt) + 0.02 * accel_ang_y;
  ang_z = ang_z_prev + (gz / 131.0) * dt; // Z-axis rotation using gyroscope only

  ang_x_prev = ang_x;
  ang_y_prev = ang_y;
  ang_z_prev = ang_z;

  switch (state) {
    case 0:
      if (move.forward(move.mmToPulses(330.0, diameter, ppr))) state++;
      Serial.print("Moving forward\n");
      break;
    case 1:
      if (move.right(move.mmToPulses(330.0, diameter, ppr))) state++;
      Serial.print("Moving right\n");
      break;
    case 2:
      if (move.backward(move.mmToPulses(330.0, diameter, ppr))) state++;
      Serial.print("Moving backward\n");
      break;
    case 3:
      if (move.left(move.mmToPulses(330.0, diameter, ppr))) state = 0;
      Serial.print("Moving left\n");
      break;

  }

    long currentLeftEncoderCount1 = encoderLeft.getEncoderCount();
    long currentRightEncoderCount1 = encoderRight.getEncoderCount();
    Serial.print((currentLeftEncoderCount1) / 600.0);   
    Serial.print(" , ");
    Serial.print((currentRightEncoderCount1) / 600.0);
    Serial.print(" , ");
    // Serial.print(state);
    Serial.print("\n");

    // turner.update(); // Call as often as possible  
    // turner.setTarget(0);   // Rotate to 90 degrees absolute

}