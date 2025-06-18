#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

const byte switchPin = 18; // For interrupt on Mega

volatile bool switchPressed = false;
volatile unsigned long lastPressTime = 0;

MPU6050 sensor;

int ax, ay, az;
int gx, gy, gz;

long tiempo_prev;
float dt;
float ang_x, ang_y, ang_z;
float ang_x_prev, ang_y_prev, ang_z_prev;

void onSwitchPress() {
  switchPressed = true;
  lastPressTime = millis();
}

void resetGyroAngles() {
  ang_x = ang_y = ang_z = 0;
  ang_x_prev = ang_y_prev = ang_z_prev = 0;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  sensor.initialize();

  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");

  pinMode(switchPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(switchPin), onSwitchPress, FALLING);

  tiempo_prev = millis();
  resetGyroAngles();
}

void loop() {
  // Handle switch press
  if (switchPressed) {
    unsigned long pressTime;
    noInterrupts();
    pressTime = lastPressTime;
    switchPressed = false;
    interrupts();

    Serial.print("Switch pressed at: ");
    Serial.println(pressTime);

    resetGyroAngles();
    Serial.println("Gyro angles reset!");
  }

  // Leer las aceleraciones y velocidades angulares
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();

  float accel_ang_x = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / 3.14);
  float accel_ang_y = atan(-ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / 3.14);

  ang_x = 0.98 * (ang_x_prev + (gx / 131.0) * dt) + 0.02 * accel_ang_x;
  ang_y = 0.98 * (ang_y_prev + (gy / 131.0) * dt) + 0.02 * accel_ang_y;
  ang_z = ang_z_prev + (gz / 131.0) * dt;

  ang_x_prev = ang_x;
  ang_y_prev = ang_y;
  ang_z_prev = ang_z;

  Serial.print("Rotacion en X:  ");
  Serial.print(ang_x);
  Serial.print("\tRotacion en Y: ");
  Serial.print(ang_y);
  Serial.print("\tRotacion en Z: ");
  Serial.println(ang_z);

  delay(10); // Optional: to avoid flooding serial output
}