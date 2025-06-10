#pragma once
#include <Arduino.h>
#include "C:\Users\samue\OneDrive\Documents\GitHub\singapore25\Codes\libraries\MPU6050\MPU6050.h"
#include "C:\Users\samue\OneDrive\Documents\GitHub\singapore25\Codes\Movement\Encoder\Ecoder measured movement\lib\move\move.h" // Your Move class

class GyroTurn {
public:
    GyroTurn(MPU6050& mpu, Move& mover)
        : sensor(mpu), move(mover) {}

    // Call in setup or after calibration to set current heading as reference
    void resetHeading(float startAngle = 0.0) {
        heading = normalizeAngle(startAngle);
        ang_z_prev = heading;
        lastUpdate = millis();
        sensor.getRotation(nullptr, nullptr, &gz_prev);
    }

    // Set the absolute target angle (degrees, 0-360)
    void setTarget(float angle) {
        target = normalizeAngle(angle);
        turning = true;
    }

    // Call this frequently in loop()
    void update() {
        // Read gyro Z and integrate angle
        int16_t gz;
        sensor.getRotation(nullptr, nullptr, &gz);

        unsigned long now = millis();
        float dt = (now - lastUpdate) / 1000.0;
        lastUpdate = now;

        heading = ang_z_prev + (gz / 131.0) * dt;
        heading = normalizeAngle(heading);
        ang_z_prev = heading;

        if (!turning) return;

        float error = smallestAngleDiff(target, heading);

        if (abs(error) < tolerance) {
            move.stop();
            turning = false;
            return;
        }

        if (error > 0) {
            move.rotateCW();
        } else {
            move.rotateCCW();
        }
    }

    // Returns true if rotation is done
    bool done() const { return !turning; }

    // Get current heading estimate (0-360)
    float getHeading() const { return normalizeAngle(heading); }

    // Set tolerance (degrees)
    void setTolerance(float tol) { tolerance = tol; }

private:
    MPU6050& sensor;
    Move& move;
    float heading = 0;
    float ang_z_prev = 0;
    float target = 0;
    float tolerance = 3;
    bool turning = false;
    unsigned long lastUpdate = 0;
    int16_t gz_prev = 0;

    // Normalize angle to [0, 360)
    static float normalizeAngle(float angle) {
        while (angle < 0) angle += 360;
        while (angle >= 360) angle -= 360;
        return angle;
    }

    // Returns minimal signed difference (-180, 180]
    static float smallestAngleDiff(float target, float current) {
        float diff = target - current;
        while (diff < -180) diff += 360;
        while (diff > 180) diff -= 360;
        return diff;
    }
};

int rotationReading(int gz){
    sensor.getRotation(&gz);

    dt = (millis() - tiempo_prev) / 1000.0;
    tiempo_prev = millis();

  // Calcular los ángulos con acelerometro

  // Calcular angulo de rotación con giroscopio y filtro complemento
//   ang_x = 0.98 * (ang_x_prev + (gx / 131.0) * dt) + 0.02 * accel_ang_x;
//   ang_y = 0.98 * (ang_y_prev + (gy / 131.0) * dt) + 0.02 * accel_ang_y;
  ang_z = ang_z_prev + (gz / 131.0) * dt; // Z-axis rotation using gyroscope only

  ang_x_prev = ang_x;
  ang_y_prev = ang_y;
  ang_z_prev = ang_z;
}