#include "cali.h"
#include <Arduino.h>

namespace Gyro {

void calibrateMPU6050(MPU6050& sensor, int samples) {
    long ax_sum = 0, ay_sum = 0, az_sum = 0;
    long gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int ax, ay, az, gx, gy, gz;

    // Collect samples
    for (int i = 0; i < samples; ++i) {
        sensor.getAcceleration(&ax, &ay, &az);
        sensor.getRotation(&gx, &gy, &gz);
        ax_sum += ax;
        ay_sum += ay;
        az_sum += az;
        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;
        delay(5);
    }

    // Calculate averages
    int ax_avg = ax_sum / samples;
    int ay_avg = ay_sum / samples;
    int az_avg = az_sum / samples;
    int gx_avg = gx_sum / samples;
    int gy_avg = gy_sum / samples;
    int gz_avg = gz_sum / samples;

    // Get current offsets
    int ax_off = sensor.getXAccelOffset();
    int ay_off = sensor.getYAccelOffset();
    int az_off = sensor.getZAccelOffset();
    int gx_off = sensor.getXGyroOffset();
    int gy_off = sensor.getYGyroOffset();
    int gz_off = sensor.getZGyroOffset();

    // Adjust offsets: accel Z should be 16384 (1g), others should be 0
    sensor.setXAccelOffset(ax_off - ax_avg);
    sensor.setYAccelOffset(ay_off - ay_avg);
    sensor.setZAccelOffset(az_off + (16384 - az_avg));
    sensor.setXGyroOffset(gx_off - gx_avg);
    sensor.setYGyroOffset(gy_off - gy_avg);
    sensor.setZGyroOffset(gz_off - gz_avg);
}

}