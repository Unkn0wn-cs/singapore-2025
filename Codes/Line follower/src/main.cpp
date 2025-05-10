#include <Arduino.h>
#include <Wire.h> // Include the Wire library for I2C communication
#include <AFMotor.h> // Include the Adafruit Motor Shield library

// Define motors
AF_DCMotor motor1(1); // Motor 1 on the Adafruit Motor Shield (left motor)
AF_DCMotor motor2(2); // Motor 2 on the Adafruit Motor Shield (right motor)

// I2C address for the line-following sensor
#define LINE_FOLLOWER_I2C_ADDR 0x78 // I2C address for the line-following module

uint8_t data; // Variable to store sensor data

// Function to write a byte to the I2C device
bool WireWriteByte(uint8_t val) {
    Wire.beginTransmission(LINE_FOLLOWER_I2C_ADDR);
    Wire.write(val);
    if (Wire.endTransmission() != 0) {
        Serial.println("false");
        return false;
    }
    Serial.println("true");
    return true;
}

// Function to read a byte from the I2C device
bool WireReadDataByte(uint8_t reg, uint8_t &val) {
    if (!WireWriteByte(reg)) {
        return false;
    }
    Wire.requestFrom(LINE_FOLLOWER_I2C_ADDR, 1);
    while (Wire.available()) {
        val = Wire.read();
    }
    return true;
}

void setup() {
    // Initialize I2C communication
    Wire.begin();

    // Set initial motor speeds
    motor1.setSpeed(255);
    motor2.setSpeed(255);

    // Start serial communication for debugging
    Serial.begin(9600);
}

void loop() {
    // Read sensor data
    if (!WireReadDataByte(1, data)) {
        Serial.println("Failed to read sensor data");
        return;
    }

    // Debugging: Print sensor data
    Serial.print("Sensor1: "); Serial.print(data & 0x01);
    Serial.print("  Sensor2: "); Serial.print((data >> 1) & 0x01);
    Serial.print("  Sensor3: "); Serial.print((data >> 2) & 0x01);
    Serial.print("  Sensor4: "); Serial.println((data >> 3) & 0x01);

    // Determine motor actions based on sensor data
    if (data & 0x01) { // Line detected on the far left
        motor1.run(BACKWARD); // Turn left
        motor2.run(FORWARD);
        Serial.println("Turn Left");

    } else if (data & 0x08) { // Line detected on the far right
        motor1.run(FORWARD); // Turn right
        motor2.run(BACKWARD);
        Serial.println("Turn Right");
    } else if (data & 0x02 || data & 0x04) { // Line detected in the center
        motor1.run(FORWARD); // Move forward
        motor2.run(FORWARD);
        Serial.println("Forward");
    } else { // No line detected
        motor1.run(RELEASE); // Stop
        motor2.run(RELEASE);
        Serial.println("Stop");
    }

    // Small delay for stability
    delay(5000);
}

