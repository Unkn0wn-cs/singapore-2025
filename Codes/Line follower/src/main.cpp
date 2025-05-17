#include <Arduino.h>
#include <Wire.h> // Include the Wire library for I2C communication
#include <AFMotor.h> // Include the Adafruit Motor Shield library

// Define motors
AF_DCMotor motor1(1); // Motor 1 on the Adafruit Motor Shield (left motor)
AF_DCMotor motor2(2); // Motor 2 on the Adafruit Motor Shield (right motor)

// I2C address for the line-following sensor
#define LINE_FOLLOWER_I2C_ADDR 0x78 // I2C address for the line-following module

int base_speed = 50; // Base speed for motors

uint8_t data; // Variable to store sensor data

// Add state variables to track hard turns
int hardTurnCounter = 0;
const int hardTurnDuration = 5; // Number of iterations to persist hard turn behavior
enum HardTurnState { NONE, LEFT, RIGHT };
HardTurnState lastHardTurn = NONE;

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

    motor1.run(FORWARD);
    motor2.run(FORWARD);

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

    // Check if a hard turn is ongoing
    if (hardTurnCounter > 0) {
        hardTurnCounter--;
        if (lastHardTurn == LEFT) {
            motor1.setSpeed(0);
            motor2.setSpeed(base_speed * 2);
            Serial.println("90 degrees left (persisting)");
        } else if (lastHardTurn == RIGHT) {
            motor1.setSpeed(base_speed * 2);
            motor2.setSpeed(0);
            Serial.println("90 degrees right (persisting)");
        }
        return;
    }

    // Determine motor actions based on sensor data
    if ((data & 0x01) && (data & 0x02) && not (data & 0x04) && not (data & 0x08)) { // Line detected on horizontal left
        motor1.setSpeed(0);
        motor2.setSpeed(base_speed * 2);
        Serial.println("90 degrees left");
        lastHardTurn = LEFT;
        hardTurnCounter = hardTurnDuration;
    } else if ((data & 0x04) && (data & 0x08) && not (data & 0x01) && not (data & 0x02)) { // Line detected on horizontal right
        motor1.setSpeed(base_speed * 2);
        motor2.setSpeed(0);
        Serial.println("90 degrees right");
        lastHardTurn = RIGHT;
        hardTurnCounter = hardTurnDuration;
    } else if ((data & 0x01) && not (data & 0x08)) { // Line detected on the far left
        motor1.setSpeed(base_speed * 0.6);
        motor2.setSpeed(base_speed * 1.4);
        Serial.println("Turn Left");
        lastHardTurn = NONE;
    } else if ((data & 0x08) && not (data & 0x01)) { // Line detected on the far right
        motor1.setSpeed(base_speed * 1.4);
        motor2.setSpeed(base_speed * 0.6);
        Serial.println("Turn Right");
        lastHardTurn = NONE;
    } else if (data & 0x02 && data & 0x04) { // Line detected in the center
        motor1.setSpeed(base_speed * 1.2);
        motor2.setSpeed(base_speed * 1.2);
        Serial.println("Forward");
        lastHardTurn = NONE;
    } else if ((data & 0x02) && not (data & 0x04)) {
        motor1.setSpeed(base_speed * 1.2);
        motor2.setSpeed(base_speed * 0.8);
        Serial.println("Turn Left");
        lastHardTurn = NONE;
    } else if ((data & 0x04) && not (data & 0x02)) {
        motor1.setSpeed(base_speed * 0.8);
        motor2.setSpeed(base_speed * 1.2);
        Serial.println("Turn Right");
        lastHardTurn = NONE;
    } else { // No line detected
        motor1.setSpeed(15); // Stop
        motor2.setSpeed(15);
        Serial.println("Stop");
        lastHardTurn = NONE;
    }
    delay(10); // Small delay for stability
}

