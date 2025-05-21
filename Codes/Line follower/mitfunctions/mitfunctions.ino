#include <Arduino.h>
#include <Wire.h> // Include the Wire library for I2C communication
#include <AFMotor.h> // Include the Adafruit Motor Shield library

// Define motors
AF_DCMotor motor1(3); // Motor 1 on the Adafruit Motor Shield (left motor)
AF_DCMotor motor2(4); // Motor 2 on the Adafruit Motor Shield (right motor)

// I2C address for the line-following sensor
#define LINE_FOLLOWER_I2C_ADDR 0x78 // I2C address for the line-following module
bool back = false;
int white_counter = 0;

// PID constants
float Kp = 10;  // Proportional gain
float Ki = 0;  // Integral gain
float Kd = 0.5;  // Derivative gain

// PID variables
float error = 0, previousError = 0, integral = 0, derivative = 0;
float correction = 0;

// Motor speed
int baseSpeed = 100; // Base speed for motors
int leftMotorSpeed = 0, rightMotorSpeed = 0;

// Function to write a byte to the I2C device
bool WireWriteByte(uint8_t val) {
    Wire.beginTransmission(LINE_FOLLOWER_I2C_ADDR);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

// Function to read a byte from the I2C device
bool WireReadDataByte(uint8_t reg, uint8_t &val) {
    if (!WireWriteByte(reg)) {
        return false;
    }
    Wire.requestFrom(LINE_FOLLOWER_I2C_ADDR, 1);
    if (Wire.available()) {
        val = Wire.read();
        return true;
    }
    return false;
}

// Function to read sensor values
void readSensorValues(int sensorValues[4]) {
    uint8_t sensorData;
    if (WireReadDataByte(1, sensorData)) {
        sensorValues[0] = sensorData & 0x01;       // Sensor 1
        sensorValues[1] = (sensorData >> 1) & 0x01; // Sensor 2
        sensorValues[2] = (sensorData >> 2) & 0x01; // Sensor 3
        sensorValues[3] = (sensorData >> 3) & 0x01; // Sensor 4
    } else {
        Serial.println("Failed to read sensor data");
    }
}

// Function to calculate error based on sensor readings
float calculateError(int sensorValues[4]) {
    int weights[4] = {4, 1, -1, -4}; // Weights for each sensor
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
    motor1.setSpeed(constrain(leftSpeed, 0, 255));
    motor2.setSpeed(constrain(rightSpeed, 0, 255));

    // Always run motors forward
    motor1.run(FORWARD);
    motor2.run(FORWARD);
}

bool white(){ //tests if there's no line 
    if ((sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3]) == 0) 
    // no line detected
    {   
        return true;
    }
}

void is_white(){ 
        white_counter++;
        if (white_counter > 100){
            motor1.setSpeed(baseSpeed);
            motor2.setSpeed(baseSpeed);
            motor1.run(BACKWARD);
            motor2.run(BACKWARD);
            back = true;
            white_counter=0;
        }
}
void turn_left() {
            back = false;
            while motor1.run(BACKWARD);
            motor2.run(FORWARD);
            motor2.setSpeed(baseSpeed * 2);
            motor1.setSpeed(baseSpeed *2);
            delay(100);
            motor1.run(FORWARD);
            motor2.run(FORWARD);
}

void turn_right() {
            back = false;
            motor1.run(FORWARD);
            motor2.run(FORWARD);
            delay(100);
            motor1.run(FORWARD);
            motor2.run(BACKWARD);
            motor2.setSpeed(baseSpeed * 2);
            motor1.setSpeed(baseSpeed *2);
            delay(100);
            motor1.run(FORWARD);
            motor2.run(FORWARD);
            back = false;
}

void setup() {
    // Initialize I2C communication
    Wire.begin();

    // Set initial motor speeds to 0
    motor1.setSpeed(200);
    motor2.setSpeed(200);

    // Set motor directions to forward
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    

    // Start serial communication for debugging
    Serial.begin(9600);
    delay(50);
}

void loop() {
    // Read sensor values
    int sensorValues[4];
    readSensorValues(sensorValues);

    // Calculate error based on sensor readings
    error = calculateError(sensorValues);
    
    if (white()) 
    // no line detected
    {   
        is_white();
    }
    
    else if ((sensorValues[0] || sensorValues[1] || sensorValues[2] || sensorValues[3]) && back == true){ //line detected but going back
        //motor run foward and turn 90 if a corner is near
        if (sensorValues[0] && sensorValues[1] && !(sensorValues[3]))
        {   
            turn_left();
        }
        else if (sensorValues[2] && sensorValues[3] && !(sensorValues[1]))
        {   
            turn_right()
        }
        back = false;
        motor1.run(FORWARD);
        motor2.run(FORWARD);
    } else {
        back = false;
        motor1.run(FORWARD);
        motor2.run(FORWARD);
    // Calculate PID terms
    integral += error;
    derivative = error - previousError;
    correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // Adjust motor speeds based on correction
    leftMotorSpeed = baseSpeed - correction;
    rightMotorSpeed = baseSpeed + correction;

    // Set motor speeds
    setMotorSpeed(leftMotorSpeed, rightMotorSpeed);

    // Update previous error
    previousError = error;
    }

}