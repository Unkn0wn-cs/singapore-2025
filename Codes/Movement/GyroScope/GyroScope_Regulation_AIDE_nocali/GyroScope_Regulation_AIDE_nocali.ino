#include <Arduino.h>
#include <AFMotor.h>
#include <QuadratureEncoder.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 sensor;
// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
int ax, ay, az;
int gx, gy, gz;

//Variables usadas por el filtro pasa bajos
long f_ax,f_ay, f_az;
int p_ax, p_ay, p_az;
long f_gx,f_gy, f_gz;
int p_gx, p_gy, p_gz;
int counter=0;
int calibrated;


//Valor de los offsets
int ax_o,ay_o,az_o;
int gx_o,gy_o,gz_o;

int alpha = 0;
long tiempo_prev;
float dt;
float ang_x, ang_y, ang_z; // Added ang_z for Z-axis rotation
float ang_x_prev, ang_y_prev, ang_z_prev; // Added ang_z_prev for Z-axis rotation


// Define motors & encoders
AF_DCMotor motor1(1); // Motor 1 on the Adafruit Motor Shield
AF_DCMotor motor2(2); // Motor 2 on the Adafruit Motor Shield
AF_DCMotor motor3(3); // Motor 3 on the Adafruit Motor Shield
AF_DCMotor motor4(4); // Motor 4 on the Adafruit Motor Shield
// Encoders leftEncoder1(A8, A9); // Encoder for motor1
// Encoders leftEncoder2(A10, A11); // Encoder for motor2
// Encoders rightEncoder1(A12, A13); // Encoder for motor3
// Encoders rightEncoder2(A14, A15); // Encoder for motor4

const int pwmValueA = 225; // Maximum PWM value (full speed)
const int pwmValueB = 225; // Maximum PWM value (full speed)
const unsigned long interval = 1000; // Interval for turning motors on and off (in milliseconds)
unsigned long lastMilli = 0;

unsigned long lastTime = 0;
int step = 1;

class Move {
  public:
    void forward() {
      motor1.run(FORWARD);
      motor2.run(FORWARD);
      motor3.run(FORWARD);
      motor4.run(FORWARD);
    }

    void backward() {
      motor1.run(BACKWARD);
      motor2.run(BACKWARD);
      motor3.run(BACKWARD);
      motor4.run(BACKWARD);
    }

    void left() {
      motor1.run(FORWARD);
      motor2.run(FORWARD);
      motor3.run(BACKWARD);
      motor4.run(BACKWARD);
    }

    void right() {
      motor1.run(BACKWARD);
      motor2.run(BACKWARD);
      motor3.run(FORWARD);
      motor4.run(FORWARD);
    }

    void stop() {
      motor1.run(RELEASE);
      motor2.run(RELEASE);
      motor3.run(RELEASE);
      motor4.run(RELEASE);
    }

    void rotate(bool clockwise, int speed) {
      motor1.setSpeed(speed);
      motor2.setSpeed(speed);
      motor3.setSpeed(speed);
      motor4.setSpeed(speed);
      if (clockwise) {
        motor1.run(BACKWARD);
        motor2.run(FORWARD);
        motor3.run(FORWARD);
        motor4.run(BACKWARD);
      } else {
        motor1.run(FORWARD);
        motor2.run(BACKWARD);
        motor3.run(BACKWARD);
        motor4.run(FORWARD);
      }
    }
};

Move move;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
    Wire.begin();           // Iniciando I2C
  sensor.initialize();    // Iniciando el sensor

  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");

  motor1.setSpeed(pwmValueA);
  motor2.setSpeed(pwmValueB);
  motor3.setSpeed(pwmValueA);
  motor4.setSpeed(pwmValueB);
}

void loop() {
  //MPU code
  // Leer las aceleraciones y velocidades angulares
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();

  ang_y = ang_y_prev + (gy / 131.0) * dt; // Z-axis rotation using gyroscope only

  ang_x_prev = ang_x;
  ang_y_prev = ang_y;
  ang_z_prev = ang_z;

  Serial.print("\tRotacion en Z: ");
  Serial.println(ang_z);

  if (ang_y >= alpha + 10) {
    move.rotate(true, 180); 
  } else if (ang_y <= alpha - 10){
    move.rotate(false, 180);
  } else if (ang_y > alpha -10 || ang_y < alpha + 10){
      motor1.setSpeed(255);
  motor2.setSpeed(255);
  motor3.setSpeed(255);
  motor4.setSpeed(255);
  move.forward();
  }

}

