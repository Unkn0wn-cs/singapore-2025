#include "C:\Users\samue\OneDrive\Documents\GitHub\singapore25\Codes\Movement\Encoder\Ecoder measured movement\lib\move\move.h"
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


#include <Pixy2.h>

// This is the main Pixy object 
Pixy2 pixy;

AF_DCMotor motor1(1); // Motor 1 on the Adafruit Motor Shield
AF_DCMotor motor2(2); // Motor 2 on the Adafruit Motor Shield
AF_DCMotor motor3(3); // Motor 3 on the Adafruit Motor Shield
AF_DCMotor motor4(4); // Motor 4 on the Adafruit Motor Shield
Encoders encoderLeft(A15, A13);  // Motor 3
Encoders encoderRight(A14, A12); // Motor 4

Move move(motor1, motor2, motor3, motor4, encoderLeft, encoderRight);


void setup()
{
  move.begin(250,250,250,250); //dfine the power
  Serial.begin(115200);
  Serial.print("Starting...\n");
  pixy.init();
  pixy.setLamp(255, 0);

  //mpu6050
  Wire.begin();           // Iniciando I2C
  sensor.initialize();    // Iniciando el sensor

}

void loop()
{ 
  //MPU code
  // Leer las aceleraciones y velocidades angulares
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();

  ang_z = ang_z_prev + (gz / 131.0) * dt; // Z-axis rotation using gyroscope only

  ang_x_prev = ang_x;
  ang_y_prev = ang_y;
  ang_z_prev = ang_z;

  Serial.print("\tRotacion en Z: ");
  Serial.println(ang_z);

  int pwm = 255;
  int i = 0; 
  // grab blocks!
  pixy.ccc.getBlocks();
  // If there are detect blocks, print them!
  if (ang_z >= alpha + 10) {
    move.rotateCCW(180, 180, 180, 180); 
  } else if (ang_z <= alpha - 10){
    move.rotateCW(180, 180, 180, 180);
  } else if (ang_z > alpha -10 || ang_z < alpha + 10){
    move.begin(250,250,250,250); //dfine the power
    if (pixy.ccc.numBlocks){
      int x = pixy.ccc.blocks[i].m_x;
        if (x < 140)
      { 
        move.simpleLeft();
        Serial.print("MOVE RIGHT\n");
        pixy.ccc.blocks[i].print();

      } else if (x > 240){
        move.simpleRight();
        Serial.print("MOVE LEFT\n");
          pixy.ccc.blocks[i].print();

      } else if (x < 240 && x > 140){
              pixy.ccc.blocks[i].print();
              move.simpleForward();

    } 
    }else{
      move.stop();
    }
}
}