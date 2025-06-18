
#include <Arduino.h>
#include <AFMotor.h>
#include <QuadratureEncoder.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "C:\Users\samue\OneDrive\Documents\GitHub\singapore25\Codes\Movement\Encoder\Ecoder measured movement\lib\move\move.h"

#include <Pixy2.h>

AF_DCMotor motor1(1); // Motor 1 on the Adafruit Motor Shield
AF_DCMotor motor2(2); // Motor 2 on the Adafruit Motor Shield
AF_DCMotor motor3(3); // Motor 3 on the Adafruit Motor Shield
AF_DCMotor motor4(4); // Motor 4 on the Adafruit Motor Shield

Encoders encoderLeft(A15, A13);	// Create an Encoder object name leftEncoder, using digitalpin 2 & 3
Encoders encoderRight(A14 , A12); // Encoder object name rightEncoder using analog pin A0 and A1 

Move move(motor1, motor2, motor3, motor4, encoderLeft, encoderRight);

MPU6050 sensor;
Pixy2 pixy;

// --- Microswitch Gyro Reset Extension ----------------------------------------------------------------

const byte switchPin = 18; // For interrupt on Mega
volatile bool switchPressed = false;
volatile unsigned long lastPressTime = 0;
bool lastSwitchState = HIGH;    // for edge detection

// variables for GyroScope ----------------------------------------------------------------------------
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

int alpha = 0;//-------------------------------ɑ

long tiempo_prev;
float dt;
float ang_x, ang_y, ang_z; // Added ang_z for Z-axis rotation
float ang_x_prev, ang_y_prev, ang_z_prev; // Added ang_z_prev for Z-axis rotation

// variables for motor/encoders---------------------------------------------------
const int pwmValueA = 225; // Maximum PWM value (full speed)
const int pwmValueB = 225; // Maximum PWM value (full speed)
const unsigned long interval = 1000; // Interval for turning motors on and off (in milliseconds)
unsigned long lastMilli = 0;

//---------------------------------
unsigned long lastTime = 0;
int step = 1;
static int routine = 0;

// functions--------------------------------------------------
void filterGyro(gy){} 

void onSwitchPress() {
  switchPressed = true;
  lastPressTime = millis();
}

void resetGyroAngles() {
  ang_x = ang_y = ang_z = 0;
  ang_x_prev = ang_y_prev = ang_z_prev = 0;
  tiempo_prev = millis(); // <-- Add this line
}

void setup() {

  Serial.begin(9600);

  //mpu 
    Wire.begin();           // Iniciando I2C
  sensor.initialize();    // Iniciando el sensor

  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");

  tiempo_prev = millis();
  resetGyroAngles();

  //movement
  move.begin(235, 235, 235, 235);
  pinMode(switchPin, INPUT_PULLUP);

  pixy.init();
  pixy.setLamp(255, 0);
    //Calculate purple position
  for (int i = 0; i < 10; i++){
    delay(10);
    pixy.ccc.getBlocks();
    if (pixy.ccc.numBlocks > 0) {
      int ix = pixy.ccc.blocks[0].m_x; 
      int iy = pixy.ccc.blocks[0].m_y;
      //definir routine 
      if (ix < 140 && iy < 80){
        routine = 1;
      } else if (ix > 240 && iy < 80){
        routine = 2;
      } else if (ix < 140 && iy > 120){
        routine = 0;
      } else if (ix > 240 && iy > 120){
        routine = 2;
      }
    } else{routine = 0;}
  }


}

void loop() {
  //Read camera
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks > 0) {int x = pixy.ccc.blocks[0].m_x;}
  if (pixy.ccc.numBlocks > 0) {int y = pixy.ccc.blocks[0].m_y;}
  // Handle switch press
  bool currentSwitchState = digitalRead(switchPin);
  if (lastSwitchState == HIGH && currentSwitchState == LOW) {
    // Button pressed
    resetGyroAngles();
    Serial.println("Gyro angles reset by microswitch!");
  }
  lastSwitchState = currentSwitchState;

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

  static int state = 0;
  static int beta = 20; //degree error
  static int mili = 500; //delay
  int diameter = 60; //Diameter of the wheel in mm
  const long ppr = 1200; // Number of pulses for each movement step

  //check if the robot is centered before acting
  if (ang_z >= alpha + beta) {
    move.rotateCW(240, 240, 240, 240); 
  } else if (ang_z <= alpha - beta){
    move.rotateCCW(240, 240, 240, 240);
  } else if (ang_z > alpha - beta || ang_z < alpha + beta){
switch (routine) {// Routines --------------
    case 0: // Routine 0 ----------------------------------------------------
      switch (state) {
        case 0:
          if (move.forward(move.mmToPulses(300.0, diameter, ppr))) state++;
          break;
        case 1:
          if (move.stopForMillis(mili)) state++;
          break;
        case 2:
          if (move.backward(move.mmToPulses(310.0, diameter, ppr))) state++;
          break;
        case 3:
          if (move.stopForMillis(mili)) state++;
          break;
        case 4:
          routine = 1; // Move to next routine
          state = 0; // Reset state for the next routine
          break;
      }
    break;
    case 1: // Routine 1 ----------------------------------------------------
      switch (state) {  
      case 0:
        if (move.forward(move.mmToPulses(360.0, diameter, ppr))) state++;
        break;
      case 1:
        if (move.stopForMillis(mili)) state++;
        break;
      case 2:
        if (move.backward(move.mmToPulses(350.0, diameter, ppr))) state++;
        break;
      case 3:
        if (move.stopForMillis(mili)) state++;
        break;
      case 4:
        routine = 2; // Move to next routine
        state = 0; // Reset state for the next routine
        break;
      }
    break;
    case 2: // Routine 2 ----------------------------------------------------
      switch (state) {
      case 0:
        if (move.right(move.mmToPulses(110.0, diameter, ppr))) state++;
        break;
      case 1:
        if (move.stopForMillis(mili)) state++;
        break;
      case 2:
        if (move.forward(move.mmToPulses(350.0, diameter, ppr))) state++;
        break;
      case 3:
        if (move.stopForMillis(mili)) state++;
        break;
      case 4:
        if (move.backward(move.mmToPulses(340.0, diameter, ppr))) state++;
        break;
      case 5:
        if (move.left(move.mmToPulses(400.0, diameter, ppr))) state++;
        break;
      case 6:
        routine = 3; // Move to next routine
        state = 0; // Reset state for the next routine
        break;
      }
    break;
    
    
    case 3: // Routine 4 // Barrido izquierda ---------------------------------------------------- Barridos ---------------
      switch (state) {
      case 0:
        if (move.forward(move.mmToPulses(400.0, diameter, ppr))) state++;//---------------------------------------------
        break;
      case 1:
        if (move.stopForMillis(mili)) state++;
        break;
      case 2:
        if (move.backward(move.mmToPulses(420.0, diameter, ppr))) state++;//--------------=--------------------------=-------=--=-=-=-==-=--
        break;
      case 3:
        if (move.stopForMillis(mili)) state++;
        break;
      case 4:
        routine = 4; // Move to next routine
        state = 0; // Reset state for the next routine
        break;
      }
      break;
    case 4: // Routine 5 ----------------------------------------------------
      switch (state) {
      case 0:
        if (move.right(move.mmToPulses(220.0, diameter, ppr))) state++;
        break;
      case 1:
        if (move.stopForMillis(mili)) state++;
        break;
      case 2:
        if (move.forward(move.mmToPulses(500.0, diameter, ppr))) state++;
        break;
      case 3:
        if (move.stopForMillis(mili)) state++;
        break;
      case 4:
        if (move.backward(move.mmToPulses(550.0, diameter, ppr))) state++;
        break;
      case 5:
        if (move.stopForMillis(mili)) state++;
        break;
      case 6:
        if (move.left(move.mmToPulses(570.0, diameter, ppr))) state++;
        break;
      case 7:
        if (move.stopForMillis(mili)) state++;
        break;
      case 8:
        routine = 3; // Move to next routine
        state = 0; // Reset state for the next routine
        break;
      }
      break;
  } 
  }

}

