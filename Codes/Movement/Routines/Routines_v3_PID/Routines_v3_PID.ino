
#include <Arduino.h>
#include <AFMotor.h>
#include <QuadratureEncoder.h>
#include <Servo.h>
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

//define objects
MPU6050 sensor;
Pixy2 pixy;
Servo myservo;

// --- Microswitch Gyro Reset Extension ----------------------------------------------------------------

const byte switchPin = 18; // For interrupt on Mega
const byte switchPin1 = 19; // For interrupt on Mega

volatile bool switchPressed = false;
volatile bool switchPressed1 = false;
volatile unsigned long lastPressTime = 0;
bool lastSwitchState = HIGH;    // for edge detection
bool lastSwitchState1 = HIGH;    // for edge detection

//Rotor variables -------------------------------------------------------------------------
const int enable34 = 23; // L293D pin 9
const int input3   = 41; // L293D pin 10
const int input4   = 31; // L293D pin 15

// variables for GyroScope ----------------------------------------------------------------------------
// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
int ax, ay, az;
int gx, gy, gz;

//Variables usadas por el filtro pasa bajos
long f_ax,f_ay, f_az;
int p_ax, p_ay, p_az;
long f_gx,f_gy, f_gz;
int p_gx, p_gy, p_gz;

//Valor de los offsets
int ax_o,ay_o,az_o;
int gx_o,gy_o,gz_o;

long tiempo_prev;
float dt;
float ang_x, ang_y, ang_z; // Added ang_z for Z-axis rotation
float ang_x_prev, ang_y_prev, ang_z_prev; // Added ang_z_prev for Z-axis rotation

// variables for motor/encoders---------------------------------------------------
const int pwmValueA = 225; // Maximum PWM value (full speed)
const int pwmValueB = 225; // Maximum PWM value (full speed)
const unsigned long interval = 1000; // Interval for turning motors on and off (in milliseconds)
unsigned long lastMilli = 0;

//PID rotation
float Kp = 4.0;   // Proportional gain (tune as needed)
float Ki = 0.0;   // Integral gain (start with 0)
float Kd = 1.5;   // Derivative gain (tune as needed)
float pid_integral = 0;
float pid_prev_error = 0;
unsigned long pid_last_time = 0;

//---------------------------------
unsigned long lastTime = 0;
int step = 1;
static int routine = 4;

// functions--------------------------------------------------
int computePID(float setpoint, float measured) {
  unsigned long now = millis();
  float dt = (now - pid_last_time) / 1000.0;
  if (dt <= 0) dt = 0.01; // Prevent division by zero

  float error = setpoint - measured;
  pid_integral += error * dt;
  float derivative = (error - pid_prev_error) / dt;

  float output = Kp * error + Ki * pid_integral + Kd * derivative;

  pid_prev_error = error;
  pid_last_time = now;

  // Clamp output to motor PWM range
  if (output > 255) output = 255;
  if (output < -50) output = -100;
  return (int)output;
}

int filterGyro(MPU6050_Base sensor){
  sensor.getRotation(&gx, &gy, &gz);

  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();

  ang_z = ang_z_prev + (gz / 131.0) * dt; // Z-axis rotation using gyroscope only

  ang_z_prev = ang_z;
  return ang_z;
} 

void onSwitchPress() {
  switchPressed = true;
  switchPressed1 = true;
  lastPressTime = millis();
  resetGyroAngles();

}

void resetGyroAngles() {
  ang_x = ang_y = ang_z = 0;
  ang_x_prev = ang_y_prev = ang_z_prev = 0;
  tiempo_prev = millis(); 
}

void setup() {

  Serial.begin(9600);
  //rotor
  pinMode(enable34, OUTPUT);
  pinMode(input3, OUTPUT);
  pinMode(input4, OUTPUT);

  digitalWrite(enable34, HIGH);
  digitalWrite(input3, LOW);
  digitalWrite(input4, HIGH);

  // servo
  myservo.attach(10); 
  myservo.write(-90);

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
  pinMode(switchPin1, INPUT_PULLUP);

  pixy.init();
  pixy.setLamp(255, 0);
    //Calculate purple position
  int center_y = 55;
  int center_x = 200;

  for (int i = 0; i < 10; i++){
    delay(5);
  pixy.ccc.getBlocks();
  if (pixy.ccc.numBlocks > 0) {
    int ix = pixy.ccc.blocks[0].m_x; 
    int iy = pixy.ccc.blocks[0].m_y;
    //definir routine 
    if (ix < center_x && iy < center_y){
      routine = 1;
      Serial.print("upper left corner, case 1");
    } else if (ix > center_x && iy < center_y){
      routine = 3;
      Serial.print("upper right corner, case 3");
    } else if (ix < center_x && iy > center_y){
      routine = 0;
      Serial.print("lower left corner, case 0");
    } else if (ix > center_x && iy > center_y){
      routine = 2;
      Serial.print("lower Right corner, case 2");
    }
  }} 
  // else{routine = 4;}
          // Serial.print("No balls 🤨");





}

void loop() {
  //VAriables para los casos -----------------------------------------------------------
  static bool repeat = false;
  static int state = 0;
  static int beta = 8; //degree error
  static int theta = 10;
  static int alpha = 0;
  static int mili = 500; //delay
  int diameter = 60; //Diameter of the wheel in mm
  const long ppr = 800; // Number of pulses for each movement ste
  int closed = 95;
  int open = 0;
  int resetRoutine = 6;

  //activate rotor
  digitalWrite(enable34, HIGH);
  digitalWrite(input3, LOW);
  digitalWrite(input4, HIGH);

  //Read camera
  pixy.ccc.getBlocks();
  if (pixy.ccc.numBlocks > 0) {int x = pixy.ccc.blocks[0].m_x;}
  if (pixy.ccc.numBlocks > 0) {int y = pixy.ccc.blocks[0].m_y;}

  // Handle switch press
  bool currentSwitchState = digitalRead(switchPin);
  bool currentSwitchState1 = digitalRead(switchPin1);
  // if (lastSwitchState == HIGH && currentSwitchState == LOW && lastSwitchState1 == HIGH && currentSwitchState1 == LOW) {
  if (currentSwitchState == LOW && lastSwitchState == HIGH) {

    // Button pressed
    onSwitchPress();
    state++;
    Serial.println("Gyro angles reset by microswitch!");
  }
  lastSwitchState = currentSwitchState;
  lastSwitchState1 = currentSwitchState1;

  //MPU code
  // Leer las aceleraciones y velocidades angulares-----------------------------------
  // int ang_z = filterGyro(sensor);
  sensor.getRotation(&gx, &gy, &gz);

  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();

  ang_z = ang_z_prev + (gz / 131.0) * dt; // Z-axis rotation using gyroscope only

  ang_z_prev = ang_z;

  Serial.print("\tRotacion en Z: ");
  Serial.println(ang_z);

  //check if the robot is centered before acting---------------------------------
  // if (ang_z >= alpha + beta) {
  //   move.rotateCW(200, 200, 200, 200);
  // } else if (ang_z <= alpha - beta){
  //   move.rotateCCW(200, 200, 200, 200);
  // } else if (ang_z > alpha - beta || ang_z < alpha + beta){

int pid_pwm = computePID(alpha, ang_z);
if (abs(alpha - ang_z) > beta) {
  // Rotate with PID output
  int pwm = abs(pid_pwm);
  if (pid_pwm > 0) {
    move.rotateCCW(pwm, pwm, pwm, pwm);
  } else {
    move.rotateCW(pwm, pwm, pwm, pwm);
  }
  return; // Wait until angle is corrected before running routines
}

    //routine 0-3 = case 0-3 ball 
    //routine 4 left sweep
    //routine 5 rigth sweep
    //routine 6 left reset
    //routine 7 right reset
    //routine 8 follow ball

switch (routine) {// Routines --------------
    case 0: // Routine 0 -------------------------------------------------------------------
      myservo.write(open);
      switch (state) {
        case 0:
          if (move.accelerateForward(move.mmToPulses(400.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
          break;
        case 1:
          if (move.stopForMillis(mili)) state++;
          break;
        case 2:
          if (move.accelerateBackward(move.mmToPulses(410.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
          break;
        case 3:
          if (move.stopForMillis(mili)) state++;
          break;
        case 4:
          routine = resetRoutine; // Move to next routine
          state = 0; // Reset state for the next routine
          myservo.write(closed);
          break;
      }
    break;
    case 1: // Routine 1 ----------------------------------------------------
      myservo.write(open);
      switch (state) {  
      case 0:
        if (move.accelerateForward(move.mmToPulses(570.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
        break;
      case 1:
        if (move.stopForMillis(mili)) state++;
        break;
      case 2:
        if (move.accelerateBackward(move.mmToPulses(580.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
        break;
      case 3:
        if (move.stopForMillis(mili)) state++;
        break;
      case 4:
        routine = resetRoutine; // Move to next routine
        state = 0; // Reset state for the next routine
        myservo.write(closed);
        break;
      }
    break;
    case 2: // Routine 2 ----------------------------------------------------
      myservo.write(open);
      switch (state) {
      case 0:
        if (move.accelerateRight(move.mmToPulses(200.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
        break;
      case 1:
        if (move.stopForMillis(mili)) state++;
        break;
      case 2:
        if (move.accelerateForward(move.mmToPulses(370.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
        break;
      case 3:
        if (move.stopForMillis(mili)) state++;
        break;
      case 4:
        if (move.accelerateBackward(move.mmToPulses(380.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
        break;
      case 5:
        if (move.accelerateLeft(move.mmToPulses(400.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
        break;
      case 6:
        routine = resetRoutine; // Move to next routine
        state = 0; // Reset state for the next routine
        myservo.write(closed);
        break;
      }
    break;
    case 3:
      myservo.write(open);
      switch (state) {
        case 0:
          if (move.accelerateRight(move.mmToPulses(200.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
          break;
        case 1:
          if (move.stopForMillis(mili)) state++;
          break;
        case 2:
          if (move.accelerateForward(move.mmToPulses(450.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
          break;
        case 3:
          if (move.stopForMillis(mili)) state++;
          break;
        case 4:
          if (move.accelerateBackward(move.mmToPulses(440.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
          break;
        case 5:
          if (move.accelerateLeft(move.mmToPulses(400.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
          break;
        case 6:
          routine = resetRoutine; // Move to next routine
          state = 0; // Reset state for the next routine
          myservo.write(closed);
          break;
        }
    break;



    case 4: // Routine 4 // Left Sweeps -------------------------------------------------------------------
      switch (state) {
      case 0:
        if (move.accelerateForward(move.mmToPulses(500.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
        break;
      case 1:
        if (move.stopForMillis(mili)) state++;
        break;
      case 2:
        if (move.accelerateBackward(move.mmToPulses(520.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
        break;
      case 3:
        if (move.stopForMillis(mili)) state++;
        break;
      case 4:
        routine = 5; // Move to next routine
        state = 0; // Reset state for the next routine
        break;
      }
      break;
    case 5: // Right Sweeps----------------------------------------------------
      switch (state) {
      case -1:
        if (move.accelerateRight(move.mmToPulses(80.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state = 1;
        break;
      case 0:
        if (move.accelerateRight(move.mmToPulses(120.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
        break;
      case 1:
        if (move.stopForMillis(mili)) state++;
        break;
      case 2:
        if (move.accelerateForward(move.mmToPulses(500.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
        break;
      case 3:
        if (move.stopForMillis(mili)) state++;
        break;
      case 4:
        if (move.accelerateBackward(move.mmToPulses(520.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
        break;
      case 5:
        if (move.stopForMillis(mili)) state++;
        break;
      case 6:
        if (move.accelerateLeft(move.mmToPulses(50.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
        break;
      case 7:
        if (move.stopForMillis(mili)) state++;
        break;
      case 8:
        if (repeat == false) {
            routine = resetRoutine; // Move to next routine
            state = 0; // Reset state for the next routine
            repeat = true;
        } else {
            state = -1;
            repeat = false;
        }
      }
      break;
      case 6: // left reset -------------------------------------------------------------
        switch(state){
          case 0:
            if (move.accelerateBackward(move.mmToPulses(150.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
            break;
          case 1:
            if (move.stopForMillis(mili)) state++;
            break;
          case 2:
            if (move.accelerateLeft(move.mmToPulses(200.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
            break;
          case 3:
            if (move.stopForMillis(mili)) state++;
            break;
          case 4:
            if (move.accelerateRight(move.mmToPulses(70, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
          case 5:
            routine = 4; // Move to next routine
            state = 0; // Reset state for the next routine
            break;
        }
        break;
        case 7: // Routine // Right reset -----------------------------------
          switch(state){
            case 0:
              if (move.accelerateBackward(move.mmToPulses(550.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
              break;
            case 1:
              if (move.stopForMillis(mili)) state++;
              break;
            case 2:
              if (move.accelerateRight(move.mmToPulses(350.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
              break;
            case 3:
              if (move.stopForMillis(mili)) state++;
              break;
            case 4:
              if(move.accelerateLeft(move.mmToPulses(200, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
            case 5:
              routine = 4; // Move to next routine
              state = 0; // Reset state for the next routine
              break;
          }
          // case 8: //Routine 8 //Follow Ball
          //   // if (pixy.ccc.numBlocks){
          //   //   int x = pixy.ccc.blocks[i].m_x;
          //   // if (x < 140)
          //   // { 
          //   //   move.simpleLeft();
          //   //   Serial.print("MOVE RIGHT\n");
          //   //   pixy.ccc.blocks[i].print();
          //   // }else if (x > 240){
          //   //   move.simpleRight();
          //   //   Serial.print("MOVE LEFT\n");
          //   //   pixy.ccc.blocks[i].print();

          //   // } else if (x < 240 && x > 140){
          //   //     pixy.ccc.blocks[i].print();
          //   //     move.simpleForward();
          //   // }
          //   break;
  // } 
  }
}

