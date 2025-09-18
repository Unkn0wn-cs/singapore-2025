#include <Arduino.h>
#include <AFMotor.h>
#include <QuadratureEncoder.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "C:\Users\samue\OneDrive\Documents\GitHub\singapore25\Codes\Movement\Encoder\Ecoder measured movement\lib\move\move.h"

AF_DCMotor motor1(1); 
AF_DCMotor motor2(2); // Motor 2 on the Adafruit Motor Shield
AF_DCMotor motor3(3); // Motor 3 on the Adafruit Motor Shield
AF_DCMotor motor4(4); // Motor 4 on the Adafruit Motor Shield

Encoders encoderLeft(A15, A13);	// Create an Encoder object name leftEncoder, using digitalpin 2 & 3
Encoders encoderRight(A14 , A12); // Encoder object name rightEncoder using analog pin A0 and A1 

Move move(motor1, motor2, motor3, motor4, encoderLeft, encoderRight);

int pwm1 = 200;
int pwm2 = 200;
int pwm3 = 200;
int pwm4 = 200;

//define objects
MPU6050 sensor;
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

//---------------------------------
unsigned long lastTime = 0;
int step = 1;
static int routine = 4;
  extern int startLeft;
  extern int startRight;

// functions--------------------------------------------------

void onSwitchPress() {
  switchPressed = true;
  switchPressed1 = true;
  lastPressTime = millis();
  resetGyroAngles();

}
bool straight;
bool first = true;
void resetGyroAngles() {
  ang_x = ang_y = ang_z = 0;
  ang_x_prev = ang_y_prev = ang_z_prev = 0;
  tiempo_prev = millis(); 
  delay(300);
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
  myservo.attach(9); 
  myservo.write(180);

  //mpu
  tiempo_prev = millis();
  resetGyroAngles();

  //movement
  pinMode(switchPin, INPUT_PULLUP);
  pinMode(switchPin1, INPUT_PULLUP);

}

void loop() {
  move.begin(pwm1,pwm2,pwm3,pwm4);

  //VAriables para los casos -----------------------------------------------------------
  bool stopped = false;
  long startLeft = move.getStartLeft();
  long startRight = move.getStartRight();
  static int currentPulses;
  static bool repeat = false;
  static int state = 0;
  static int beta = 8; //degree error
  static int theta = 10;
  static int alpha = 0;
  static int mili = 500; //delay
  int diameter = 60; //Diameter of the wheel in mm
  const long ppr = 1200; // Number of pulses for each movement ste
  int closed = 180;
  int open = 0;
  int resetRoutine = 7;
  int largo = 700;
  int rotationTime = 400;

  //activate rotor
  digitalWrite(enable34, HIGH);
  digitalWrite(input3, LOW);
  digitalWrite(input4, HIGH);

  // Handle switch press
  bool currentSwitchState = digitalRead(switchPin);
  bool currentSwitchState1 = digitalRead(switchPin1);
  if (currentSwitchState == LOW && lastSwitchState == HIGH) {

    // Button pressed
    onSwitchPress();
    state++;
    Serial.println("Gyro angles reset by microswitch!");
  }
  lastSwitchState = currentSwitchState;
  lastSwitchState1 = currentSwitchState1;

  //   //MPU code
  // Leer las aceleraciones y velocidades angulares-----------------------------------
  // int ang_z = filterGyro(sensor);
  if (sensor.testConnection()){
    sensor.getRotation(&gx, &gy, &gz);

    dt = (millis() - tiempo_prev) / 1000.0;
    tiempo_prev = millis();

    ang_z = ang_z_prev + (gz / 131.0) * dt; // Z-axis rotation using gyroscope only

    ang_z_prev = ang_z;
  }



  static int routine = 4;
  switch (routine) {// Routines --------------
    case 0: // Routine 0 -------------------------------------------------------------------
      myservo.write(open);
      switch (state) {
        case 0:
          if (move.accelerateForward(currentPulses = move.mmToPulses(400.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
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
          if (move.accelerateForward(move.mmToPulses(550.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
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
        if (move.accelerateBackward(currentPulses = move.mmToPulses(50.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state = 1;
        break; 
      case 1:
        if (move.stopForMillis(mili)) state = 2;
        stopped = true;
        break;
      case 2:
      if (first == true){
        pwm1 = 230;
        pwm2 = 220;
        pwm3 = 220;
        pwm4 = 230;}
      else if (first == false){
        pwm1 = 220;
        pwm2 = 240;
        pwm3 = 220;
        pwm4 = 240;
        }
        if (move.accelerateForward(currentPulses = move.mmToPulses(largo, 60, ppr), 210, 230, 20, 40, 0, 1)) state = 3;
        break;
      case 3:
        if (move.stopForMillis(mili)) state = 4;
        stopped = true;
        break;
      case 4:
        pwm1 = 230;
        pwm2 = 220;
        pwm3 = 220;
        pwm4 = 230;
        if (move.accelerateBackward(move.mmToPulses(largo + 50.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state = 5;
        straight = true;
        break;
      case 5:
        if (move.stopForMillis(mili)) state = 6;
        stopped = true;
        break;
      case 6:
        if (move.left(move.mmToPulses(320, diameter, ppr))) state = 7;
        straight = false;
        break;
      case 7:
        if (move.stopForMillis(mili)) state = 8;
        break;
      case 8:
        if (move.accelerateBackward(move.mmToPulses(120.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state=9;
        break;
      case 9:
        if (move.stopForMillis(mili))state++;
        break;
      case 10:
      if (first == true){
        first = false; 
        routine = 5;}
      else if (first == false){
        routine = 8; // Move to next routine
      }
        state = 0; // Reset state for the next routine
        break;
      }
      break;
    case 5: // Right Sweeps----------------------------------------------------
      switch (state) {
        case 0:
            pwm1 = 225;
            pwm2 = 225;
            pwm3 = 225;
            pwm4 = 225;   
          straight = false;
          if (move.right(move.mmToPulses(320.0, 60, ppr))) state=1;
          break;
        case 1:
          straight = false;
          if (move.accelerateBackward(move.mmToPulses(300.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state=2;
          break;
        case 2:
          if (move.stopForMillis(mili)) state=3;
          break;
        case 3:
        pwm1 = 230;
        pwm2 = 220;
        pwm3 = 220;
        pwm4 = 230;
          if (move.accelerateForward(move.mmToPulses(largo / 3 - 120.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state=4;
          break;
        case 4:
          pwm1 = 230;
          pwm2 = 0;
          pwm3 = 230;
          pwm4 = 0;
          // if (move.stopForMillis(mili)) state++;
          move.stop();
          delay(mili); 
          state=5;
          stopped = true;
          break;
        case 5:
          if (move.accelerateForward(move.mmToPulses(largo / 3 + largo / 3 + 140, 60, ppr), 210, 230, 20, 40, 0, 1)) state=6;
          break;
        case 6:
          if (move.stopForMillis(2*(mili))) state++;
          break;
        case 7:
          straight = true;
        pwm1 = 230;
        pwm2 = 220;
        pwm3 = 220;
        pwm4 = 230;
          if (move.accelerateBackward(move.mmToPulses(largo + 20.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state=8;
          break;
        case 8:
          if (move.stopForMillis(mili)) state=9;
          break;
        case 9:
          // if (repeat == false) {
              routine = resetRoutine; // Move to next routine
              state = 0; // Reset state for the next routine
              // repeat = true;
          // } else {
          //     state = -1;
          //     repeat = false;
          // }
        }
      break;
      case 6: // left reset -------------------------------------------------------------
        switch(state){
          case 0:
              if (move.accelerateBackward(move.mmToPulses(250.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state=1;
              break;
          case 1:
              if (move.stopForMillis(mili)) state=2;
              break;
          case 2:
              if (move.forward(move.mmToPulses(50, diameter, ppr))) state=3;
              break;
          case 3: // Pause after forward
              if (move.stopForMillis(mili)) state=4;
              break;
          case 4:
              digitalWrite(enable34, LOW);
              alpha = -70;
              if (ang_z >= alpha + beta) {
                  move.rotateCCW(pwm1, pwm2, pwm3, pwm4);
              } else if (ang_z <= alpha - beta){
                  move.rotateCW(pwm1, pwm2, pwm3, pwm4);
              } else if (ang_z > alpha - beta || ang_z < alpha + beta) state=5;
              break;
          case 5: // Pause after rotate
              if (move.stopForMillis(mili)) state=6;
              stopped = true;
              break;
          case 6:
            pwm1 = 220;
            pwm2 = 220;
            pwm3 = 220;
            pwm4 = 220;
            if (move.left(move.mmToPulses(120, diameter, ppr))) state=7;
            break;
          case 7: // Pause after left
            pwm1 = 220;
            pwm2 = 200;
            pwm3 = 200;
            pwm4 = 220;
            if (move.stopForMillis(mili)) state=8;
            break;
          case 8:
            if (move.forward(move.mmToPulses(580, diameter, ppr))) state=9;
            break;
          case 9: // Pause after forward
            pwm1 = 50;
            pwm2 = 230;
            pwm3 = 50;
            pwm4 = 230;
            digitalWrite(enable34, HIGH);
            if (move.stopForMillis(mili/4)) state=10;
            break;
          case 10:
              pwm1 = 215;
              pwm2 = 230;
              pwm3 = 230;
              pwm4 = 215;
              digitalWrite(enable34, LOW);
              if (move.backward(move.mmToPulses(20, diameter, ppr))) state=11;
              break;
          case 11: // Pause after backward
              if (move.stopForMillis(mili)) state=12;
              break;
          case 12:
              alpha = 0;
              if (ang_z >= alpha + beta) {
                  move.rotateCCW(pwm1, pwm2, pwm3, pwm4);
              } else if (ang_z <= alpha - beta){
                  move.rotateCW(pwm1, pwm2, pwm3, pwm4);
              } else if (ang_z > alpha - beta || ang_z < alpha + beta) state=13;
              break;
          case 13: // Pause after rotate
              digitalWrite(enable34, HIGH);
              if (move.stopForMillis(mili)) state=14;
              break;
          case 14:
              //add microswitch activation
              if (move.accelerateLeft(move.mmToPulses(150.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state=15;
              break;
          case 15: // Pause after accelerateLeft
              if (move.stopForMillis(mili)) state=16;
              break;
          case 16:
              if (move.backward(move.mmToPulses(150, diameter, ppr))) state=17;
              break;
          case 17: // Pause after accelerateRight
              if (move.stopForMillis(mili)) state=18;
              break;
          case 18:
              routine = 4; // Move to next routine
              state = 0; // Reset state for the next routine
              break;
        }
        break;
        // case 7: // Routine // Right reset -----------------------------------
          //   switch(state){
          //     case 0:
          //       if (move.accelerateBackward(move.mmToPulses(150.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
          //       break;
          //     case 1:
          //       if (move.stopForMillis(mili)) state++;
          //       break;
          //     case 2:
          //       if (move.accelerateRight(move.mmToPulses(350.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
          //       break;
          //     case 3:
          //       if (move.stopForMillis(mili)) state++;
          //       break;
          //     case 4:
          //       if(move.accelerateLeft(move.mmToPulses(220, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
          //       break;
          //     case 5:
          //       routine = 4; // Move to next routine
          //       state = 0; // Reset state for the next routine
          //       break;
          //   }
        case 8: // Routine 4 // Left Sweeps -------------------------------------------------------------------
          switch (state) {
          case 0:
            pwm1 = 200;
            pwm2 = 200;
            pwm3 = 200;
            pwm4 = 200;
            if (move.right(move.mmToPulses(180, diameter, ppr))) state=1;
            break;
          case 1:
        pwm1 = 230;
        pwm2 = 220;
        pwm3 = 220;
        pwm4 = 230;
            if (move.accelerateBackward(currentPulses = move.mmToPulses(100.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state=2;
            break; 
          case 2:
            if (move.stopForMillis(mili)) state=3;
            stopped = true;
            break;
          case 3:
            straight = true;
            if (move.accelerateForward(currentPulses = move.mmToPulses(largo, 60, ppr), 210, 230, 20, 40, 0, 1)) state=4;
            break;
          case 4:
            if (move.stopForMillis(mili)) state=5;
            stopped = true;
            break;
          case 5:
            if (move.accelerateBackward(move.mmToPulses(largo + 50.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state=6;
            straight = true;
            break;
          case 6:
            if (move.stopForMillis(mili)) state=7;
            stopped = true;
            break;
          case 7:
            if (move.left(move.mmToPulses(650, diameter, ppr))) state=8;
            straight = false;
            break;
          case 8:
            if (move.stopForMillis(mili)) state=9;
            break;
          case 9:
            if (move.accelerateBackward(move.mmToPulses(100.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state=10;
            break;
          case 10:
            routine = 5; // Move to next routine
            state = 0; // Reset state for the next routine
            break;
          }
        break;
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