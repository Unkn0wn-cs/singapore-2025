#include <Arduino.h>
#include <AFMotor.h>
#include <QuadratureEncoder.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "C:\Users\samue\OneDrive\Documents\GitHub\singapore25\Codes\Movement\Encoder\Ecoder measured movement\lib\move\move.h"
#include <Pixy2.h>

//move
AF_DCMotor motor1(1); // Motor 1 on the Adafruit Motor Shield
AF_DCMotor motor2(2); // Motor 2 on the Adafruit Motor Shield
AF_DCMotor motor3(3); // Motor 3 on the Adafruit Motor Shield
AF_DCMotor motor4(4); // Motor 4 on the Adafruit Motor Shield

  int pwmf[4] = {220, 230, 230, 220};
  int pwms[4] = {200, 200, 200, 200};
  // int pwmf[4] = {230, 220, 220, 230};
  // int pwms[4] = {200, 200, 200, 200};
  

Encoders encoderLeft(A15, A14);	// Create an Encoder object name leftEncoder, using digitalpin 2 & 3
Encoders encoderRight(A13 , A12); // Encoder object name rightEncoder using analog pin A0 and A1 
Move move(
  motor1, motor2, motor3, motor4, encoderLeft, encoderRight,
  pwmf[0], pwmf[1], pwmf[2], pwmf[3],      // Forward/backward PWM values
  pwms[0], pwms[1], pwms[2], pwms[3]       // Left/right/diagonal PWM values
);


//define objects----------------------------------------
MPU6050 sensor;
Pixy2 pixy;
Servo myservo;

//Pixy Cam---------------------------------------------
int purpleSignature = 1;
int orangeSignature = 2;
int redSignature = 3;

// --- Microswitch Gyro Reset Extension ----------------------------------------------------------------

const byte backSwitchPin = 18; // For interrupt on Mega
const byte sideSwitchPin = 19; // For interrupt on Mega

volatile bool backSwitchPressed = false;
volatile bool sideSwitchPressed = false;
bool lastBackSwitchState = HIGH;    // for edge detection
bool lastSideSwitchState = HIGH;    // for edge detection

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
//

//---------------------------------
static int routine = 4;
int state = 0;
bool first = true;

int closed = 180;
int open = 0;

int beta = 8; //degree error
int alpha = 0;
  
int lenght = 1100;

const int mili = 2000; //delay
const int diameter = 60; //Diameter of the wheel in mm

enum rlane {
  OUTER, 
  MIDDLE,
  INNER
}; rlane lane = MIDDLE;

enum side {
  RIGHT,
  LEFT
}; 

//conditional----------------------------------------------------------------------------------------------- aqui Samuel 
const long pulses = 1020; // Number of pulses for each movement step
side robotSide = LEFT;



// functions-----------------------------------------------------------------------------

int mm(int mm){
  return(move.mmToPulses(mm, diameter, pulses));
}

bool outerForward(int mili){
  if (robotSide == RIGHT){
    move.begin(pwmf[1] + 20, pwmf[2], pwmf[3], pwmf[4] + 20);  
  }else if (robotSide == LEFT){
    move.begin(pwmf[1], pwmf[2] + 80, pwmf[3] + 80, pwmf[4]);  
  }
  if(move.forwardp(mm(mili))) return true;
}

bool inner(int mili){
  if (robotSide == RIGHT){
    if(move.left(mm(mili))) return true;
  }else if (robotSide == LEFT){
    if(move.right(mm(mili)))return true;
  }
}

bool outer(int mili){
  if (robotSide == RIGHT){
    if(move.right(mm(mili)))return true;
  }else if (robotSide == LEFT){
    if(move.left(mm(mili))) return true;
  }
}

bool innerDiagonal(int mili) {
  if (robotSide == RIGHT){
    if(move.forwardLeft(mm(mili))) return true;
  }else if (robotSide == LEFT){
    if(move.forwardRight(mm(mili))) return true;
  }
}

void enableDrivers(){digitalWrite(enable34, HIGH);}
void disableDrivers(){digitalWrite(enable34, LOW);}

//----------------------------------------------------------------------------------------
int filterGyro(MPU6050_Base sensor){
  sensor.getRotation(&gx, &gy, &gz);

  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();

  ang_z = ang_z_prev + (gz / 131.0) * dt; // Z-axis rotation using gyroscope only

  ang_z_prev = ang_z;
  return ang_z;
} 

void onSwitchPress() {
  backSwitchPressed = true;
  resetGyroAngles();
}

void resetGyroAngles() {
  ang_x = ang_y = ang_z = 0;
  ang_x_prev = ang_y_prev = ang_z_prev = 0;
  tiempo_prev = millis(); 
}

void setup() { //---------------------------------------------------------------------------------------------

  Serial.begin(9600);
  //servo--------------------------------------------
  myservo.attach(9); 
  
  //rotor
  pinMode(enable34, OUTPUT);
  pinMode(input3, OUTPUT);
  pinMode(input4, OUTPUT);

    // LED DEBUGER SUPERIOR GRAN RESERVA PRO MAX ROJO TRUMP MAGA UNIMET #FORMAFALICA
    pinMode(34, OUTPUT);


  digitalWrite(enable34, HIGH);
  digitalWrite(input3, LOW);
  digitalWrite(input4, HIGH);

  // //mpu 
  // Wire.begin();           // Iniciando I2C
  // sensor.initialize();    // Iniciando el sensor

  // if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  // else Serial.println("Error al iniciar el sensor");

  tiempo_prev = millis();
  resetGyroAngles();

  //microSwitch
  pinMode(backSwitchPin, INPUT_PULLUP);
  pinMode(sideSwitchPin, INPUT_PULLUP);

  // pixy.init();
  // pixy.setLamp(255, 0);
  //   //Calculate purple position
  // int center_y = 55;
  // int center_x = 200;

  // for (int i = 0; i < 10; i++){
  //   delay(5);
  // pixy.ccc.getBlocks();
  // if (pixy.ccc.numBlocks > 0) {
  //   for (int i = 0; i < pixy.ccc.numBlocks; i++) {
  //     if (pixy.ccc.blocks[i].m_signature == purpleSignature) {

  //       int ix = pixy.ccc.blocks[i].m_x; 
  //       int iy = pixy.ccc.blocks[i].m_y;
  //       //definir routine 
  //       if (ix < center_x && iy < center_y){
  //         routine = 1;
  //         Serial.print("upper left corner, case 1");
  //       } else if (ix > center_x && iy < center_y){
  //         routine = 3;
  //         Serial.print("upper right corner, case 3");
  //       } else if (ix < center_x && iy > center_y){
  //         routine = 0;
  //         Serial.print("lower left corner, case 0");
  //       } else if (ix > center_x && iy > center_y){
  //         routine = 2;
  //         Serial.print("lower Right corner, case 2");
  //       }
  // }} }}
  // pixy.m_link.close();
  





}

void loop() {
  //notas -----------------------------------------------------------
  // int diameter = ::diameter, int pulses = ::ppr


  // Handle switch press
  bool currentBackSwitchState = digitalRead(backSwitchPin);
  if (currentBackSwitchState == LOW && lastBackSwitchState == HIGH) {

    // Button pressed
    onSwitchPress();
    state++;
    Serial.println("Gyro angles reset by microswitch!");
  }
  lastBackSwitchState = currentBackSwitchState;



  //MPU code
  // Leer las aceleraciones y velocidades angulares-----------------------------------
  // int ang_z = filterGyro(sensor);
  // sensor.getRotation(&gx, &gy, &gz);

  // dt = (millis() - tiempo_prev) / 1000.0;
  // tiempo_prev = millis();

  // ang_z = ang_z_prev + (gz / 131.0) * dt; // Z-axis rotation using gyroscope only

  // ang_z_prev = ang_z;

  // Serial.print("\tRotacion en Z: ");
  // Serial.println(ang_z);


  //check if the robot is centered before acting---------------------------------
  if (ang_z >= alpha + beta) {
    routine = 8;
  } else if (ang_z <= alpha - beta){
    routine = 8;
  }

    // routine 0-3 = case 0-3 ball 
    // routine 4 Simple Lanes
    // routine 5 Diagonal Lane
    // routine 6 simple reset
    // routine 7 Corner checking reset
    // routine 8 Reorient itself

switch (routine) {
  case 4:
    switch(state){
      case -1:
        if(inner(180)) state++;
        break;
      case 0:
        if(move.backward(mm(180))) state++;
        break;
      case 1:
        if(move.stopForMillis(mili)) state++;
        break;
      case 2:
        digitalWrite(enable34, HIGH);
        if (lane == MIDDLE){
          if(move.forward(mm(lenght))) state++;
        } else if(lane == OUTER){
          if(outerForward(lenght)) state++;
        }
        break;
      case 3:
        if(move.stopForMillis(2*mili)) state++;
        break;
      case 4:
        if (robotSide == RIGHT || first == false){
          if (lane == OUTER){
            lane = MIDDLE;
          }else if(lane == MIDDLE){
            lane = INNER;
          }
        }else if (robotSide == LEFT){
          first = false;
          if (lane = OUTER){
            lane = MIDDLE;
          } else if (lane = MIDDLE) {
            lane = INNER;
          } else if (lane = INNER) {
            return(1);
          }
        }
        routine = 6;
        state = 0;      
      break;
    }
  break;
  case 5:
    switch(state){
      case 0:
        if(inner(320)) state++;
        break;
      case 1:
        if(move.stopForMillis(mili)) state++;
        break;
      case 2:
        if(move.backward(300)) state++;
        break;
      case 3:
        if(move.stopForMillis(mili)) state++;
        break;
      case 4:
        if(move.forward(mm(lenght / 3))) state++;
        break;
      case 5:
        if(move.stopForMillis(mili)) state++;
        break;
      case 6:
        if(innerDiagonal((200))) state++;
        digitalWrite(34, HIGH); 
        break;
      case 7:
        if (move.stopForMillis(mili)) state++;
        digitalWrite(34, LOW);
        break;
      case 8:
        if(innerDiagonal((200))) state++;
        digitalWrite(34, HIGH); 
        break;
      case 9:
        if(move.stopForMillis(2*mili)) state++;
        digitalWrite(34, LOW);
        break;
      case 10:
        if(outer(50)) state++;
        break;
      case 11:
        if(move.stopForMillis(mili)) state++;
        break;
      case 12:
        lane = OUTER;
        routine = 7;
        state = 0;
        break;
    }
    break;
  case 6:
    switch(state){
      case 0:
        if(move.backward(mm(lenght + 20))) state++;
        break;
      case 1:
        if(move.stopForMillis(mili*3)) state++;
        break;
      case 2:
        if(outer(350)) state++;
        break;
      case 3:
        if(move.stopForMillis(mili*3)) state++;
        break;
      case 4:
        if(lane == INNER){
          routine = 5;
          state = 0;
        }else if(lane == MIDDLE){
          routine = 4;
          state = -1;
        }else if(lane == OUTER){
          routine = 4;
          state = 0;
        }
    }
  break;
  case 7:
    float time;
    switch(state){
      case 0:
        if(move.backward(mm(lenght + 20))) state++;
        break;
      case 1:
        if(move.stopForMillis(mili)) state++;
        break;
      case 2:
        if(move.forward(50)) state++;
        break;
      case 3:
        digitalWrite(enable34, LOW);
        if(move.stopForMillis(mili)) state++;
        break;
      case 4:
        static bool rotation = false;
        if (rotation == false){
          time = millis();
          rotation = true;
        } if (time + 1200 < millis()) state++;

        if (robotSide = LEFT){
          alpha = -70;
        } else {alpha  = 70;}
        
        if (ang_z >= alpha + beta) {
            move.rotateCCW(200, 200, 200, 200);
        } else if (ang_z <= alpha - beta){
            move.rotateCW(200, 200, 200, 200);
        } else if (ang_z > alpha - beta || ang_z < alpha + beta) state++;
      case 5:
        if(move.stopForMillis(mili)) state++;
        break;
      case 6:
        if(outer(100)) state++;
        break;
      case 7:
        if(move.stopForMillis(mili/2)) state++;
        break;
      case 8:
        if(move.forward(580)) state++;
        break;
      case 9:
        digitalWrite(enable34, HIGH);
        if(move.stopForMillis(250)) state++;
        break;
      case 10:
        digitalWrite(enable34, LOW);
        if(move.backward(20)) state++;
        break;
      case 11:
        if(move.stopForMillis(mili/2)) state++;
        rotation = false;
        break;
      case 12:
        if (rotation == false){
          time = millis();
          rotation = true;
        } if (time + 1200 < millis()) state++;

        alpha = 0;
        if (ang_z >= alpha + beta) {
            move.rotateCCW(200, 200, 200, 200);
        } else if (ang_z <= alpha - beta){
            move.rotateCW(200, 200, 200, 200);
        } else if (ang_z > alpha - beta || ang_z < alpha + beta) state++;
      case 13:
        if(move.stopForMillis(mili)) state++;
        break;
      case 14:
        if(outer(120)) state++;
        break;
      case 15:
        if(move.stopForMillis(mili)) state++;
        break;
      case 16:
        if(move.backward(200)) state++;
        break;
      case 17:
        if(move.stopForMillis(mili)) state++;
        break;  
      case 18:
        routine = 4;
        state = 0;
        lane = OUTER;
    }
  break;
  // case 8:
} 
  

  delay(5); //delay for stability introduced july 
}