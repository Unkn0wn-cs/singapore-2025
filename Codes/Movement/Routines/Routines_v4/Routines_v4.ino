    int LED = 34;

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
  //LEFT
  int pwmf[4] = {220, 232, 232, 220};
  int pwms[4] = {200, 210, 200, 200};
  //RIGHT
  // int pwmf[4] = {233, 238, 238, 233};
  // int pwms[4] = {215, 200, 200, 200};
  

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
int purpleSignature = 2;
int orangeSignature = 1;
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
const int input4   = 25; // L293D pin 10
const int input3   = 27; // L293D pin 15

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

const int mili = 750; //delay
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

//conditional------------------------------------------------------------------------------------- aqui Samuel 
// const long pulses = 1650; // Number of pulses for each movement step
const long pulses = 1020; // Number of pulses for each movement step
side robotSide = LEFT;
int lenght = 0;

//servo--------------------------------------------
int closedGate = 170;
int openGate = 55;
//RIGHT
// int closedGate = 90;
// int openGate = 0;

// functions-----------------------------------------------------------------------------

int mm(int mm){
  return(move.mmToPulses(mm, diameter, pulses));
}

void outerForward(){
  if (robotSide == RIGHT){
    move.begin(pwmf[1] + 20, pwmf[2], pwmf[3], pwmf[4] + 20);  
  }else if (robotSide == LEFT){
    move.begin(pwmf[1], pwmf[2] + 80, pwmf[3] + 80, pwmf[4]);  
  }
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

  void blink(){
        digitalWrite(LED, HIGH);
        delay(500);
        digitalWrite(LED, LOW);
        delay(500);
        return;
  }

void setup() { //-----------------------------------------------------------------------------------------------------------------------------------///

  Serial.begin(9600);
  if(robotSide == RIGHT){
    lenght = 720;
  }else{
    lenght = 1100;
  } 
  //servo--------------------------------------------
  myservo.attach(9); 
  myservo.write(closedGate);
  
  //rotor
  pinMode(enable34, OUTPUT);
  pinMode(input3, OUTPUT);
  pinMode(input4, OUTPUT);

    // LED DEBUGER SUPERIOR GRAN RESERVA PRO MAX ROJO TRUMP MAGA UNIMET #FORMAFALICA
    pinMode(LED, OUTPUT);


  digitalWrite(enable34, 50);
  digitalWrite(input3, HIGH);
  digitalWrite(input4, LOW);

  // //mpu 
  Wire.begin();           // Iniciando I2C
  sensor.initialize();    // Iniciando el sensor

  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");

  tiempo_prev = millis();
  resetGyroAngles();

  //microSwitch
  pinMode(backSwitchPin, INPUT_PULLUP);
  pinMode(sideSwitchPin, INPUT_PULLUP);


  pixy.init();
  pixy.setLamp(200, 0);
  delay(1000);
    //Calculate purple position
  int center_y = 32;
  int center_x = 200;

void purple();{
  for (int i = 0; i < 60; i++){
    
    pixy.ccc.getBlocks();
    if (pixy.ccc.numBlocks > 0) {
      for (int i = 0; i < pixy.ccc.numBlocks; i++) {
        if (pixy.ccc.blocks[i].m_signature == purpleSignature &&
            pixy.ccc.blocks[i].m_age > 85 &&
            pixy.ccc.blocks[i].m_x > 105 && pixy.ccc.blocks[i].m_x < 300 &&
            pixy.ccc.blocks[i].m_y > 15 && pixy.ccc.blocks[i].m_y < 100) {
              
          int ix = pixy.ccc.blocks[i].m_x; 
          int iy = pixy.ccc.blocks[i].m_y;
          //definir routine 
          if (ix < center_x && iy < center_y){
            routine = 0;
            // blink();
            return;
            Serial.print("upper left corner, case 1");
          } else if (ix > center_x && iy < center_y){
            routine = 1;
            // blink(); blink();
            return;
            Serial.print("upper right corner, case 3");
          } else if (ix < center_x && iy > center_y){
            routine = 2;
            // blink(); blink(); blink();
            return;
            Serial.print("lower left corner, case 0");
          } else if (ix > center_x && iy > center_y){
            routine = 3;
            // blink(); blink(); blink(); blink();
            return;
            Serial.print("lower Right corner, case 2");
          }
        } 
      } 
      } 
      delay(17);
  }
}


  pixy.m_link.close();
    delay(20);
    pixy.setLamp(0, 0);







}

void loop() {//---------------------------------------------------------------------------------------------------------------------------------------------////
  static int microSwitchTime = 0;
  int currentTime = millis();
    bool currentBackSwitchState = digitalRead(backSwitchPin);
    bool currentSideSwitchState = digitalRead(sideSwitchPin);


    if (currentTime - microSwitchTime > 500){
      // Handle switch press
      if (currentBackSwitchState == LOW && lastBackSwitchState == HIGH) {
        microSwitchTime = millis();
        // Button pressed
        onSwitchPress();
        state++;
        Serial.println("Gyro angles reset by microswitch!");
      }
      if (currentSideSwitchState == LOW && lastSideSwitchState == HIGH && routine!= 4) {
        microSwitchTime = millis();
        // Button pressed
        state++;
      }
    }
    lastBackSwitchState = currentBackSwitchState;
    lastSideSwitchState = currentSideSwitchState;

    static bool lastRoutine = false;
    if (lastRoutine == false && millis() > 105000 && routine != 7){
      lastRoutine = true;

      routine = 6;
      state = 0;
      lane = INNER;
       
    }


// MPU code
    const float GYRO_SENSITIVITY = 131.0; // MPU6050 scale factor for ±250°/s

    // Get filtered Z-axis rotation rate
    float gz_filtered = filterGyro(sensor); // Should return raw gyro Z in deg/s

    // Time delta in seconds
    unsigned long now = millis();
    dt = (now - tiempo_prev) / 1000.0;
    tiempo_prev = now;


    // Integrate angular velocity to get angle
    ang_z = ang_z_prev + (gz_filtered / GYRO_SENSITIVITY) * dt;
    ang_z_prev = ang_z;
    

    
  



  //check if the robot is centered before acting---------------------------------
  // if (ang_z >= alpha + beta && routine != 7) {
  //   routine = 8;
  // } else if (ang_z <= alpha - beta && routine != 7){
  //   routine = 8;
  // }

    // routine 0-3 = case 0-3 ball 
    // routine 4 Simple Lanes
    // routine 5 Diagonal Lane
    // routine 6 simple reset
    // routine 7 Corner checking reset
    // routine 8 Reorient itself
    // routine 10 debugging


switch (routine) {
  case 0:
    switch(state){
      case 0:
        if(move.backward(mm(100))) state++;
        break;
      case 1:
        myservo.write(closedGate);
        if(move.stopForMillis(mili)) state++;
        break;
      case 2:
        if(move.forward(mm(550))) state++;
        break;
      case 3:
        // if(move.stopForMillis(mili)) 
        state++;
        break;
      case 4:
        myservo.write(openGate);
        if(move.forward(mm(300))) state++;
        break;
      case 5:
        myservo.write(closedGate);
        if(move.stopForMillis(mili)) state++;
        break;
      case 6:
        routine = 6; state = 0;
        if (robotSide == RIGHT){
          lane = OUTER;
        } else {lane = INNER;}
      }
    break;
  case 1:
    switch(state){
      case 0:
        if (move.right(mm(250))) state++;
        break;
      case 1:
        if(move.backward(mm(100))) state++;
        break;
      case 2:
        myservo.write(closedGate);
        if(move.stopForMillis(mili)) state++;
        break;
      case 3:
        if(move.forward(mm(550))) state++;
        break;
      case 4:
        // if(move.stopForMillis(mili))
        state++;
        break;
      case 5:
        myservo.write(openGate);
        if(move.forward(mm(300))) state++;
        break;
      case 6:
        myservo.write(closedGate);
        if(move.stopForMillis(mili)) state++;
        break;
      case 7:
        routine = 6; state = 0;
        lane = MIDDLE;
      }
    break;
  case 2:
    switch(state){
      case 0:
        if(move.backward(mm(100))) state++;
        break;
      case 1:
        myservo.write(openGate);
        if(move.stopForMillis(mili)) state++;
        break;
      case 2:
        if(move.forward(mm(550))) state++;
        break;
      case 3:
        // if(move.stopForMillis(mili)) 
        state++;
        break;
      case 4:
        myservo.write(closedGate);
        if(move.forward(mm(200))) state++;
        break;
      case 5:
        routine = 6; state = 0;
        if (robotSide == RIGHT){
          lane = OUTER;
        } else {lane = INNER;}
    }
    break;
  case 3:
    switch(state){
      case 0:
        if(move.right(mm(250))) state++;
        break;
      case 1:
        if(move.backward(mm(100))) state++;
        break;
      case 2:
        myservo.write(openGate);
        if(move.stopForMillis(mili)) state++;
        break;
      case 3:
        if(move.forward(mm(550))) state++;
        break;
      case 4:
        // if(move.stopForMillis(mili)) 
        state++;
        break;
      case 5:
        myservo.write(closedGate);
        if(move.forward(mm(200))) state++;
        break;
      case 6:
        routine = 6; state = 0;
        lane = MIDDLE;
    }
    break;
  





  ////-------------------------------LOOOP-----------------------------------------------------------////
  case 4:
    switch(state){
      case -1:
        if(inner(200)) state++;
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
              if (robotSide == RIGHT){
                if(move.forwardp(mm(lenght + 50), true)) state++;  
              }else{
                if(move.forwardp(mm(lenght + 100), false)) state++;  
              }
          }
        break;
      case 3:
        if(move.stopForMillis(mili)) state++;
        break;
      case 4:
        if(move.stopForMillis(mili)) state++;
        break;
      case 5:

        if (lane == OUTER){
          state = -1;
        } else{state = 0;}

        if (robotSide == RIGHT && first == false){
          if (lane == OUTER){
            lane = MIDDLE;
          }else if(lane == MIDDLE){
            lane = INNER;
          }
        } else if (robotSide == RIGHT && first == true){
          lane = OUTER;
          first = false;
        }else if (robotSide == LEFT){
          first = false;
          if (lane == OUTER){
            lane = MIDDLE;
          } else if (lane == MIDDLE) {
            lane = INNER;
          } else if (lane == INNER) {
            return(1);
          }
        }

        // blink();
        // //cases for debugging
        // if (lane == INNER && first  == false){
        //   routine = 10;
        //   state = 1;
        //   return;
        // } else if (lane == MIDDLE){
        //   routine = 10;
        //   state = 2;
        //   return;
        // } else if (lane == OUTER){
        //   routine = 10;
        //   state = 3;
        //   return;
        // }

        routine = 6;
      break;
    }
  break;
  case 5:
    switch(state){
      case 0:
        if(inner(340)) state++;
        break;
      case 1:
        if(move.stopForMillis(mili)) state++;
        break;
      case 2:
        if(move.backward(600)) state++;
        break;
      case 3:
        if(move.stopForMillis(mili)) state++;
        break;
      case 4:
        if(move.forward(mm(250))) state++;
        break;
      case 5:
        state++;
        break;
      case 6:
        if (robotSide == RIGHT){
          if(move.forwardLeft(mm(380))) state++;
        } else{
          if(move.forwardRight(mm(260))) state++;
        }
        digitalWrite(LED, HIGH); 
        break;
      case 7:
        state++;
        break;
      case 8:
        if(move.forward(mm(lenght/2 + 30))){state++;}
        break;
      case 9:
        if(move.stopForMillis(2*mili)) state++;
        digitalWrite(LED, LOW);
        break;
      case 10:
        if (robotSide == LEFT){
          if(outer(50)) state++;
        } else{
          state++;
        }
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
      case -1:
        if (robotSide == RIGHT){
          if(move.backwardp(mm(lenght + 50), true)) state++;  
        }else{
          if(move.backwardp(mm(lenght + 100), false)) state++;  
        }
        break;
      case 0:
        if(move.backward(mm(lenght + 250))) state++;
        break;
      case 1:
        if(move.stopForMillis(mili)) state++;
        break;
      case 2:
        if(outer(550)) state++;
        break;
      case 3:
        if(move.stopForMillis(mili)) state++;
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
   
    switch(state){
      case 0:
        if(move.backward(mm(lenght + 250))) state++;
        break;
      case 1:
        if(move.stopForMillis(mili)) state++;
        break;
      case 2:
        if(move.forward(80)) state++;
        break;
      case 3:
        digitalWrite(enable34, LOW);
        if(move.stopForMillis(mili)) state++;
        break;
      case 4:
        digitalWrite(LED, HIGH);
        // static bool rotation = false;
        // if (rotation == false){
        //   time = millis();
        //   rotation = true;
        // } if (time + 1200 < millis()) state++;

        if (robotSide == LEFT){
          alpha = -80;
        } else {alpha  = 80;}
        
        if (ang_z >= alpha + beta) {
            move.rotateCCW(200, 200, 200, 200);
        } else if (ang_z <= alpha - beta){
            move.rotateCW(200, 200, 200, 200);
        } else {
          state++;
        } 
        break;
      case 5:
        if(move.stopForMillis(mili)) state++;
        digitalWrite(LED, LOW);
        break;
      case 6:
        if(outer(mm(40))) state++;
        break;
      case 7:
        if(move.stopForMillis(mili/2)) state++;
        break;
      case 8:
          if (robotSide == RIGHT){
            if(move.forwardp(mm(550), true)) state++;  
          }else{
            if(move.forwardp(mm(550), false)) state++;  
          }
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
        // rotation = false;
        break;
      case 12:
        digitalWrite(LED, HIGH);
        // if (rotation == false){
        //   time = millis();
        //   rotation = true;
        // } if (time + 1200 < millis()) state++;

        if (robotSide == LEFT){
          alpha = 10;
        } else {alpha  = 10;}


        if (ang_z >= alpha + beta) {
            move.rotateCCW(200, 200, 200, 200);
        } else if (ang_z <= alpha - beta) {
            move.rotateCW(200, 200, 200, 200);
        } else {
            state++;
        }
        break;
      case 13:
        if(move.stopForMillis(mili)) state++;
        digitalWrite(LED, LOW);
        break;
      case 14:
        if(outer(120)) state++;
        break;
      case 15:
        if(move.stopForMillis(mili)) state++;
        break;
      case 16:
        if(move.backward(mm(200))) state++;
        break;
      case 17:
        state++;
        break;  
      case 18:
        routine = 4;
        state = 0;
        lane = OUTER;
    }
  break;
  case 8:
    alpha = 0;
    if (ang_z >= alpha + beta) {
        move.rotateCCW(200, 200, 200, 200);
    } else if (ang_z <= alpha - beta) {
        move.rotateCW(200, 200, 200, 200);
    } else {
        routine = 7;
        state = 0;
    }


  case 10:
    move.stop();
    switch(state){
      case 0:
        routine = 6;
        state = 0;
        break;
      case 1:
        blink();
        state = 0;
        break;
      case 2:
        blink();
        blink();
        state = 0;
        break;
      case 3:
        blink();
        blink();
        blink();
        state = 0;
        break;
      case 4:
        for (int i = 0; i < 4; i++){
          blink();
        } 
        state = 0;
        break;
      case 5:
        for (int i = 0; i < 4; i++){
          blink();
        } 
        state = 0;
        break;
    }
    break;
  
} 



  delay(5); //delay for stability introduced july 😝
}