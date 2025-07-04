#include <Arduino.h>
#include <AFMotor.h>
#include <QuadratureEncoder.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "C:\Users\samue\OneDrive\Documents\GitHub\singapore25\Codes\Movement\Encoder\Ecoder measured movement\lib\move\move.h"
#include <Pixy2.h>

//move -------------------------------------------------------------------------------------
AF_DCMotor motor1(1); // Motor 1 on the Adafruit Motor Shield
AF_DCMotor motor2(2); // Motor 2 on the Adafruit Motor Shield
AF_DCMotor motor3(3); // Motor 3 on the Adafruit Motor Shield
AF_DCMotor motor4(4); // Motor 4 on the Adafruit Motor Shield

Encoders encoderLeft(A15, A13);	// Create an Encoder object name leftEncoder, using digitalpin 2 & 3
Encoders encoderRight(A14 , A12); // Encoder object name rightEncoder using analog pin A0 and A1 
int lpwm = 160, pwm = 200;
int pwm1 = 160, pwm2 = 200, pwm3 = 200, pwm4 = 160;  
Move move(motor1, motor2, motor3, motor4, encoderLeft, encoderRight);

//define objects
MPU6050 sensor;
Pixy2 pixy;
Servo myservo;

// --- Microswitch------------------------------------------------------------------------------
  const byte switchPin = 18; // For interrupt on Mega
  const byte switchPin1 = 19; // For interrupt on Mega

  volatile bool switchPressed = false;
  volatile bool switchPressed1 = false;
  volatile unsigned long lastPressTime = 0;
  bool lastSwitchState = HIGH;    // for edge detection
  bool lastSwitchState1 = HIGH;    // for edge detection

// --- Rotor variables -------------------------------------------------------------------------
  const int enable34 = 23; // L293D pin 9
  const int input3   = 41; // L293D pin 10
  const int input4   = 31; // L293D pin 15

// --- variables for GyroScope -----------------------------------------------------------------
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

// --- Case variables --------------------------------------------------------------------------
  static int routine = 4;
  static int state = 0;
  static bool first = true;
  static int beta = 8; //degree error
  static int alpha = 0; //Target angle
  static int mili = 1000; //delay
  int diameter = 60; //Diameter of the wheel in mm
  int wheelDiameterMM = diameter;
  const long ppr = 800; // Number of pulses for each wheel rotation
  const long pulsesPerRevolution = ppr;
  //servo
  int closed = 180;
  int open = 0;
  //difference
  int resetRoutine = 6;
  int largo = 1100;

//

//functions 
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
    delay(300);
  }
//

void setup() {
  Serial.begin(9600);
  //rotor
    pinMode(enable34, OUTPUT);
    pinMode(input3, OUTPUT);
    pinMode(input4, OUTPUT);

    digitalWrite(enable34, HIGH);
    digitalWrite(input3, LOW);
    digitalWrite(input4, HIGH);
  //servo
    myservo.attach(9); 
    myservo.write(closed);
  //mpu
    Wire.begin();           // Iniciando I2C
    sensor.initialize();    // Iniciando el sensor

    if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
    else Serial.println("Error al iniciar el sensor");

    tiempo_prev = millis();
    resetGyroAngles();
  //microswitch
    pinMode(switchPin, INPUT_PULLUP);
    pinMode(switchPin1, INPUT_PULLUP);
  //

  // pixy.init();
  // pixy.setLamp(255, 0);
  //   //Calculate purple position
  // int center_y = 55;
  // int center_x = 200;

  // for (int i = 0; i < 10; i++){
  //   delay(5);
  // pixy.ccc.getBlocks();
  // if (pixy.ccc.numBlocks > 0) {
  //   int ix = pixy.ccc.blocks[0].m_x; 
  //   int iy = pixy.ccc.blocks[0].m_y;
  //   //definir routine 
  //   if (ix < center_x && iy < center_y){
  //     routine = 1;
  //     Serial.print("upper left corner, case 1");
  //   } else if (ix > center_x && iy < center_y){
  //     routine = 3;
  //     Serial.print("upper right corner, case 3");
  //   } else if (ix < center_x && iy > center_y){
  //     routine = 0;
  //     Serial.print("lower left corner, case 0");
  //   } else if (ix > center_x && iy > center_y){
  //     routine = 2;
  //     Serial.print("lower Right corner, case 2");
  //   }
  // }} 
  // else{routine = 4;}
          // Serial.print("No balls 🤨");
}
//
void loop() {
  move.begin(pwm1,pwm2,pwm3,pwm4);

//micro switch
  bool currentSwitchState = digitalRead(switchPin);
  if (currentSwitchState == LOW && lastSwitchState == HIGH) {

    // Button pressed
    onSwitchPress();
    state++;
    Serial.println("Gyro angles reset by microswitch!");
  }
  lastSwitchState = currentSwitchState;

  bool currentSwitchState1 = digitalRead(switchPin1);
  if (currentSwitchState1 == LOW && lastSwitchState1 == HIGH) {

    Serial.println("Gyro angles reset by microswitch!");
  }
  lastSwitchState1 = currentSwitchState1;
//

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
//

switch(routine){
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

  case 3: // Routine 3 ----------------------------------------------------
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
  
  case 4:
    switch(state){
      case 0:
        if (move.backward(move.mmToPulses(50, wheelDiameterMM, pulsesPerRevolution))) state++;
        break;
      case 1:
        if (move.stopForMillis(mili)) state++;
        break;
      case 2:
        if(move.forward(move.mmToPulses(largo, wheelDiameterMM, pulsesPerRevolution))) state++;
        break;
      case 3:
        if(move.stopForMillis(mili)) state++;
        break;
      case 4:
        if(move.backward(move.mmToPulses(largo + 20.0, wheelDiameterMM, pulsesPerRevolution))) state++;
        break;
      case 5:
        if(move.stopForMillis(mili)) state++;
        break;
      case 6:
        // if (resetRoutine = 6){
        if (move.left(move.mmToPulses(600, diameter, ppr))) state++;
        // } else {
        // if(move.right(move.mmToPulses(450, wheelDiameterMM, pulsesPerRevolution))) state++;
        // }
        break;
      case 7:
        if(move.stopForMillis(mili)) state++;
        break;
      case 8:
        if(move.backward(move.mmToPulses(100, wheelDiameterMM, pulsesPerRevolution))) state++;
        break;
      case 9:
        move.stop();
      break;
    }
    break; 
}





}
