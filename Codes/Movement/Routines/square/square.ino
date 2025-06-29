#include <Arduino.h>
#include <AFMotor.h>
#include <QuadratureEncoder.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "C:\Users\samue\OneDrive\Documents\GitHub\singapore25\Codes\Movement\Encoder\Ecoder measured movement\lib\move\move.h"

AF_DCMotor motor1(1); // Motor 1 on the Adafruit Motor Shield
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
  const long ppr = 800; // Number of pulses for each movement ste
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

  switch(state){
    case 0:
      if (move.accelerateBackward(currentPulses = move.mmToPulses(100.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
      break;
    case 1:
      if (move.accelerateForward(currentPulses = move.mmToPulses(largo, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
      break;
    case 2:
      if (move.stopForMillis(mili)) state++;
      break;
    case 3:
      move.rotateCW(150, 150, 150, 150);
      if (move.stopForMillis(rotationTime)) state++;
      break;
    case 4:
      if (move.stopForMillis(mili)) state++;
      break;
    case 5:
      if (move.accelerateBackward(currentPulses = move.mmToPulses(70.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
      break;
    case 6:
      if (move.accelerateForward(currentPulses = move.mmToPulses(400, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
      break;
    case 7:
      if (move.stopForMillis(mili)) state++;
      break;
    case 8:
      move.rotateCW(150, 150, 150, 150);
      if (move.stopForMillis(rotationTime)) state++;
      break;
    case 9:
      if (move.stopForMillis(mili)) state++;
      break;
    case 10:
      if (move.accelerateForward(currentPulses = move.mmToPulses(largo, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
      break;
    case 11:
      if (move.stopForMillis(mili)) state++;
      break;
    case 12:
      move.rotateCW(150, 150, 150, 150);
      if (move.stopForMillis(rotationTime)) state++;
      break;
    case 13:
      if (move.accelerateBackward(currentPulses = move.mmToPulses(100.0, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
      break;
    case 14:
      if (move.accelerateForward(currentPulses = move.mmToPulses(300, 60, ppr), 210, 230, 20, 40, 0, 1)) state++;
      break;
    case 15:
      if (move.stopForMillis(mili)) state++;
      break;
    case 16:
      move.rotateCW(150, 150, 150, 150);
      if (move.stopForMillis(rotationTime)) state = 0;
      break;
  }
}