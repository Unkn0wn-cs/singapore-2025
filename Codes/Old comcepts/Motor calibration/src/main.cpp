#include <QuadratureEncoder.h>
#include <AFMotor.h>
#include <PIDController.h>


PIDController pid[4]; // Create an instance of the PID controller class, called "pid"


//INSERT VALUES HERE
int targetRPM = 250; // Target speed in RPM - Edit: making it not constant as it should vary depending on the possition of the robot
int motorN1 = 1;
int motorN2 = 2;
int motorN3 = 3;
int motorN4 = 4;

Encoders leftFrontEncoder(A12, A13);   // Encoder for left front motor
Encoders rightFrontEncoder(A15, A14);  // Encoder for right front motor
Encoders leftRearEncoder(A8, A9);        // Encoder for left rear motor
Encoders rightRearEncoder(A11, A10);       // Encoder for right rear motor

// Create motor objects
AF_DCMotor motor1(motorN1); // create motor #1, 64KHz pwm
AF_DCMotor motor2(motorN2); // create motor #2, 64KHz pwm
AF_DCMotor motor3(motorN3);
AF_DCMotor motor4(motorN4);


const int pulsesPerRevolution = 1020; // Number of pulses per revolution
const unsigned long interval = 250000; // Interval for speed calculation in microseconds
//This will be the ditance used as target . must change later 💦
const int target = 6000; //roughtly equal to a meter

unsigned long lastTime = 0;
long lastEncoderCount1 = 0;
long lastEncoderCount2 = 0;
long lastEncoderCount3 = 0;
long lastEncoderCount4 = 0;
float pwmValue1 = 140.0; // Initial PWM value for motor 1 (50% duty cycle)
float pwmValue2 = 140.0; // Initial PWM value for motor 2 (50% duty cycle)
float pwmValue3 = 140.0;
float pwmValue4 = 140.0;
float change = 20.0; // Change in PWM value

void setup() {

  for (int i = 0; i<4; i++){
    pid[i].begin();          // initialize the PID instance
    pid[i].setpoint(target);    // The "goal" the PID controller tries to "reach"
    pid[i].tune(1, 0, 0);    // Tune the PID, arguments: kP, kI, kD
    pid[i].limit(0, 255); 
  }


  Serial.begin(9600);
  motor1.setSpeed(pwmValue1); // Start with initial PWM value
  motor1.run(FORWARD); // Set motor direction
  motor2.setSpeed(pwmValue2); // Start with initial PWM value
  motor2.run(FORWARD); // Set motor direction
  motor3.setSpeed(pwmValue3); // Start with initial PWM value
  motor3.run(FORWARD); // Set motor direction
  motor4.setSpeed(pwmValue4); // Start with initial PWM value
  motor4.run(FORWARD); // Set motor direction
}

void loop() {
  unsigned long currentTime = micros();
  if (currentTime - lastTime >= interval) {
    long currentEncoderCount1 = rightRearEncoder.getEncoderCount();
    long currentEncoderCount2 = leftRearEncoder.getEncoderCount();
    long currentEncoderCount3 = leftFrontEncoder.getEncoderCount();
    long currentEncoderCount4 = rightFrontEncoder.getEncoderCount();
    long pulses1 = currentEncoderCount1 - lastEncoderCount1;
    long pulses2 = currentEncoderCount2 - lastEncoderCount2;
    long pulses3 = currentEncoderCount3 - lastEncoderCount3;
    long pulses4 = currentEncoderCount4 - lastEncoderCount4;
    lastEncoderCount1 = currentEncoderCount1;
    lastEncoderCount2 = currentEncoderCount2;
    lastEncoderCount3 = currentEncoderCount3;
    lastEncoderCount4 = currentEncoderCount4;

    float period = currentTime - lastTime;

    // Calculate RPM for motor 1
    // float rpm1 = (pulses1 / (float)pulsesPerRevolution) * (60.0 * 1000000.0 / period);    
    // // Calculate RPM for motor 2
    // float rpm2 = (pulses2 / (float)pulsesPerRevolution) * (60.0 * 1000000.0 / period);  
    // // Calculate RPM for motor 3
    // float rpm3 = (pulses3 / (float)pulsesPerRevolution) * (60.0 * 1000000.0 / period);
    // // Calculate RPM for motor 4
    // float rpm4 = (pulses4 / (float)pulsesPerRevolution) * (60.0 * 1000000.0 / period);  

    // // Print RPM to Serial Monitor
    // Serial.print("Current RPM Motor 1: ");
    // Serial.println(rpm1);
    // Serial.print("Current RPM Motor 2: ");
    // Serial.println(rpm2);
    // Serial.print("Current RPM Motor 3: ");
    // Serial.println(rpm3);
    // Serial.print("Current RPM Motor 4: ");
    // Serial.println(rpm4);


    static int mode = 2;

    switch (mode)
    {
      case 1:
        motor1.run(RELEASE); // Set motor direction
        motor2.run(RELEASE); // Set motor direction
        motor3.run(RELEASE); // Set motor direction
        motor4.run(RELEASE); // Set motor direction


      //     // Adjust motor speed based on RPM for motor 1
      //   if (rpm1 < targetRPM) {
      //     pwmValue1 += change; // Increase PWM value
      //   } else if (rpm1 > targetRPM) {
      //     pwmValue1 -= change; // Decrease PWM value
      //   }
      //   pwmValue1 = constrain(pwmValue1, 0.0, 255.0);
      //   motor1.setSpeed(pwmValue1);

      //   // Adjust motor speed based on RPM for motor 2
      //   if (rpm2 < targetRPM) {
      //     pwmValue2 += change; // Increase PWM value
      //   } else if (rpm2 > targetRPM) {
      //     pwmValue2 -= change; // Decrease PWM value
      //   }
      //   pwmValue2 = constrain(pwmValue2, 0.0, 255.0);
      //   motor2.setSpeed(pwmValue2);

      //   // Adjust motor speed based on RPM for motor 3
      //   if (rpm3 < targetRPM) {
      //     pwmValue3 += change; // Increase PWM value
      //   } else if (rpm3 > targetRPM) {
      //     pwmValue3 -= change; // Decrease PWM value
      //   }
      //   pwmValue3 = constrain(pwmValue3, 0.0, 255.0);
      //   motor3.setSpeed(pwmValue3);

      //   // Adjust motor speed based on RPM for motor 4
      //   if (rpm4 < targetRPM) {
      //     pwmValue4 += change; // Increase PWM value
      //   } else if (rpm4 > targetRPM) {
      //     pwmValue4 -= change; // Decrease PWM value
      //   }
      //   pwmValue4 = constrain(pwmValue4, 0.0, 255.0);
      //   motor4.setSpeed(pwmValue4);
        break;
      
      case 2:
        // Adjust motor speed based on RPM for motor 1
        pwmValue1 = pid[0].compute(currentEncoderCount1);
        pwmValue1 = constrain(pwmValue1, 80.0, 255.0);
        motor1.setSpeed(pwmValue1);

        // Adjust motor speed based on RPM for motor 2
        pwmValue2 = pid[1].compute(currentEncoderCount2);
        pwmValue2 = constrain(pwmValue2, 80.0, 255.0);
        motor2.setSpeed(pwmValue2);

        // Adjust motor speed based on RPM for motor 3
        pwmValue3 = pid[2].compute(currentEncoderCount3);
        pwmValue3 = constrain(pwmValue3, 80.0, 255.0);
        motor3.setSpeed(pwmValue3);

        // Adjust motor speed based on RPM for motor 4
        pwmValue4 = pid[3].compute(currentEncoderCount4);
        pwmValue4 = constrain(pwmValue4, 80.0, 255.0);
        motor4.setSpeed(pwmValue4);
        break;
    }
   
    // if (currentEncoderCount1 >= target){
    //   mode = 1;
    // }

    // // Print PWM values to Serial Monitor
    // Serial.print("PWM Value Motor 1: ");
    // Serial.println((pwmValue1) * 1.0);
    // Serial.print("PWM Value Motor 2: ");
    // Serial.println((pwmValue2) * 1.0);

    lastTime = currentTime;
    if (change > 1){
      change = change - 1;
    } 
  }
}