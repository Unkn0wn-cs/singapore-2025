// Arduino test code for L293D motor driver
#include <Servo.h>
Servo myservo;  // create Servo object to control a servo

const int enable34 = 23; // L293D pin 9
const int input3   = 41; // L293D pin 10
const int input4   = 31; // L293D pin 15

void setup() {
  pinMode(enable34, OUTPUT);
  pinMode(input3, OUTPUT);
  pinMode(input4, OUTPUT);

  digitalWrite(enable34, HIGH); // Enable motor driver

  myservo.attach(10);  // attaches the servo on pin 9 to the Servo object
  digitalWrite(input3, LOW);
  digitalWrite(input4, HIGH);
}

void loop() {
  // Forward
  myservo.write(95);


  delay(2000);

  myservo.write(0);

  delay(5000);
  // digitalWrite(input3, LOW);
  // digitalWrite(input4, LOW);
  // delay(1000);

  // // Reverse
  // digitalWrite(input3, LOW);
  // digitalWrite(input4, HIGH);
  // delay(2000);

  // // Stop
  // digitalWrite(input3, LOW);
  // digitalWrite(input4, LOW);
  // delay(1000);
}