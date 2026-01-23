#include <QuadratureEncoder.h>
// must also have enableInterrupt.h library

// You can use any 2 pins per encoder (digital or analog pins).
// Max number of Encoder objects you can create is 4.

// Example pin assignments (adjust to your wiring!):
Encoders leftFrontEncoder(A12, A13);   // Encoder for left front motor
Encoders rightFrontEncoder(A15, A14);  // Encoder for right front motor
Encoders leftRearEncoder(A8, A9);        // Encoder for left rear motor
Encoders rightRearEncoder(A11, A10);       // Encoder for right rear motor

unsigned long lastMilli = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  // print encoder counts every 50 ms
  if (millis() - lastMilli > 50) {
    long leftFrontCount  = leftFrontEncoder.getEncoderCount();
    long rightFrontCount = rightFrontEncoder.getEncoderCount();
    long leftRearCount   = leftRearEncoder.getEncoderCount();
    long rightRearCount  = rightRearEncoder.getEncoderCount();

    Serial.print("LF: "); Serial.print(leftFrontCount);
    Serial.print(" | RF: "); Serial.print(rightFrontCount);
    Serial.print(" | LR: "); Serial.print(leftRearCount);
    Serial.print(" | RR: "); Serial.println(rightRearCount);

    lastMilli = millis();
  }
}