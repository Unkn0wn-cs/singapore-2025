
#include <QuadratureEncoder.h>
// must also have enableInterrupt.h library

// Use any 2 pins for interrupt, this utilizes EnableInterrupt Library. 
// Even analog pins can be used. A0 = 14,A1=15,..etc for arduino nano/uno

// Max number of Encoders object you can create is 4. This example only uses 2.

Encoders leftEncoder(A14, A12);	// Create an Encoder object name leftEncoder, using digitalpin 2 & 3
Encoders rightEncoder(A13 , A15); // Encoder object name rightEncoder using analog pin A0 and A1 

void setup() {
  Serial.begin(9600);
}


unsigned long lastMilli = 0;

void loop() {
  // put your main code here, to run repeatedly:
  // print encoder count every 50 millisecond
  if(millis()-lastMilli > 50){ 
    
    long currentLeftEncoderCount = leftEncoder.getEncoderCount();
    long currentRightEncoderCount = rightEncoder.getEncoderCount();
    
    Serial.print(currentLeftEncoderCount);
    Serial.print(" , ");
    Serial.println(currentRightEncoderCount);
    
    lastMilli = millis();
  }
  //  delay(1000);
}