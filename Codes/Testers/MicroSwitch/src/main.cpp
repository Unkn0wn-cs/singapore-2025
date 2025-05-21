#include <Arduino.h>

const byte switchPin = 18; //for interrupt on Mega

volatile bool switchPressed = false;
volatile unsigned long lastPressTime = 0;

void onSwitchPress() {
  switchPressed = true;
  lastPressTime = millis();
}

void setup() {
  pinMode(switchPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(switchPin), onSwitchPress, FALLING);
  Serial.begin(9600);
}

void loop() {
  if (switchPressed) {
    // Copy volatile variable to local variable for safe use
    unsigned long pressTime;
    noInterrupts();
    pressTime = lastPressTime;
    switchPressed = false;
    interrupts();

    Serial.print("Switch pressed at: ");
    Serial.println(pressTime);
  }

  // delay(10000); // Debounce delay
}
