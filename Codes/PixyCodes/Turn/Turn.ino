#include <Servo.h>

//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//
// This sketch is a good place to start if you're just getting started with 
// Pixy and Arduino.  This program simply prints the detected object blocks 
// (including color codes) through the serial console.  It uses the Arduino's 
// ICSP SPI port.  For more information go here:
//
// https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:hooking_up_pixy_to_a_microcontroller_-28like_an_arduino-29
//
  
#include <Pixy2.h>

// This is the main Pixy object 
Pixy2 pixy;
Servo myservo;

void setup()
{
  Serial.begin(115200);
  Serial.print("Starting...\n");
  myservo.attach(3);
  
  pixy.init();
}

void loop()
{ 
  int i; 
  // grab blocks!
  pixy.ccc.getBlocks();
  // If there are detect blocks, print them!
  if (pixy.ccc.numBlocks){
      if (pixy.ccc.blocks[i].m_x < 140)
  {
    Serial.print("MOVE RIGHT\n");
    pixy.ccc.blocks[i].print();
    delay (1000);

  } else if (pixy.ccc.blocks[i].m_x > 240){
    Serial.print("MOVE LEFT\n");
        pixy.ccc.blocks[i].print();

    delay (1000);
  } else {
            pixy.ccc.blocks[i].print();

  }
  delay(1000);
  }
}

