// ...existing code...
#include <Servo.h>
#include <Pixy2.h>

Pixy2 pixy;
Servo myservo;

int servoAngle = 90; // Start centered

void setup()
{
  Serial.begin(115200);
  myservo.attach(9);
  pixy.init();
}

void loop()
{
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks > 0) {
    int x = pixy.ccc.blocks[0].m_x;

    // Dead zone: 140 to 180
    if (x < 140) {
      servoAngle += 2; // turn right
    } else if (x > 180) {
      servoAngle -= 2; // turn left
    }
    // Clamp angle
    if (servoAngle > 180) servoAngle = 180;
    if (servoAngle < 0) servoAngle = 0;

    myservo.write(servoAngle);

    Serial.print("Object X: ");
    Serial.print(x);
    Serial.print(" | Servo Angle: ");
    Serial.println(servoAngle);
  }
  delay(50);
}
// ...existing code...