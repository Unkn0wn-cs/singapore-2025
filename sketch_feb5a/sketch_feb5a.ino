void setup() {
  int value = 0;
  int speed;
  int y = 15;
}

void loop() {
  
  int test(int x) //test the speed of the motor
  {
    //check encoder return speed

    if (speed > x) {value - y;}//if the speed is too high decrease value by y and test()
 
    if (speed < x) {value + y;}//if the speed is too low increse the value by y and test()

    if ((value = 255 && speed < x) || (value < 15 && speed > x){return 1;} //if value 255 and speed to low or value < 15 and speed to high{return motor faulty}

    if (speed = x) {return value;}//if speed = x {return value}

    return 2;
  }
}
