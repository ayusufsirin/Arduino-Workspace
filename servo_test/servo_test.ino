#include <Servo.h>
Servo servoM;

void setup() {
  servoM.attach(9);
  
  int angle = map(180.00, -180, 180, 0, 150);
  servoM.write(angle);
}

void loop() {
   // int(ypr[0] * 180/M_PI) * (-1));
//  delay(15);
}
