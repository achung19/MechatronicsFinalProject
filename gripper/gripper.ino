#include <Servo.h>

#define gServo 9
#define baseServo1 10
#define baselineGrip 85

Servo gripperServo;
Servo baseServo;

void setup() {
  Serial.begin(9600);
  gripperServo.attach(gServo);
  baseServo.attach(baseServo1);
  
  
}

void loop() {
  
  
  releaseGripper();
  lowerGripper();
  grip();
  liftGripper();

  delay(5000);
  lowerGripper();
  releaseGripper();
  liftGripper();
  grip();
  
}

void releaseGripper() {
  gripperServo.detach();
  gripperServo.attach(gServo);
  delay(2000);
  for (int i = 82; i < 86; i++) {
    gripperServo.write(i);
    delay(200);
    Serial.println(gripperServo.read());
  }
  delay(2000);
}

void grip() {
  for (int i = 95; i < 100; i++) {
    gripperServo.write(i);
    delay(100);
    Serial.println(gripperServo.read());
  }
  delay(2000);
}

void liftGripper() {
  for (int i = baseServo.read(); i < 91; i++) {
    baseServo.write(i);
    delay(30);
    Serial.println(gripperServo.read());
  }
  delay(2000);
}

void lowerGripper() {
  for (int i = baseServo.read(); i > -1; i--) {
    baseServo.write(i);
    delay(30);
    Serial.println(gripperServo.read());
  }
  delay(2000);
}
