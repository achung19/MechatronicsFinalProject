#include <Servo.h>

#define gServo 10
#define baseServo1 11
#define baselineGrip 85

#define IRpin A0

Servo gripperServo;
Servo baseServo;

void setup() {
  Serial.begin(9600);
  //gripperServo.attach(gServo);
  //baseServo.attach(baseServo1);

}

void loop() {
  //pickUp();
  //delay(5000);
  //deposit();

  Serial.println(IRDistance());
  delay(500);
}

void pickUp() {
  releaseGripper();
  lowerGripper();
  grip();
  liftGripper();
}

void deposit() {
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


float IRDistance() {
  float volt = analogRead(IRpin) * 0.0048828125;
  //return volt;
  return 1.9735 * volt * volt - 15.378 * volt + 35.352;
}
