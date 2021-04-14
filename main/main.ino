#define enA 3 //left front
#define enB 5 //left back
#define enC 6 //right front
#define enD 9 //right back
#define inA1 38
#define inA2 39
#define inB1 41
#define inB2 40
#define inC1 45
#define inC2 44
#define inD1 46
#define inD2 47

#define Ping 4

void setup() {
  Serial.begin(9600);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(enC, OUTPUT);
  pinMode(enD, OUTPUT);
  pinMode(inA1, OUTPUT);
  pinMode(inA2, OUTPUT);
  pinMode(inB1, OUTPUT);
  pinMode(inB2, OUTPUT);
  pinMode(inC1, OUTPUT);
  pinMode(inC2, OUTPUT);
  pinMode(inD1, OUTPUT);
  pinMode(inD2, OUTPUT);
  stopMotors();
  
  delay(2000);
}
 
void loop() {
  int throttle = 30;

  if (readPING() > 10) {
    forward(throttle);
    Serial.println("Forward");
    
    
  } else {
    stopMotors();
  }

  delay(100);
  //
  
  //back(throttle);
  //Serial.println("Rev");
  
  //delay(5000);
  //stopMotors();
  
}

void stopMotors() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  analogWrite(enC, 0);
  analogWrite(enD, 0);  
}

void forward(int throttle) {
  setForward();
  int an = map(throttle, 0, 100, 0, 255);
  analogWrite(enA, an);
  analogWrite(enB, an);
  analogWrite(enC, an);
  analogWrite(enD, an); 
}

void back(int throttle) {
  setBack();
  int an = map(throttle, 0, 100, 0, 255);
  analogWrite(enA, an);
  analogWrite(enB, an);
  analogWrite(enC, an);
  analogWrite(enD, an); 
}

void setForward() {
  digitalWrite(inA1, LOW);
  digitalWrite(inA2, HIGH);
  digitalWrite(inB1, LOW);
  digitalWrite(inB2, HIGH);
  digitalWrite(inC1, LOW);
  digitalWrite(inC2, HIGH);
  digitalWrite(inD1, LOW);
  digitalWrite(inD2, HIGH);
}

void setBack() {
  digitalWrite(inA1, HIGH);
  digitalWrite(inA2, LOW);
  digitalWrite(inB1, HIGH);
  digitalWrite(inB2, LOW);
  digitalWrite(inC1, HIGH);
  digitalWrite(inC2, LOW);
  digitalWrite(inD1, HIGH);
  digitalWrite(inD2, LOW);
}

unsigned long measureDistance()
{
  // set pin as output so we can send a pulse
  pinMode(Ping, OUTPUT);
  // set output to LOW
  digitalWrite(Ping, LOW);
  delayMicroseconds(5);

  // now send the 5uS pulse out to activate Ping)))
  digitalWrite(Ping, HIGH);
  delayMicroseconds(5);
  digitalWrite(Ping, LOW);

  // now we need to change the digital pin
  // to input to read the incoming pulse
  pinMode(Ping, INPUT);

  // finally, measure the length of the incoming pulse
  return pulseIn(Ping, HIGH);
}

float readPING() {
  //read the front distance of the robot (PING)
  float a = 343.0 * measureDistance() / 1000000 / 2 * 100;
  return a;
}
