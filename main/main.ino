#include <Servo.h>
#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <nRF24L01.h> 
#include <RF24.h>

RF24 radio(22, 23);                     // CE, CSN pins     
const byte address[6] = "00001";

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display = Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define gServo 10
#define baseServo1 11
#define df1 5
#define df2 6

#define IRpin A0

#define enA 3 //right back
#define enD 9 //left back

#define pwmA 110

#define inA1 38
#define inA2 39
#define inD1 46
#define inD2 47

#define Ping 4

Servo gripperServo;
Servo baseServo;
Servo disinfectMotor1;
Servo disinfectMotor2;

double Setpoint, Input, Output;
double Kp=3.2, Ki=0, Kd=0.1;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {

  mlx.begin(); 
  displayInit();
  
  baseServo.attach(baseServo1);
  disinfectMotor1.attach(df1);
  disinfectMotor2.attach(df2);
  
  Serial.begin(9600);
  pinMode(enA, OUTPUT);
  pinMode(enD, OUTPUT);
  pinMode(inA1, OUTPUT);
  pinMode(inA2, OUTPUT);
  pinMode(inD1, OUTPUT);
  pinMode(inD2, OUTPUT);
  stopMotors();

  radio.begin();
  setRadioWrite();

  if(!bno.begin())
  {
    displayString("BNO issue", 0, 0);
    while(1);
  }
  bno.setExtCrystalUse(true);
  resetPID();
  
  
  delay(5000);

  //straight(5000);
}



void loop() {

  //if (readPING() > 10) {
  //  forward();
  //  Serial.println("Forward");
    
  //Serial.println(mlx.readObjectTempF());  
  //} else {
  //  stopMotors();
  //}
  //nudgeFollow();
  
  userCommandReceived();

  delay(50);
}

void nudgeFollow() {
  radioSend("robot following obstacle edge");
  displayString("following obstacle", 5000, 1);
  
  while (1) {
    double ir = IRDistance();
    double ping = readPING();
    if (ping < 10) {
      turn(90);
    } else if (ir > 30) {
      int counter = 0;
      stopMotors();
      delay(500);
      for (int i = 0; i < 3; i++) {
        ir = IRDistance();
        if (ir > 30) {
          counter++;
        }
        delay(100);
      } 
      if (counter > 2) {
        //straight(400);
        stopMotors();
        delay(500);
        turn(-70);
        
        straight(1000);
      }
    } else if (ir > 25) {
      
      slightLeft();
    } else if (ir < 11) {
      slightRight();
    } else {
      straight(200);
    }
  }
}

void slightLeft() {
  displayString("adjusting left", 1000, 1);
  stopMotors();
  delay(500);
  turn(-30);
  straight(600);
  stopMotors();
  delay(500);
}

void slightRight() {
  displayString("adjusting right", 1000, 1);
  stopMotors();
  delay(500);
  turn(30);
  straight(600);
  stopMotors();
  delay(500);
}

void stopMotors() {
  motorOut(0, 0);  
}

void motorOut(int right, int left) {
  if (right < 0) {
    rightBack();
  } else {
    rightForward();
  }
  if (left < 0) {
    leftBack();
  } else {
    leftForward();
  }

  analogWrite(enA, abs(right));
  analogWrite(enD, abs(left)); 
  Serial.println(String(right) + " , " + String(left));
}

void forward() {
  motorOut(pwmA, pwmA);
}

void back() {
  motorOut(-pwmA, -pwmA);
}

void rightForward() {
  digitalWrite(inA1, LOW);
  digitalWrite(inA2, HIGH);
}

void leftForward() {
  digitalWrite(inD1, LOW);
  digitalWrite(inD2, HIGH);
}

void rightBack() {
  digitalWrite(inA1, HIGH);
  digitalWrite(inA2, LOW);
}

void leftBack() {
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

void pickUp() {
  displayString("Picking up vial", 0, 0);
  radioSend("robot picking up vial");
  releaseGripper();
  lowerGripper();
  forwardSlow();
  delay(1000);
  stopMotors();
  delay(1000);
  grip();
  delay(1000);
  backSlow();
  delay(1000);
  stopMotors();
  liftGripper();
}

void deposit() {
  displayString("Dropping off vial", 0, 0);
  radioSend("robot dropping off vial");
  lowerGripper();
  forwardSlow();
  delay(1000);
  stopMotors();
  delay(1000);
  releaseGripper();
  backSlow();
  delay(1000);
  stopMotors();
  delay(1000);
  liftGripper();
  grip();
}

void releaseGripper() {
  gripperServo.detach();
  gripperServo.attach(gServo);
  delay(500);
  radioSend("Releasing gripper");
  for (int i = gripperServo.read(); i > 80; i--) {
    gripperServo.write(i);
    delay(40);
    //Serial.println(gripperServo.read());
  }
  delay(500);
}

void grip() {
  radioSend("Gripping gripper");
  for (int i = gripperServo.read(); i < 105; i++) {
    gripperServo.write(i);
    delay(40);
    //Serial.println(gripperServo.read());
  }
  delay(500);
}

void liftGripper() {
  radioSend("Lift gripper");
  for (int i = baseServo.read(); i < 105; i++) {
    baseServo.write(i);
    delay(15);
    //Serial.println(baseServo.read());
  }
  delay(500);
}

void lowerGripper() {
  radioSend("Lower gripper");
  for (int i = baseServo.read(); i > -1; i--) {
    baseServo.write(i);
    delay(15);
    //Serial.println(baseServo.read());
  }
  delay(500);
}


float IRDistance() {
  float volt = analogRead(IRpin) * 0.0048828125;
  float dist = 1.9735 * volt * volt - 15.378 * volt + 35.352;
  displayString(String(dist), 0, 0);
  
  //Serial.println(dist);
  
  return dist;
}

void backSlow() {
  for (int i = 0; i > -69; i--) {
    motorOut(i, i);
    delay(20);
  }

  delay(1000);
  for (int i = -69; i < 0; i++) {
    motorOut(i, i);
    delay(20);
  }
}

void forwardSlow() {
  //for (int i = 0; i < 80; i++) {
  //  motorOut(i, i);
  motorOut(85, 89);
  //  delay(20);
  //}
  delay(250);
  for (int i = 87; i > 0; i--) {
    motorOut(i, i);
    delay(10);
  }
}

void displayString(String str, int dly, boolean clr) {
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(str);
  display.display();

  if (clr) {
    delay(dly);
    display.clearDisplay();
    display.display();
  }
}

void displayInit() {
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  delay(1000);
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.display();
}

void takeTemp() {

  displayString("Taking temp", 0, 0);
  radioSend("take temp");
  
  lowerGripper();
  delay(2000);
  double init_temp = mlx.readObjectTempF();
  displayString("Please place wrist under the gripper arm.", 0, 0);
  radioSend("Waiting for wrist");
  while (mlx.readObjectTempF() < init_temp + 8) {
    //Serial.println(mlx.readObjectTempF());
    delay(100);
  }
  //Serial.println(mlx.readObjectTempF());
  radioSend("Wrist detected, taking temp");
  displayString("Currently Reading Temperature. Please do not move your arm...", 0, 0);
  double total = 0;
  for (int i = 0; i < 100; i++) {
    total += mlx.readObjectTempF();
    delay(50);
  }

  total = total / 100;
  
  
  radioSend(String(total) + " °F.");
  displayString("Your Temperature is \n" + String(mlx.readObjectTempF()) + " °F.", 5000, 1);
  
  liftGripper();
}

/*************Wireless Function*************/
boolean userCommandReceived() {
  //true when wireless module recieves user command
  //update global "userCommand" variable to the state of the user-desired task (3-6)
  //update "start" and "destination" chars with appropriate path colors
  setRadioRead();
  
  if (radio.available()) {
    char cmd[32] = "";
    radio.read(&cmd, sizeof(cmd));
    if (strcmp(cmd, "") != 0) {
      Serial.println("Received: " + String(cmd));
      displayString("Received: " + String(cmd), 0, 0);
    }
    
    //start = colorDetected();
    if(strcmp(cmd,"pickup") == 0) {

      pickUp();
      
      //destination = paths[0];
      //task = 3;
    } else if (strcmp(cmd,"dropoff") == 0) {

      deposit();
      
      //destination = paths[1]; 
      //task = 4;
    } else if (strcmp(cmd,"disinfect") == 0) {
      
      disinfect();
      
      //destination = paths[2];
      //task = 5;
    } else if (strcmp(cmd,"temp") == 0) {

      takeTemp();
      //destination = paths[3];
      //task = 6;
    } else if (strcmp(cmd, "obs") == 0) {

      
      nudgeFollow();
    }
    
    return true;
  } 
  displayString("Waiting for command", 0, 0);
  return false;
}

void radioSend(String str) {
  setRadioWrite();
  int str_len = str.length() + 1;
  char cmd[str_len];
  str.toCharArray(cmd, str_len);
  radio.write(&cmd, sizeof(cmd));
}

void turn(int angle) {
  double starting = getYaw();
  double targetAbs = getAbs(starting, angle);
  Setpoint = 0;
  resetPID();
  unsigned long startingTime = millis();

  double cur = getYaw();
  Input = getRel(cur, targetAbs);

  while ((millis() < startingTime + 10000) && abs(Input) > 10) {

    //displayString("turning: " + String(Input), 0, 0);
    cur = getYaw();
    Input = getRel(cur, targetAbs);
   
    myPID.Compute();
    if (abs(angle) < 45) {
      motorOut(Output * 1.3, -Output * 1.3);
    } else {
      motorOut(Output * 1.05, -Output * 1.05);
    }
    delay(20);
  }

  stopMotors();
  delay(1000);
  
}

void straight(int t) {
  double starting = getYaw();
  double targetAbs = starting;
  Setpoint = 0;
  resetPID();
  unsigned long startingTime = millis();

  double cur = getYaw();
  Input = getRel(cur, targetAbs);

  while (millis() < startingTime + t) {

    if (readPING() < 10) {
      return;
    }
    
    //displayString("straight: " + String(Input), 0, 0);
    cur = getYaw();
    Input = getRel(cur, targetAbs);
   
    myPID.Compute();
    
    motorOut(pwmA + (Output * 0.9), pwmA - (Output * 0.9));
    delay(20);
  }

  stopMotors();
  delay(250);
}

double getYaw() {
  sensors_event_t event; 
  bno.getEvent(&event);
  return event.orientation.x;
}

double getAbs(double a, double rel) {
  double b = a + rel;
  if (b > 180) {
    return b - 360;
  } else if (b < -180) {
    return b + 360;
  } 
  return b;
}

double getRel(double cur, double tar) {
  int c = (int)(10 * (cur + 180)) % 3600;
  int t = (int)(10 * (tar + 180)) % 3600;
  return (-1 * ((5400 + c - t) % 3600 - 1800)) / 10.0;
}

void disinfect(){
  displayString("Disinfecting", 0, 0);
  radioSend("robot disinfecting");
  turn(180);

  delay(1000);
  //three spray action
  for (int i = 0; i < 3; i++) {
    for(int angle = 0; angle < 180; angle += 30){                                  
      disinfectMotor1.write(180-angle);  
      disinfectMotor2.write(angle);   
      delay(15); 
      if (angle<10){
        delay(100);                      
      }
    } 
    delay(1000);
  }
}

void resetPID() {
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-115, 115);
  myPID.SetSampleTime(20);
}

void setRadioWrite() {
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}

void setRadioRead() {
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}
