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
#include <Pixy2.h>
#include <PIDLoop.h>
//#include <SoftwareSerial.h>


//SoftwareSerial relay(12, 13);

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

#define pixyLED 8

Servo gripperServo;
Servo baseServo;
Servo disinfectMotor1;
Servo disinfectMotor2;

double Setpoint, Input, Output;
double Kp=2.8, Ki=0.9, Kd=0.1;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

//Motor speeds, maximum allowed is 255:
#define SPEED_FAST        85 //**adjust!
#define SPEED_SLOW        85 //**adjust!
#define X_CENTER          (pixy.frameWidth/2)

Pixy2 pixy;
PIDLoop headingLoop(3000, 0, 0, false);

int state; 

void setup() {
  Serial.begin(9600);
  //relay.begin(9600);
  Serial1.begin(9600);
  
  mlx.begin(); 
  displayInit();
  
  baseServo.attach(baseServo1);
  disinfectMotor1.attach(df1);
  disinfectMotor2.attach(df2);
  
  pinMode(enA, OUTPUT);
  pinMode(enD, OUTPUT);
  pinMode(inA1, OUTPUT);
  pinMode(inA2, OUTPUT);
  pinMode(inD1, OUTPUT);
  pinMode(inD2, OUTPUT);

  pinMode(pixyLED, OUTPUT);
  digitalWrite(pixyLED, HIGH);
   
  stopMotors();

  if(!bno.begin())
  {
    displayString("BNO issue", 0, 0);
    while(1);
  }
  bno.setExtCrystalUse(true);
  resetPID();

  Serial.println("setting up pixy");
  pixy.init();
  //pixy.changeProg("color_connected_components");
  //delay(3000);
  pixy.changeProg("line");
  pixy.setLED(200, 200, 200);

  state = 0;
  Serial.println("pixy setup complete");

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  
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

  
  //lineFollowingTest();
  
  switch(state) 
  {
    case -1: { //test pixy line following program
      lineFollowingTest();
      break;
    }
    case 0: { //awaiting a command/stop
      //Serial.println(state);
      userCommandReceived();
      break;
    }
  }
  
  
  //Serial.println(colorDetected());
  
  //delay(50);
}

void lineFollowingTest() {
  int8_t res = pixy.line.getMainFeatures();
  if(res <= 0) { //no line detected, stall
    Serial.println("no line detected...switching to stall");
  } else if(res&LINE_INTERSECTION) {
    Serial.println("DETECTED INTERSECTION");
    stopMotors(); //stop and check color
    Serial.print("detected: ");
      Serial.println(colorDetected());
    pixy.line.setNextTurn(-90);
    followLine();
  } else if(res&LINE_VECTOR) { //if line detected, follow the line
    followLine();
  } 
  delay(100);
}

char colorDetected() {
  //returns color currently visible by Pixy
  pixy.changeProg("color_connected_components");
  delay(1000); //delay after program change
  pixy.ccc.getBlocks();
  char color = ' ';

  if (pixy.ccc.numBlocks) {  
    int signature = (int32_t)pixy.ccc.blocks[0].m_signature; //color signature

    switch(signature) 
    {
      case 1: { 
        color = 'g';
        displayString("green", 0, 0); 
        break;} //signature 1: green
      case 2: { 
        color = 'r'; 
        displayString("red", 0, 0);
        break;} //signature 2: red
      case 3: { 
        color = 'b'; 
        displayString("blue", 0, 0);
        break;} //signature 3: blue
      case 4: { 
        color = 'p'; 
        displayString("purple", 0, 0);
        break;} //signature 4: purple
      case 5: { 
        color = 'y'; 
        displayString("yellow", 0, 0);
        break;} //signature 5: yellow
    }
  } else {
    displayString("none found", 0, 0);
  }
  
  pixy.changeProg("line");
  delay(1000); //delay after program change
  return color; 
}

float followLine() { //refer to line_zumo_demo.ino example Pixy2
  int32_t error; 
  int left, right; // left and right motor speeds

  // Calculate heading error with respect to m_x1, which is the far-end of the vector,
  // the part of the vector we're heading toward.
  error = (int32_t)pixy.line.vectors->m_x1 - (int32_t)X_CENTER;

  pixy.line.vectors->print();

  // Perform PID calcs on heading error.
  headingLoop.update(error);

  // separate heading into left and right wheel velocities.
  left = headingLoop.m_command;
  right = -headingLoop.m_command;

  // If vector is heading away from us (arrow pointing up), things are normal.
  if (pixy.line.vectors->m_y0 > pixy.line.vectors->m_y1) 
  {
    // ... but slow down a little if intersection is present, so we don't miss it.
    //if (pixy.line.vectors->m_flags&LINE_FLAG_INTERSECTION_PRESENT) 
    //{
    //  left += SPEED_SLOW;
    //  right += SPEED_SLOW;
    //} 
    //else // otherwise, pedal to the metal!
    //{ 
      left += SPEED_FAST;
      right += SPEED_FAST;
    //}    
  } 
  /*
  else // If the vector is pointing down, or down-ish, we need to go backwards to follow.
  {  
    left -= SPEED_SLOW;
    right -= SPEED_SLOW;  
  } */

  //set right and left motor speeds
  displayString("right:" + String(right) + "\n" + "left:" + String(left), 0, 0);
  //motorOut(right, left);
  delay(25);                       //**adjust!
}


void nudgeFollow(unsigned long t) {
  radioSend("robot following obstacle edge");
  displayString("following obstacle for " + String(t/1000.0) + " s", 2000, 1);
  unsigned long initT = millis();
  
  while (millis() < initT + t) {
    double ir = IRDistance();
    double ping = readPING();
    if (ping < 20) {
      turn(75);
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
        
        straight(1000, 1);
      }
    } else if (ir > 25) {
      
      slightLeft();
    } else if (ir < 11) {
      slightRight();
    } else {
      straight(400, 1);
    }
  }
}

void slightLeft() {
  displayString("adjusting left", 1000, 1);
  stopMotors();
  delay(500);
  turn(-30);
  straight(600, 1);
  stopMotors();
  delay(500);
}

void slightRight() {
  displayString("adjusting right", 1000, 1);
  stopMotors();
  delay(500);
  turn(30);
  straight(600, 1);
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
  //displayString(String(a), 0, 0);
  return a;
}

void pickUp() {
  displayString("Picking up vial", 0, 0);
  radioSend("robot picking up vial");
  releaseGripper();
  lowerGripper(0);
  forwardSlow();
  delay(1000);
  stopMotors();
  delay(1000);
  grip();
  delay(1000);
  liftGripper();
  backSlow();
  delay(1000);
  stopMotors();
  
}

void deposit() {
  displayString("Dropping off vial", 0, 0);
  radioSend("robot dropping off vial");
  
  forwardSlow();
  delay(1000);
  stopMotors();
  delay(1000);
  lowerGripper(0);
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
  radioSend("Releasing gripper");
  displayString(String(gripperServo.read()), 2000, 1);
  if (gripperServo.read() > 80) {
    for (int i = gripperServo.read(); i > 80; i--) {
      gripperServo.write(i);
      delay(25);
      Serial.println(gripperServo.read());
    }  
  } else {
    for (int i = gripperServo.read(); i < 80; i++) {
      gripperServo.write(i);
      delay(25);
      Serial.println(gripperServo.read());
    }
  }
  
  delay(500);
}

void grip() {
  radioSend("Gripping gripper");
  for (int i = gripperServo.read(); i < 105; i++) {
    gripperServo.write(i);
    delay(25);
    //Serial.println(gripperServo.read());
  }
  delay(500);
}

void liftGripper() {
  radioSend("Lift gripper");
  for (int i = baseServo.read(); i < 105; i++) {
    baseServo.write(i);
    delay(15);
    //displayString(String(baseServo.read()), 0, 0);
  }
  delay(500);
}

void lowerGripper(int ang) {
  radioSend("Lower gripper");
  for (int i = baseServo.read(); i > ang-1; i--) {
    baseServo.write(i);
    delay(15);
    //displayString(String(baseServo.read()), 0, 0);
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

  while (readPING() < 25) {
    backStraight(250);
    stopMotors();
    delay(500);
  }
  //backStraight(500);
}

void forwardSlow() {

  while (readPING() > 17.5) {
    straight(100, 0);
    stopMotors();
    delay(500);
  }
  //straight(500);
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
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.display();
}

void takeTemp() {

  displayString("Taking temp", 0, 0);
  radioSend("take temp");
  
  lowerGripper(0);
  delay(1000);
  double init_temp = mlx.readObjectTempF();
  displayString("Place wrist under gripper arm.", 0, 0);
  radioSend("Waiting for wrist");
  while (mlx.readObjectTempF() < init_temp + 8) {
    Serial.println(mlx.readObjectTempF());
    delay(100);
  }
  //Serial.println(mlx.readObjectTempF());
  radioSend("Wrist detected, taking temp");
  displayString("Currently Reading Temp.", 0, 0);
  double total = 0;
  for (int i = 0; i < 50; i++) {
    Serial.println(mlx.readObjectTempF());
    total += mlx.readObjectTempF();
    delay(50);
  }

  total = total / 50;
  
  
  radioSend(String(total) + " °F.");
  displayString("Your Temp is \n" + String(mlx.readObjectTempF()) + " °F.", 5000, 1);
  
  liftGripper();
}

/*************Wireless Function*************/
boolean userCommandReceived() {
  if (Serial1.available()) {        // If HC-12 has data
    String str = Serial1.readStringUntil('\n');
    if (str != "") {
      str = str.substring(0, str.length() - 1);
      displayString(str, 0, 0);
      digitalWrite(2, HIGH);
      delay(3000);
      digitalWrite(2, LOW);
      //Serial.println(String(str.length()));
      //start = colorDetected();
      if(str =="pickup") {
        pickUp();
      } else if (str == "dropoff") {
        deposit();
      } else if (str == "disinfect") {
        disinfect();
      } else if (str == "temp") {
  
        takeTemp();
      } else if (str == "obs") {
        straight(5000, 1);
        stopMotors();
        delay(1500);
        turn(80);
        delay(1500);
        nudgeFollow(80000);
      } else if (str== "line") {
        state = -1;
        displayString("Following line", 0, 0);
      } else if (str.substring(0, 5) == "turn:") {
        int num = str.substring(6, str.length()).toInt();
        if (str.charAt(5) == '-') {
          turn(-num);
        } else if (str.charAt(5) == '+') {
          turn(num);
        }
      } else if (str.substring(0, 9) == "straight:") {
        int num = str.substring(10, str.length()).toInt();
        if (str.charAt(9) == '-') {
          backStraight(-num);
        } else if (str.charAt(9) == '+') {
          straight(num, 1);
        }
      } else if (str == "color") {
        unsigned long initT = millis();
        while (millis() < initT + 30000) {
          colorDetected();
        }
      }
    }
    return true;
  }
  displayString("Waiting for command", 0, 0);
  return false;
 
}

void turn(int angle) {
  double starting = getYaw();
  double targetAbs = getAbs(starting, angle);
  Setpoint = 0;
  resetPID();
  unsigned long startingTime = millis();

  double cur = getYaw();
  Input = getRel(cur, targetAbs);

  while ((millis() < startingTime + 5000) && abs(Input) > 10) {

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

void straight(int t, boolean checkPing) {
  double starting = getYaw();
  double targetAbs = starting;
  Setpoint = 0;
  resetPID();
  unsigned long startingTime = millis();

  double cur = getYaw();
  Input = getRel(cur, targetAbs);

  while (millis() < startingTime + t) {

    if (checkPing) {
      if (readPING() < 25) {
        stopMotors();
        return;
      }
    }
    
    //displayString("straight: " + String(Input), 0, 0);
    cur = getYaw();
    Input = getRel(cur, targetAbs);
   
    myPID.Compute();
    
    motorOut(pwmA + (Output), pwmA - (Output));
    delay(20);
  }

  stopMotors();
  delay(250);
}

void backStraight(int t) {
  double starting = getYaw();
  double targetAbs = starting;
  Setpoint = 0;
  resetPID();
  unsigned long startingTime = millis();

  double cur = getYaw();
  Input = getRel(cur, targetAbs);

  while (millis() < startingTime + t) {
    
    //displayString("straight: " + String(Input), 0, 0);
    cur = getYaw();
    Input = getRel(cur, targetAbs);
   
    myPID.Compute();
    
    motorOut(-pwmA + (Output * 0.9), -pwmA - (Output * 0.9));
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
  straight(3000, 1);
  delay(1500);
  turn(80);
  delay(1500);
  turn(80);

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
  
  straight(1500, 1);
}

void resetPID() {
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-115, 115);
  myPID.SetSampleTime(20);
}

void radioSend(String str) {
  Serial1.println(str);
}
