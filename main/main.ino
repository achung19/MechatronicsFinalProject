#include <Servo.h>
#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display = Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define gServo 10
#define baseServo1 11
#define baselineGrip 85

#define IRpin A0

#define enA 3 //right back
//#define enB 5 //left front
//#define enC 6 //right front
#define enD 9 //left back

#define pwmA 100
#define turnVar 40

#define inA1 38
#define inA2 39
//#define inB1 41
//#define inB2 40
//#define inC1 45
//#define inC2 44
#define inD1 46
#define inD2 47

#define Ping 4

Servo gripperServo;
Servo baseServo;

double Setpoint, Input, Output;
double Kp=2.1, Ki=0, Kd=0;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {

  mlx.begin(); 
  displayInit();

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-85, 85);
  myPID.SetSampleTime(25);
  Setpoint = 20;
  
  baseServo.attach(baseServo1);
  
  Serial.begin(9600);
  pinMode(enA, OUTPUT);
  pinMode(enD, OUTPUT);
  pinMode(inA1, OUTPUT);
  pinMode(inA2, OUTPUT);
  pinMode(inD1, OUTPUT);
  pinMode(inD2, OUTPUT);
  stopMotors();
  
  delay(5000);


  //displayString("Hello", 5000, 1);
  //displayString("world", 0, 0);
  
  //pickUp();
  //delay(3000);
  //deposit();

  takeTemp();
  
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
  

  
  delay(50);
  
}

void nudgeFollow() {
  double ir = IRDistance();
  double ping = readPING();
  if (ping < 10) {
    stopMotors();
    delay(500);
    right();
    forward();
    delay(1000);
  } else if (ping < 35) {
    while (readPING() > 15) {
      forward();
      delay(20);
    }
  } 
  /*else if (ir > 25) {
    int counter = 0;
    stopMotors();
    delay(500);
    for (int i = 0; i < 3; i++) {
      forward();
      delay(100);
      ir = IRDistance();
      if (ir > 25) {
        counter++;
      }
    } 
    if (counter > 2) {
      stopMotors();
      delay(500);
      left();
      forward();
      delay(1000);
    }
  } */
  else if (ir > 20) {
    slightLeft();
  } else if (ir < 10) {
    slightRight();
  } else {
    forward();
    delay(50);
  }
}

void slightLeft() {
  stopMotors();
  delay(500);
  motorOut(100, -100);
  delay(350);
  motorOut(90, 90);
  delay(500);
  stopMotors();
  delay(500);
}

void slightRight() {
  stopMotors();
  delay(500);
  motorOut(-100, 100);
  delay(250);
  motorOut(90, 90);
  delay(500);
  stopMotors();
  delay(500);
}

void PIDfollow() {
  Input = IRDistance();
  if (Input > 25) {
    int counter = 0;
    stopMotors();
    delay(1000);
    for (int i = 0; i < 10; i++) {
      forward();
      delay(100);
      Input = IRDistance();
      if (Input > 25) {
        counter++;
      }
    } 
    if (counter > 9) {
      stopMotors();
      delay(1000);
      left();
      forward();
      delay(1000);
    }
  } else {
    myPID.Compute();
    int right = pwmA - Output;
    int left = pwmA + Output;
    //if (right == 0) {
      //right = -100;
      //left = 150;
    //} else if (left == 0) {
      //left = -100;
      //right = 150;
    //}
    motorOut(right, left);
  }
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
  //analogWrite(enB, abs(left));
  //analogWrite(enC, abs(right));
  analogWrite(enD, abs(left)); 
  Serial.println(String(right) + " , " + String(left));
}

void forward() {
  motorOut(pwmA, pwmA);
}

void right() {
  motorOut(-100, 100);
  delay(700);
  stopMotors();
  delay(500);
}

void left() {
  motorOut(100, -100);
  delay(700);
  stopMotors();
  delay(500);
}

void back() {
  motorOut(-pwmA, -pwmA);
}

void rightForward() {
  digitalWrite(inA1, LOW);
  digitalWrite(inA2, HIGH);
  //digitalWrite(inC1, LOW);
  //digitalWrite(inC2, HIGH);
}

void leftForward() {
  //digitalWrite(inB1, LOW);
  //digitalWrite(inB2, HIGH);
  digitalWrite(inD1, LOW);
  digitalWrite(inD2, HIGH);
}

void rightBack() {
  digitalWrite(inA1, HIGH);
  digitalWrite(inA2, LOW);
  //digitalWrite(inC1, HIGH);
  //digitalWrite(inC2, LOW);
}

void leftBack() {
  //digitalWrite(inB1, HIGH);
  //digitalWrite(inB2, LOW);
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
  delay(2000);
  for (int i = gripperServo.read(); i > 80; i--) {
    gripperServo.write(i);
    delay(100);
    Serial.println(gripperServo.read());
  }
  delay(2000);
}

void grip() {
  for (int i = gripperServo.read(); i < 105; i++) {
    gripperServo.write(i);
    delay(100);
    Serial.println(gripperServo.read());
  }
  delay(2000);
}

void liftGripper() {
  for (int i = baseServo.read(); i < 105; i++) {
    baseServo.write(i);
    delay(30);
    Serial.println(baseServo.read());
  }
  delay(2000);
}

void lowerGripper() {
  for (int i = baseServo.read(); i > -1; i--) {
    baseServo.write(i);
    delay(30);
    Serial.println(baseServo.read());
  }
  delay(2000);
}


float IRDistance() {
  float volt = analogRead(IRpin) * 0.0048828125;
  //return volt;
  float dist = 1.9735 * volt * volt - 15.378 * volt + 35.352;
  Serial.println(dist);
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
  delay(2000);
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.display();
}

void takeTemp() {
  lowerGripper();
  delay(3000);
  double init_temp = mlx.readObjectTempF();
  displayString("Please place wrist under the gripper arm.", 0, 0);
  while (mlx.readObjectTempF() < init_temp + 8) {
    Serial.println(mlx.readObjectTempF());
    delay(100);
  }
  Serial.println(mlx.readObjectTempF());
  displayString("Currently Reading Temperature. Please do not move your arm...", 3000, 1);
  displayString("Your Temperature is \n" + String(mlx.readObjectTempF()) + " °F.", 5000, 1);
  liftGripper();
}
