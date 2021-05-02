#include <Pixy2.h>
#include <PIDLoop.h>
#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h> 
#include <RF24.h>

/***************Variables***************/
//Motor vars:
#define enA 3 //right back
#define enD 9 //left back
#define pwmA 100
#define inA1 38
#define inA2 39
#define inD1 46
#define inD2 47


//Motor speeds, maximum allowed is 255:
#define SPEED_FAST        150 //**adjust!
#define SPEED_SLOW        100 //**adjust!
#define X_CENTER          (pixy.frameWidth/2)

//Misc. sensor/actuator vars:
#define Ping 4
#define servoPin 11
#define servoPin2 12
Servo servoMotor;
Servo servoMotor2;
Pixy2 pixy;
PIDLoop headingLoop(5000, 0, 0, false);
RF24 radio(7, 8);                     // CE, CSN pins     
const byte address[6] = "00001";

//Program vars:
int state;                            //current state of FSM
int task;                             //state of user-desired task 

//The robot will always navigate from start->ring->destination
//colors are encoded as characters: 'g', 'r', 'b', 'p' and 'y'
char paths[4] = {'g', 'r', 'b', 'p'}; //color of node paths
char ring = 'y';                      //color of ring path
char start;                           //color path that robot begins navigation 
char destination;                     //color path of the destination node
char curr;                            //color path that robot is currently following      
/**************************************/

void setup() {
  Serial.begin(9600);
  Serial.println("Starting up...");

  //initialize motors
  pinMode(enA, OUTPUT);
  pinMode(enD, OUTPUT);
  pinMode(inA1, OUTPUT);
  pinMode(inA2, OUTPUT);
  pinMode(inD1, OUTPUT);
  pinMode(inD2, OUTPUT);
  stopMotors();

  //intialize Pixy:
  pixy.init();
  pixy.changeProg("line");

  //initialize servo motors:
  servoMotor.attach(servoPin);
  servoMotor2.attach(servoPin2);

  //begin wireless communication
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  
  //initialize state variables:
  state = -1;
  
  delay(2000);
}

void loop() {

  switch(state) 
  {
    case -1: { //test pixy line following program
      lineFollowingTest();
      break;
    }
    case 0: { //awaiting a command/stop
      stopMotors();
      if(userCommandRecieved()) {
        state = 1;
      } 
      else if(destinationReached()) { //assuming path will end slightly before destination is reached
        state = task;
      }
      break;
    }
    case 1: { //follow line
      int8_t res = pixy.line.getMainFeatures();
      if(obstacleDetected()) { //if an obstacle is detected, break
        stopMotors();
        state = 4; 
      }
      else if(res <= 0) { //no line detected, stall
        state = 0;
      }
      else if(res&LINE_VECTOR) { //if line detected, follow the line
        followLine();
      } 
      else if(res&LINE_INTERSECTION) {
        stopMotors(); //stop and check color
        makeNavigation();
      }
      break;
    } 
    case 2: { //obstacle ovidance
      //if obstacle detected, follow edge of obstacle
      //re-find path, then
      state = 1;
      break;
    }
    case 3: { //pick up vial
      pickUpVial();
      resetVars();
      state = 0;
      break;
    }
    case 4: { //drop off vial
      dropOffVial();
      resetVars();
      state = 0;
      break;
    }
    case 5: { //disinfect
      disinfect();
      resetVars();
      state = 0;
      break;
    }
    case 6: { //check patient temperature
      checkTemperature();
      resetVars();
      state = 0;
      break;
    }
  }
}

/******************Test*********************/
void lineFollowingTest() {
  int8_t res = pixy.line.getMainFeatures();
  if(res <= 0) { //no line detected, stall
    Serial.println("no line detected...switching to stall");
  }
  else if(res&LINE_VECTOR) { //if line detected, follow the line
    followLine();
  } 
  else if(res&LINE_INTERSECTION) {
    Serial.println("DETECTED INTERSECTION");
    stopMotors(); //stop and check color
    Serial.print("detected: ");
      Serial.println(colorDetected());
    pixy.line.setNextTurn(-90);
  }
  delay(1000);
}
/*************Wireless Function*************/
boolean userCommandRecieved() {
  //true when wireless module recieves user command
  //update global "userCommand" variable to the state of the user-desired task (3-6)
  //update "start" and "destination" chars with appropriate path colors
  
  if (radio.available()) {
    char cmd[32] = "";
    radio.read(&cmd, sizeof(cmd));
    start = colorDetected();
    if(strcmp(cmd,"pickup") == 0) {
      destination = paths[0];
      task = 3;
    } else if (strcmp(cmd,"dropoff") == 0) {
      destination = paths[1]; 
      task = 4;
    } else if (strcmp(cmd,"disinfect") == 0) {
      destination = paths[2];
      task = 5;
    } else if (strcmp(cmd,"temp") == 0) {
      destination = paths[3];
      task = 6;
    } 
    return true;
  }
  return false;
}

/************Navigation Functions************/
void makeNavigation() {
  //robot will always navigate from start->ring->destination
  if(curr == start) {                     //if robot was on starting path
    if(colorDetected() == ring) {         //if ring detected, turn right
      curr = ring;
      pixy.line.setNextTurn(-90); 
    }
  } 
  else if(curr == ring) {                 //if robot was on ring path
    if(colorDetected() == destination) {  //if destination path detected, turn right
      curr = destination;
      pixy.line.setNextTurn(-90);
    } else {                              //if irrelevant color paths detected, continue straight
      pixy.line.setNextTurn(0);
    }
  } 
}

boolean destinationReached() {
  return (curr == destination);
}

/***************Motor Functions**************/
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
  
void stopMotors() {
  motorOut(0, 0);  
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

/*************Distance Functions*************/
unsigned long measureDistance() {
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

boolean obstacleDetected() {
  //true if obstacle detected less than 10cm ahead
  return readPING() < 10;
}

/*************Pixy Functions***************/
char colorDetected() {
  //returns color currently visible by Pixy
  pixy.changeProg("color_connected_components");
  pixy.ccc.getBlocks();
  char color = ' ';

  if (pixy.ccc.numBlocks) {  
    int signature = (int32_t)pixy.ccc.blocks[0].m_signature; //color signature

    switch(signature) 
    {
      case 1: { color = 'g';} //signature 1: green
      case 2: { color = 'r';} //signature 2: red
      case 3: { color = 'b';} //signature 3: blue
      case 4: { color = 'p';} //signature 4: purple
      case 5: { color = 'y';} //signature 5: yellow
    }
  }
  
  pixy.changeProg("line");
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
    if (pixy.line.vectors->m_flags&LINE_FLAG_INTERSECTION_PRESENT) 
    {
      left += SPEED_SLOW;
      right += SPEED_SLOW;
    } 
    else // otherwise, pedal to the metal!
    { 
      left += SPEED_FAST;
      right += SPEED_FAST;
    }    
  } 
  else // If the vector is pointing down, or down-ish, we need to go backwards to follow.
  {  
    left -= SPEED_SLOW;
    right -= SPEED_SLOW;  
  } 

  //set right and left motor speeds
  motorOut(right, left);
  delay(100);                       //**adjust!
}

/**********Component/Task Functions**********/
void dropOffVial() {
}

void pickUpVial() {
}

void disinfect(){  
  //one spray action
  for(int angle = 0; angle < 180; angle += 30){                                  
    servoMotor.write(180-angle);  
    servoMotor2.write(angle);   
    delay(15); 
    if (angle<10){
      delay(100);                      
    }
  } 
  delay(1000);
}

void checkTemperature() {
}

/*******************Reset******************/
void resetVars() {
  start = ' ';
  destination = ' '; 
  curr = ' '; 
  task = 0; 
}
