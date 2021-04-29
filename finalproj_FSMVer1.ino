#include <Pixy2.h>
#include <PIDLoop.h>
#include <Servo.h>

/***************Variables***************/
//Motor vars:
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
//Motor speeds, maximum allowed is 255:
#define SPEED_FAST        150 //**adjust!
#define SPEED_SLOW        100 //**adjust!
#define X_CENTER          (pixy.frameWidth/2)

//Misc. sensor/actuator vars:
#define Ping 4
#define servoPin 7
#define servoPin2 8
Servo servoMotor;
Servo servoMotor2;
Pixy2 pixy;
PIDLoop headingLoop(5000, 0, 0, false);

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
  Serial.println("Starting up...");

  //initialize motors
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

  //intialize Pixy:
  pixy.init();
  pixy.changeProg("line");

  //initialize servo motors:
  servoMotor.attach(servoPin);
  servoMotor2.attach(servoPin2);
  
  //initialize state variables:
  state = 0;
  
  delay(2000);
}

void loop() {

  switch(state) 
  {
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

/*************Wireless Function*************/
boolean userCommandRecieved() {
  //true when wireless module recieves user command
  //update global "userCommand" variable to the state of the user-desired task (5-8)
  //update "start" and "destination" chars with appropriate path colors
  
  //if wireless recieved {
  //  start = colorDetected();
  //  if (typedCommand == "pickup") {
  //      destination = paths[0];
  //      task = 3;
  //  } else if (typedCommand == "dropoff") {
  //      destination = paths[1]; 
  //      task = 4;
  //  } else if (typedCommand == "disinfect") {
  //      destination = paths[2];
  //      task = 5;
  //  } else if (typedCommand == "temp") {
  //      destination = paths[3];
  //      task = 6;
  //  } else { //typedCommand is unreadable
  //      Serial.println("Please type command again:\n");
  //      return false;
  //  }
  //  return true;
  //}
  
  //return false; //no command received
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

  //set left and right motor speeds
  analogWrite(enA, left);
  analogWrite(enB, left);
  analogWrite(enC, right);
  analogWrite(enD, right);
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
