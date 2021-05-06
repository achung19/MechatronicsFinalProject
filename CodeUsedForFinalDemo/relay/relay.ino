/*
 * code for relay arduino on the robot, parses SPI commands from radio into serial commands
 */

#include <SoftwareSerial.h>
#include <SPI.h>
#include <nRF24L01.h> 
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN pins
const byte address[6] = "00001";


SoftwareSerial relay(4, 5); // HC-12 TX Pin, HC-12 RX Pin

void setup() {
  Serial.begin(9600);             // Serial port to computer
  relay.begin(9600);               // Serial port to HC12

  //set the default radio mode to be reading commands from the command station
  radio.begin();
  setRadioRead();
  
}
void loop() {

  //set the default radio mode to be reading commands from the command station
  setRadioRead();


  //when a radio command is received (already checked by command station), relay to robot via software serial link
  if (radio.available()) {
    char cmd[32] = "";
    radio.read(&cmd, sizeof(cmd));
    //Serial.println(String(cmd));
    //relay.println(String(cmd));
    if (strcmp(cmd, "") != 0) {
      Serial.println("Received from radio: " + String(cmd));
      Serial.println("Relaying to robot: " + String(cmd));
      relay.println(String(cmd));
      //displayString("Received: " + String(cmd), 0, 0);
      //if (String(cmd) == "line") {
      delay(1000);
      //}
    }
  } 

  //check to see there are messages from the robot that need to be relayed back to command station
  //if so, relay back via radio
  if (relay.available()) {        // If HC-12 has data
    String str = Serial.readStringUntil('\n');
    if (str != "") {
      radioSend(str);
      Serial.println("Received from robot: " + str);
      Serial.println("Sending back via radio");// Send the data to Serial monitor

    }
  }

  

  //relay.println("a");
  //delay(1000);
}



//functions for setting radio modes and sending messages via radio.
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

void radioSend(String str) {
  setRadioWrite();
  int str_len = str.length() + 1;
  char cmd[str_len];
  str.toCharArray(cmd, str_len);
  radio.write(&cmd, sizeof(cmd));
}
