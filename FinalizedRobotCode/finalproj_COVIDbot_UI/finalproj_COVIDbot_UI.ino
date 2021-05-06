#include <SPI.h>
#include <nRF24L01.h> 
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN pins
const byte address[6] = "00001";

#define numCommands 10

//acceptable commands, everything else gets cut
const char *validCmd[numCommands] = {"pickup", "dropoff", "disinfect", "temp", "obs", "line", "test1", "test2", "test3", "color"};
//commands that require the user to give arguments (turning time or straight line time): relies on separate substring checking 
String substr[4] = {"turn:+", "straight:+", "turn:-", "straight:-"};

void setup() {
  Serial.begin(9600);
  //HC12.begin(9600);
  radio.begin();
  setRadioRead();
}

void loop() {

  //set the system default to accept reports from the robot radio
  setRadioRead();

  if (radio.available()) {
    char text[64] = "";
    radio.read(&text, sizeof(text));
    Serial.println("Received: " + String(text));
  }

  //when there is a command to be sent, switch to send mode, and send command text after checking the commands
  if(Serial.available() > 0) {

    setRadioWrite();
    //Convert Serial string input to char cmd[] packet
    String str = Serial.readStringUntil('\n');
    int str_len = str.length() + 1;
    char cmd[str_len];
    str.toCharArray(cmd, str_len);

    if(isValid(cmd, str)){
      radio.write(&cmd, sizeof(cmd));
      Serial.println("Sent: " + String(cmd));
    } else {
      Serial.println("Invalid input.");
    }
    
  }

  
  
}

//command checker
boolean isValid(char cmd[], String str) {
  
  for (int i = 0; i < numCommands; i++) {
    if (strcmp(cmd,validCmd[i]) == 0) {
      return 1;
    }
  }

  for (int i = 0; i < 4; i++) {
    if (str.substring(0, substr[i].length()) == substr[i]) {
      return 1;
    }
  }
  return 0;
 
}

//functions to set radio mode (sending/receiving)
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
