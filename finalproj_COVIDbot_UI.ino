#include <SPI.h>
#include <nRF24L01.h> 
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN pins
const byte address[6] = "00001";

void setup() {
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}

void loop() {
  Serial.println("Type a command.");
  while(!Serial.available()){}; //wait
  
  if(Serial.available() > 0) {
    //Convert Serial string input to char cmd[] packet
    String str = Serial.readString();
    int str_len = str.length() + 1;
    char cmd[str_len];
    str.toCharArray(cmd, str_len);

    if(isValid(cmd)){
      radio.write(&cmd, sizeof(cmd));
      delay(1000);
    } else {
      Serial.println("Invalid input.");
    }
  }
  
}

boolean isValid(char cmd[]) {
  return strcmp(cmd,"pickup")==0 || strcmp(cmd,"dropoff")==0 || 
  strcmp(cmd,"disinfect")==0 || strcmp(cmd,"temp")==0;
}
