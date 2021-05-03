#include <SPI.h>
#include <nRF24L01.h> 
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN pins
const byte address[6] = "00001";

#define numCommands 5
const char *validCmd[numCommands] = {"pickup", "dropoff", "disinfect", "temp", "obs"};

void setup() {
  Serial.begin(9600);
  radio.begin();
  setRadioRead();
}

void loop() {
  setRadioRead();

  if (radio.available()) {
    char text[64] = "";
    radio.read(&text, sizeof(text));
    Serial.println(text);
  }
  
  if(Serial.available() > 0) {

    setRadioWrite();
    //Convert Serial string input to char cmd[] packet
    String str = Serial.readStringUntil('\n');
    int str_len = str.length() + 1;
    char cmd[str_len];
    str.toCharArray(cmd, str_len);

    if(isValid(cmd)){
      radio.write(&cmd, sizeof(cmd));
      Serial.println("Sent: " + String(cmd));
    } else {
      Serial.println("Invalid input.");
    }
    
  }

  
  
}

boolean isValid(char cmd[]) {
  
  for (int i = 0; i < numCommands; i++) {
    if (strcmp(cmd,validCmd[i]) == 0) {
      return 1;
    }
  }
  return 0;
 
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
