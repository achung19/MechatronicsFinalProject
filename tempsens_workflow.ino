#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display = Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define Message_Display 1
#define Temperature_Reading 2
#define Temperature_Display 3

int state = 1;
int init_temp = 0;

void setup() {
  Serial.begin(115200);

  mlx.begin(); 

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  delay(2000);
  display.clearDisplay();

  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,10);             // Start at top-left corner
  
  display.display();
}

void loop() {
  switch(state) {
    case Message_Display: {
      // TODO: Lower gripper arm code here

      delay(3000);
      init_temp = mlx.readObjectTempF();
      display.clearDisplay();
      display.println("Please place wrist under the gripper arm.");
      display.display();   
      state = Temperature_Reading;   
      break;
    }

    case Temperature_Reading: {
      if(mlx.readObjectTempF() > init_temp + 8) {
        state = Temperature_Display;
      }
      break;
    }

    case Temperature_Display: {
      display.clearDisplay();
      display.println("Currently Reading Temperature. Please do not move your arm...");
      display.display();

      delay(5000);

      display.clearDisplay();
      display.println("Your Temperature is ");
      display.print(mlx.readObjectTempF());
      display.println("°F.");
      display.display();
      
      delay(3000);
      state = Message_Display;
      break;
    }
  }
  
}
