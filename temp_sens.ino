#include <Wire.h>
#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

void setup() {
  Serial.begin(9600);

  Serial.println("Adafruit MLX90614 test");  

  mlx.begin();  
}

void loop() {
  Serial.print("Object = "); 
  Serial.print(mlx.readObjectTempF()); 
  Serial.println("*F");

  Serial.println();
  delay(500);
}
