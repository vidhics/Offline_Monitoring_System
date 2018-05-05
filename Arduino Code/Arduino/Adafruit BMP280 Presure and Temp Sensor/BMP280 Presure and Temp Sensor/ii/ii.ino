//#include "SPI.h"

#include "Arduino.h"
#include "Adafruit_BMP280.h"
#include "Adafruit_Sensor.h"


#include "Wire.h"


Adafruit_BMP280 bmp;

void setup(){


  Serial.begin(9600);
  if(!bmp.begin()){
    Serial.println("Cannot connect to BMP280 ");
    while(1);
  }
}

long currentTime,lastTime;

void loop(){

  currentTime = millis();
  if(currentTime - lastTime > 2000) {

  Serial.print("Temperature:");
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");
  Serial.print("Pressure : ");
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

   lastTime = millis();
  
}
}

