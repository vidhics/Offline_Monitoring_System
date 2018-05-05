#include <SoftwareSerial.h>
 
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600; //Baudrate of your GPS here
 
// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
 
void setup()
{
  Serial.begin(115200); // Beginning the serial monitor at Baudrate 115200 and make sure you select same in serial monitor
  ss.begin(GPSBaud);
}
 
void loop()
{
  // Output raw GPS data to the serial monitor
  while (ss.available() > 0){
    Serial.write(ss.read());
  }
}
