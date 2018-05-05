#include<Wire.h>
#include<mySD.h>
 
// BME280 I2C address is 0x76(108)
#define Addr 0x76






// For ESP32 Bluetooth
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLECharacteristic *pCharacteristic;
File root;

bool deviceConnected = false;
float txValue = 0;


double cTemp=0.0;
double pressure = 0.0;

#define LED 2

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"



unsigned long sendRate = 10000; // Send data to app every 2s
unsigned long t_start = 0; // For keeping time during reflow process
unsigned long previousMillis = 0;
unsigned long duration, t_final, windowStartTime, timer;


// Bluetooth app settings. Define which characters belong to which functions
#define dataChar "*" // App is receiving data from ESP32
#define stopChar "!" // App is receiving command to stop receiving data from ESP32
#define startflow "A" // Command from app to "activate" the data logging process
#define stopflow "S" // Command from app to "stop" the data logging
#define download "D"

bool temp = false; 
bool dwld = false;
bool first = true;
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();
    char s[10];
    int j=0;
    if (rxValue.length() > 0) {
      Serial.print("<-- Received Value: ");

      for (int i = 0; i < rxValue.length(); i++) {
        Serial.print(rxValue[i]);
        
        if(i>0)
        {
          s[j] =  rxValue[i];
        //S+= rxValue[i];
        j++;
        }
      }
      Serial.println();

      // Do stuff based on the command received from the app
      if (rxValue.find("A") != -1) { // Command from app to start datalogging
        
        temp = true;
        t_start = millis(); // Record the start time
        timer = millis(); // Timer for logging data points
        Serial.println("<-- ***Data Logging started!"); // Left arrow means it received a command
      }
      else if (rxValue.find("S") != -1) { // Command to stop reflow process
        digitalWrite(LED, LOW); // Turn off LED
        temp = false;
        Serial.println("<-- ***Data Logging stopped!");
      }
      else if (rxValue.find("D") != -1){
        Serial.println("Download command received");
        dwld = true;
        Serial.println("Download started!");
      } else if(rxValue.find("T") != -1){
          Serial.println("Interval Received--->");
          //Serial.println(S);
          int t = atoi(s);
          Serial.println(t);
          sendRate = t*1000;
      }
      // Add you own functions here and have fun with it!
    }
  }
};

void setup() {
  Wire.begin();
  Serial.begin(9600);

  Serial.println("***** ESP32 Temperature/Pressure Sensor*****");
  
  
  pinMode(LED, OUTPUT);

  digitalWrite(LED, LOW); // Set default LED state to OFF

  Serial.print("Initializing SD card...");
  /* initialize SD library with SPI pins */
  if (!SD.begin(5, 23, 19, 18)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  /* Begin at the root "/" */
 
  /* open "data.txt" for writing */
   root = SD.open("data.txt", FILE_WRITE);
    if (root) {
    root.flush();
   /* close the file */
    root.close();
  } else {
    /* if the file open error, print an error */
    Serial.println("inside setup - error opening data.txt");
  }



  /***************************** BLUETOOTH SETUP *****************************/
  // Create the BLE Device
  BLEDevice::init("ESP32"); // Give it a name

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
                      
  pCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting for a BLE client to connect...");
}



void loop() { 
  /***************************** TEMPERATURE/PRESSURE PROCESS CODE *****************************/
  if (temp) {
    digitalWrite(LED, HIGH); // Blue LED indicates logging data in on
 unsigned int b1[24];
unsigned int data[8];
unsigned int dig_H1 = 0;
for(int i = 0; i < 24; i++)
{
// Start I2C Transmission
Wire.beginTransmission(Addr);
// Select data register
Wire.write((136+i));
// Stop I2C Transmission
Wire.endTransmission();
 
// Request 1 byte of data
Wire.requestFrom(Addr, 1);
 
// Read 24 bytes of data
if(Wire.available() == 1)
{
b1[i] = Wire.read();
}
}
 
// Convert the data
// temp coefficients
unsigned int dig_T1 = (b1[0] & 0xff) + ((b1[1] & 0xff) * 256);
int dig_T2 = b1[2] + (b1[3] * 256);
int dig_T3 = b1[4] + (b1[5] * 256);
 
// pressure coefficients
unsigned int dig_P1 = (b1[6] & 0xff) + ((b1[7] & 0xff ) * 256);
int dig_P2 = b1[8] + (b1[9] * 256);
int dig_P3 = b1[10] + (b1[11] * 256);
int dig_P4 = b1[12] + (b1[13] * 256);
int dig_P5 = b1[14] + (b1[15] * 256);
int dig_P6 = b1[16] + (b1[17] * 256);
int dig_P7 = b1[18] + (b1[19] * 256);
int dig_P8 = b1[20] + (b1[21] * 256);
int dig_P9 = b1[22] + (b1[23] * 256);
 
// Start I2C Transmission
Wire.beginTransmission(Addr);
// Select data register
Wire.write(161);
// Stop I2C Transmission
Wire.endTransmission();
 
// Request 1 byte of data
Wire.requestFrom(Addr, 1);
 
// Read 1 byte of data
if(Wire.available() == 1)
{
dig_H1 = Wire.read();
}
 
for(int i = 0; i < 7; i++)
{
// Start I2C Transmission
Wire.beginTransmission(Addr);
// Select data register
Wire.write((225+i));
// Stop I2C Transmission
Wire.endTransmission();
 
// Request 1 byte of data
Wire.requestFrom(Addr, 1);
 
// Read 7 bytes of data
if(Wire.available() == 1)
{
b1[i] = Wire.read();
}
}
 
// Convert the data
// humidity coefficients
int dig_H2 = b1[0] + (b1[1] * 256);
unsigned int dig_H3 = b1[2] & 0xFF ;
int dig_H4 = (b1[3] * 16) + (b1[4] & 0xF);
int dig_H5 = (b1[4] / 16) + (b1[5] * 16);
int dig_H6 = b1[6];
 
// Start I2C Transmission
Wire.beginTransmission(Addr);
// Select control humidity register
Wire.write(0xF2);
// Humidity over sampling rate = 1
Wire.write(0x01);
// Stop I2C Transmission
Wire.endTransmission();
 
// Start I2C Transmission
Wire.beginTransmission(Addr);
// Select control measurement register
Wire.write(0xF4);
// Normal mode, temp and pressure over sampling rate = 1
Wire.write(0x27);
// Stop I2C Transmission
Wire.endTransmission();
 
// Start I2C Transmission
Wire.beginTransmission(Addr);
// Select config register
Wire.write(0xF5);
// Stand_by time = 1000ms
Wire.write(0xA0);
// Stop I2C Transmission
Wire.endTransmission();
 
for(int i = 0; i < 8; i++)
{
// Start I2C Transmission
Wire.beginTransmission(Addr);
// Select data register
Wire.write((247+i));
// Stop I2C Transmission
Wire.endTransmission();
 
// Request 1 byte of data
Wire.requestFrom(Addr, 1);
 
// Read 8 bytes of data
if(Wire.available() == 1)
{
data[i] = Wire.read();
}
}
 
 
// Convert pressure and temperature data to 19-bits
long adc_p = (((long)(data[0] & 0xFF) * 65536) + ((long)(data[1] & 0xFF) * 256) + (long)(data[2] & 0xF0)) / 16;
long adc_t = (((long)(data[3] & 0xFF) * 65536) + ((long)(data[4] & 0xFF) * 256) + (long)(data[5] & 0xF0)) / 16;
// Convert the humidity data
long adc_h = ((long)(data[6] & 0xFF) * 256 + (long)(data[7] & 0xFF));
 
// Temperature offset calculations
double var1 = (((double)adc_t) / 16384.0 - ((double)dig_T1) / 1024.0) * ((double)dig_T2);
double var2 = ((((double)adc_t) / 131072.0 - ((double)dig_T1) / 8192.0) *
(((double)adc_t)/131072.0 - ((double)dig_T1)/8192.0)) * ((double)dig_T3);
double t_fine = (long)(var1 + var2);
cTemp = (var1 + var2) / 5120.0;
double fTemp = cTemp * 1.8 + 32;
 
// Pressure offset calculations
var1 = ((double)t_fine / 2.0) - 64000.0;
var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
var2 = var2 + var1 * ((double)dig_P5) * 2.0;
var2 = (var2 / 4.0) + (((double)dig_P4) * 65536.0);
var1 = (((double) dig_P3) * var1 * var1 / 524288.0 + ((double) dig_P2) * var1) / 524288.0;
var1 = (1.0 + var1 / 32768.0) * ((double)dig_P1);
double p = 1048576.0 - (double)adc_p;
p = (p - (var2 / 4096.0)) * 6250.0 / var1;
var1 = ((double) dig_P9) * p * p / 2147483648.0;
var2 = p * ((double) dig_P8) / 32768.0;
pressure = ((p + (var1 + var2 + ((double)dig_P7)) / 16.0) / 100)*0.0145;
 
// Humidity offset calculations
double var_H = (((double)t_fine) - 76800.0);
var_H = (adc_h - (dig_H4 * 64.0 + dig_H5 / 16384.0 * var_H)) * (dig_H2 / 65536.0 * (1.0 + dig_H6 / 67108864.0 * var_H * (1.0 + dig_H3 / 67108864.0 * var_H)));
double humidity = var_H * (1.0 - dig_H1 * var_H / 524288.0);
if(humidity > 100.0)
{
humidity = 100.0;
}
else if(humidity < 0.0)
{
humidity = 0.0;
}

  } else {

     digitalWrite(LED, LOW);
  }

/*   /***************************** BLUETOOTH CODE *****************************/
  
       
//Send data to the app periodically
  if (millis() - previousMillis > sendRate) {
    previousMillis = millis();
    root = SD.open("data.txt", FILE_WRITE);
    if (root && first)
    {
      SD.remove("data.txt");
      first = false;
    } else if (first == false) {
      root = SD.open("data.txt", FILE_WRITE);
     
      root.print(cTemp);
      root.print(",");
      root.print(pressure);
      root.println(",");
    //root.print(",");
    //root.println(pressure);
    //root.print(",");
    root.flush();
   /* close the file */
    root.close();
  } else {
    /* if the file open error, print an error */
    Serial.println("inside loop - error opening data.txt");
  }  

     // Only send temperature value if it's legit and if connected via BLE
    if (deviceConnected && (dwld==false)) {
      // First convert the value to a char array
      char tempBuff[16]; // make sure this is big enuffz
      char pressureBuff[16];
      char txString[16];
      dtostrf(cTemp, 1, 2, tempBuff);
      dtostrf(pressure, 1, 2, pressureBuff);
      // float_val, min_width, digits_after_decimal, char_buffer
      sprintf(txString, "%s,%s,", tempBuff, pressureBuff); // The 'dataChar' character indicates that we're sending data to the app
      
      pCharacteristic->setValue(txString);
      pCharacteristic->notify(); // Send the value to the app
    }

  }

    if(deviceConnected && (dwld == true)){
      root = SD.open("data.txt");
      char buffer[16];
      char pbuffer[16];
      int count=0;
      int bufindex=0;
      int pbufindex=0;
      float pressdata;
      float tempdata;
      bool tbuff = false;
      bool pbuff = false; 
      if(root)
      {    
          //Serial.print("Inside Download root");
          while (root.available()){
         char c = root.read();
            //Serial.print("Char ");
            //Serial.println(c);
            while(count=0)
            {
            if ( c!= ','){
              //Serial.println("Inside count 0");
              buffer[bufindex] = c;
              bufindex++;
              }
              else{
                //Serial.print("INside else");
                tempdata = atof(buffer);
                bufindex=0;
                count++;
                memset(buffer, 0, 255);
                //Serial.println(tempdata);
                tbuff=true;
              }
            }
            while(count== 1)
            {
              if ( c!= ','){
              pbuffer[pbufindex] = c;
              pbufindex++;
              }
              else{
               // Serial.print("INside pressure else");
                 pressdata = atof(pbuffer);
                pbufindex=0;
                count=0;
                memset(pbuffer, 0, 255);
               // Serial.println(pressdata);
                pbuff=true;
              }
            }
             if(tbuff && pbuff)
             {
             char tempBuff[16]; // make sure this is big enuffz
             char pressureBuff[16];
             char txString[16];
             dtostrf(tempdata, 1, 2, tempBuff);
             dtostrf(pressdata, 1, 2, pressureBuff);
      // float_val, min_width, digits_after_decimal, char_buffer
              sprintf(txString, "%s,%s,", tempBuff,pressureBuff);// The 'dataChar' character indicates that we're sending data to the app
              pCharacteristic->setValue(txString);
              pCharacteristic->notify();
         
        
          }
          }
          
          root.close();
          dwld=false;
          }
    }
  }
  




  













