/***************************************************************************
  This is a library for the BMP280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BMEP280 Breakout 
  ----> http://www.adafruit.com/products/2651

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required 
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include "bmp280.h"
#include <Adafruit_Sensor.h> // require installing library Adafruit Unified Sensor
#include <Adafruit_BMP280.h> // require installiong library Adafruit BMP280

//Adafruit_BMP280 bme; // I2C
Adafruit_BMP280 bme(BMP_CS); // hardware SPI
//Adafruit_BMP280 bme(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

BMP280::BMP280(float seaLevelhPa)
{
    this->seaLevelhPa = seaLevelhPa;
}
  
void BMP280::init() {  
  while (!bme.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    delay(1000);
  }
}
  
String BMP280::collectData() {
//    float temp = bme.readTemperature();
    float pressure = bme.readPressure();
//    float altitude = bme.readAltitude(this->seaLevelhPa);
    
    String output;
//    output = temp;
//    output += ',';
    output += pressure;
//    output += ',';
//    output += altitude;

//    Serial.print("Temperature = ");
//    Serial.print(temp);
//    Serial.println(" *C");
    
//    Serial.print("Pressure = ");
//    Serial.print(pressure);
//    Serial.println(" Pa");

//    Serial.print("Approx altitude = ");
//    Serial.print(altitude); // this should be adjusted to your local forcase
//    Serial.println(" m");
    
//    Serial.println();
    return output;
}
