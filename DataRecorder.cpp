#include "DataRecorder.h"
#include <SPI.h>
#include <SD.h>

void DataRecorder::init()
{
    pinMode(SD_CS, OUTPUT);
    
    while(!SD.begin(SD_CS)) {
      Serial.println("SD Card initialization failed!");
      delay(1000);
    }
}

void DataRecorder::writeData(String string)
{
    Serial.println("opening sample.csv");
    File myFile = SD.open("sample.csv", FILE_WRITE);
 
    if (myFile) {
      int bytes = myFile.println(string);
      myFile.close();
      Serial.print(bytes);
      Serial.println(" bytes written to sample.csv");
    } else {
      // if the file didn't open, print an error:
      Serial.println("error opening sample.csv");
    }
}
