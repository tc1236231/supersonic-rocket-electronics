#include "DataRecorder.h"
#include <SD.h>

void DataRecorder::init()
{
    pinMode(SD_CS, OUTPUT);
    
    while(!SD.begin(SD_CS)) {
      Serial.println("SD Card initialization failed!");
      delay(1000);
    }
}

void DataRecorder::writeData(char* string)
{
    File myFile = SD.open("sample.csv", FILE_WRITE);
 
    if (myFile) {
      myFile.println(string);
      myFile.close();
    } else {
      // if the file didn't open, print an error:
      Serial.println("error opening sample.csv");
    }
}
