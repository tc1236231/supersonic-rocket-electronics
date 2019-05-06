#include "DataRecorder.h"
#include <SPI.h>
#include <SD.h>

const int partitionInterval = 300000; //5 mins
unsigned long thresholdTime = 0;
int fileNumber = 0;
String currentFileName = "";
int randomSessionNumber = 0;

void DataRecorder::init()
{
    pinMode(SD_CS, OUTPUT);
    
    while(!SD.begin(SD_CS)) {
      Serial.println("SD Card initialization failed!");
      delay(1000);
    }

    randomSeed(42); 
    randomSessionNumber = random(10000);
    
    fileNumber = 0;
    currentFileName = String(randomSessionNumber,DEC) + "-" + String(fileNumber,DEC) + ".csv";
    if(SD.exists(currentFileName))
    {
      SD.remove(currentFileName);
    }
    thresholdTime = partitionInterval;
}

void DataRecorder::writeData(long currentTime, String string)
{
    File myFile = SD.open(currentFileName, FILE_WRITE);

    if(currentTime > thresholdTime)
    {
      thresholdTime = currentTime + partitionInterval;
      fileNumber++;
      currentFileName = String(randomSessionNumber,DEC) + "-" + String(fileNumber,DEC) + ".csv";
      if(SD.exists(currentFileName))
      {
        SD.remove(currentFileName);
      }
      myFile.close();
      myFile = SD.open(currentFileName, FILE_WRITE);
      Serial.println("partition newFile");
    }

    if (myFile) {
      int bytes = myFile.println(string);
      myFile.close();
      Serial.print(bytes);
      Serial.println(" bytes written");
    } else {
      // if the file didn't open, print an error:
      Serial.println("error opening .csv");
    }
}
