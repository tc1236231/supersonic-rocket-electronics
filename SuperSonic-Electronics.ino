#include "bmp280.h"
#include "SparkFun_LIS331.h"
#include "i3g4250d.h"
#include "DataRecorder.h"

BMP280 bmp(1013.25); //!!! Sea Level Pressure For the Day
LIS331 acc;
i3g4250d gyro;
DataRecorder recorder;

void setup() {
  Serial.begin(9600);
  bmp.init();
  acc.init();
  gyro.init();
  recorder.init();
}

void loop() {
  static long loopTimer = 0;
  if (millis() - loopTimer > 100)
  {
    loopTimer = millis();
    char* bmp_data = bmp.collectData();
    char* acc_data = acc.collectData();
    char* gyro_data = gyro.collectData();
    char totalData[512];
    sprintf(totalData, "%ld,%s,%s,%s", loopTimer,bmp_data,acc_data,gyro_data);
    recorder.writeData(totalData);
  }
}
