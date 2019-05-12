#include "bmp280.h"
#include "SparkFun_LIS331.h"
#include "i3g4250d.h"
#include "DataRecorder.h"

BMP280 bmp(1013.25); //!!! Sea Level Pressure For the Day
LIS331 acc;
i3g4250d gyro;
DataRecorder recorder;

void setup() {
  Serial.begin(115200);
  //Order seems to matter
  gyro.init();
  bmp.init();
  acc.init();
  recorder.init();

}

void loop() {
  static long loopTimer = 0;
  if (millis() - loopTimer >= 100)
  {
    loopTimer = millis();
    String bmp_data = bmp.collectData();
    String acc_data = acc.collectData();
    String gyro_data = gyro.collectData();
    String total_data = (String)loopTimer;
    total_data += ',';
    total_data += bmp_data;
    total_data += ',';
    total_data += gyro_data;
    total_data += ',';
    total_data += acc_data;

    Serial.println(total_data);

    recorder.writeData(loopTimer, total_data);
  }
}
