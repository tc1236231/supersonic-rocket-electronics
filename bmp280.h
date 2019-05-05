#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11 
#define BMP_CS 8

#include <WString.h>

class BMP280
{
  public:
    BMP280(float seaLevelhPa);
    void init();
    String collectData();
  private:
    float seaLevelhPa;
};
