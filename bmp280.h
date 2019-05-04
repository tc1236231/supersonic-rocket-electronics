#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11 
#define BMP_CS 8

class BMP280
{
  public:
    BMP280(float seaLevelhPa);
    void init();
    char* collectData();
  private:
    float seaLevelhPa;
};
