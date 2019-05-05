#include "i3g4250d_reg.h"

#define GYRO_SCK 13
#define GYRO_MISO 12
#define GYRO_MOSI 11 
#define GYRO_CS 6

#include <WString.h>

class i3g4250d
{
  public:
    void init();
    String collectData();
    static int32_t platform_write(void *handle, uint8_t reg_address, uint8_t *data,
                              uint16_t len);
    static int32_t platform_read(void *handle, uint8_t reg_address, uint8_t *data,
                             uint16_t len);
    void tx_com(uint8_t *tx_buffer, uint16_t len);
    void platform_init(void);                        
};
