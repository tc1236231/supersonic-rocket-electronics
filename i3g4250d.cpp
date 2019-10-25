#include "i3g4250d.h"
#include <SPI.h>
#include <Wire.h>

i3g4250d_ctx_t dev_ctx;

void i3g4250d::init() {  
    this->platform_init();
    
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    //  dev_ctx.handle = &hi2c1;
    
    uint8_t whoamI;
    
    i3g4250d_device_id_get(&dev_ctx, &whoamI);
    while(whoamI != I3G4250D_ID){
        Serial.println("Could not find a valid i3g4250d sensor, check wiring!");
        delay(1000);
        i3g4250d_device_id_get(&dev_ctx, &whoamI);
    }

    i3g4250d_fifo_mode_set(&dev_ctx, I3G4250D_FIFO_STREAM_MODE);

    i3g4250d_fifo_enable_set(&dev_ctx, PROPERTY_ENABLE);
  
    i3g4250d_data_rate_set(&dev_ctx, I3G4250D_ODR_100Hz);
}
  
String i3g4250d::collectData() {
    uint8_t flags;
    uint8_t num = 0;
    
    axis3bit16_t data_raw_angular_rate;
    float angular_rate_mdps[3];
    /*
     * Read watermark interrupt flag
     */
    i3g4250d_fifo_wtm_flag_get(&dev_ctx, &flags);
    String output;
    //if (flags)
    {
      /*
       * Read how many samples in fifo
       */
      i3g4250d_fifo_data_level_get(&dev_ctx, &num);
      while (num-- > 0)
      {
        /* Read angular rate data */
        memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
        i3g4250d_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate.u8bit);
        angular_rate_mdps[0] = i3g4250d_from_fs245dps_to_mdps(data_raw_angular_rate.i16bit[0]);
        angular_rate_mdps[1] = i3g4250d_from_fs245dps_to_mdps(data_raw_angular_rate.i16bit[1]);
        angular_rate_mdps[2] = i3g4250d_from_fs245dps_to_mdps(data_raw_angular_rate.i16bit[2]);
      }

//      Serial.print(data_raw_angular_rate.i16bit[0]);
//      Serial.print("\t");
//      Serial.print(data_raw_angular_rate.i16bit[1]);
//      Serial.print("\t");
//      Serial.println(data_raw_angular_rate.i16bit[2]);
  
      output = data_raw_angular_rate.i16bit[0];
      output += ',';
      output += data_raw_angular_rate.i16bit[1];
      output += ',';
      output += data_raw_angular_rate.i16bit[2];
    }

    return output;
}

static int32_t i3g4250d::platform_write(void *handle, uint8_t reg_address, uint8_t *data,
                              uint16_t len)
{
    /* Write multiple command */
    digitalWrite(GYRO_CS, LOW);
    SPI.transfer(reg_address | 0x40);
    for (int i=0; i<len; i++)
    {
      SPI.transfer(data[i]);
    }
    digitalWrite(GYRO_CS, HIGH);
    
    return 0;
}

static int32_t i3g4250d::platform_read(void *handle, uint8_t reg_address, uint8_t *data,
                             uint16_t len)
{
    /* Read multiple command */
    digitalWrite(GYRO_CS, LOW);
    SPI.transfer(reg_address | 0xC0);
    for (int i=0; i<len; i++)
    {
      data[i] = SPI.transfer(0);
    }
    digitalWrite(GYRO_CS, HIGH);;
    
    return 0;
}

void i3g4250d::tx_com(uint8_t *tx_buffer, uint16_t len)
{
  /*
    #ifdef NUCLEO_F411RE_X_NUCLEO_IKS01A2
      HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
    #endif
    #ifdef STEVAL_MKI109V3
      CDC_Transmit_FS(tx_buffer, len);
    #endif
  */
}

void i3g4250d::platform_init(void)
{
    pinMode(GYRO_CS, OUTPUT);    // CS for SPI
    digitalWrite(GYRO_CS, HIGH); // Make CS high
    pinMode(GYRO_MOSI, OUTPUT);    // MOSI for SPI
    pinMode(GYRO_MISO, INPUT);     // MISO for SPI
    pinMode(GYRO_SCK, OUTPUT);    // SCK for SPI
    SPI.begin();
}
