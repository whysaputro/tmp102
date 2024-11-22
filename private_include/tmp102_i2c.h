#ifndef __TMP102_I2C_H__
#define __TMP102_I2C_H__

#include "esp_err.h"
#include "driver/i2c_master.h"

esp_err_t TMP102_I2C_Init(uint8_t addr, int pin_sda, int pin_scl);
esp_err_t TMP102_I2C_Read(uint8_t reg, uint8_t *data);
esp_err_t TMP102_I2C_Write(uint8_t reg, uint8_t data);
esp_err_t TMP102_I2C_Read16(uint8_t reg, uint16_t *data);
esp_err_t TMP102_I2C_Write16(uint8_t reg, uint16_t data);;

#endif