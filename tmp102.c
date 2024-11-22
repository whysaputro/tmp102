/**
 * @file tmp102.c
 * @brief Driver for the TMP102 temperature sensor using ESP-IDF framework.
 *
 * This source file provides functions for initializing and interacting with the TMP102 sensor,
 * including reading temperature, configuring alerts, and adjusting various settings.
 *
 * This code is adapted from the SparkFun TMP102 Arduino Library. For details, see @ref sparkfun_library.
 *
 * @author Hendra Wahyu <hendra.wahyu.s26@gmail.com>
 * @date November 9, 2024
 *
 * @ref sparkfun_library https://github.com/sparkfun/SparkFun_TMP102_Arduino_Library
 */

#include <math.h>

#include "esp_log.h"
#include "tmp102_i2c.h"
#include "tmp102.h"

static const char *TAG = "TMP102";

esp_err_t tmp102_init(tmp102_config_t *config)
{
   if (config == NULL)
   {
      ESP_LOGE(TAG, "tmp102_init: Invalid argument (NULL pointer)");
      return ESP_ERR_INVALID_ARG;
   }

   esp_err_t ret = TMP102_I2C_Init(config->addr, config->pin_sda, config->pin_scl);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "tmp102_init: Failed initialize sensor: %s", esp_err_to_name(ret));
      return ret;
   }

   return ESP_OK;
}

float tmp102_read_temp_c(void)
{
   esp_err_t ret;
   uint16_t register_data = 0; /**< Variable to hold the raw 16-bit data from the sensor */
   int16_t digital_temp = 0;   /**< Variable to hold the processed temperature value */
   bool extended_mode = false; /**< Flag to indicate if the sensor is in extended mode (13-bit) */

   /* Read 16-bit data from the temperature register of the TMP102 */
   ret = TMP102_I2C_Read16(TMP102_TEMP_REG, &register_data);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "tmp102_read_temp_c: Failed to read temperature: %s", esp_err_to_name(ret));
      return NAN;
   }

   /* Determine if the sensor is in extended mode */
   extended_mode = register_data & 0x0001; /**< Check the LSB to determine mode */
                                           /**< 0 - 12-bit temperature data */
                                           /**< 1 - 13-bit temperature data */

   if (extended_mode)
   {
      digital_temp = register_data >> 3; /**< Extract the 13-bit temperature data */
      if (digital_temp & 0x1000)         /**< Check if the temperature is negative */
      {
         digital_temp |= 0xE000; /**< Sign-extend the 13-bit value to 16 bits */
      }
   }
   else
   {
      digital_temp = register_data >> 4; /**< Extract the 12-bit temperature data */
      if (digital_temp & 0x0800)         /**< Check if the temperature is negative */
      {
         digital_temp |= 0xF000; /**< Sign-extend the 12-bit value to 16 bits */
      }
   }

   /* Convert the digital temperature value to Celsius */
   return digital_temp * 0.0625f;
}

float tmp102_read_temp_f(void)
{
   const float conversion_factor = 9.0 / 5.0; /**< Conversion factor for Celsius to Fahrenheit */
   float temp_c = 0;                          /**< Variable to hold the temperature in Celsius */

   /* Read the temperature in Celsius */
   temp_c = tmp102_read_temp_c();
   if (isnan(temp_c))
   {
      return NAN;
   }

   /* Convert Celsius to Fahrenheit and return the result */
   return (temp_c * conversion_factor) + 32.0;
}

esp_err_t tmp102_sleep(void)
{
   esp_err_t ret;
   uint8_t register_data = 0; /**< Variable to hold the configuration register value */

   /* Read the current configuration register value */
   ret = TMP102_I2C_Read(TMP102_CONFIG_REG, &register_data);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "tmp102_sleep: Failed to read config: %s", esp_err_to_name(ret));
      return ret;
   }

   /* Set the sleep bit (bit 0) in the configuration register */
   register_data |= 0x01;

   /* Write the updated configuration back to the register */
   ret = TMP102_I2C_Write(TMP102_CONFIG_REG, register_data);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "tmp102_sleep: Failed to entered sleep mode: %s", esp_err_to_name(ret));
      return ret;
   }

   return ESP_OK;
}

esp_err_t tmp102_wakeup(void)
{
   esp_err_t ret;
   uint8_t register_data = 0; /**< Variable to hold the configuration register value */

   /* Read the current configuration register value */
   ret = TMP102_I2C_Read(TMP102_CONFIG_REG, &register_data);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "tmp102_wakeup: Failed to read config: %s", esp_err_to_name(ret));
      return ret;
   }

   /* Clear the sleep bit (bit 0) in the configuration register */
   register_data &= 0xFE;

   /* Write the updated configuration back to the register */
   ret = TMP102_I2C_Write(TMP102_CONFIG_REG, register_data);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "tmp102_wakeup: Failed to wake up TMP102: %s", esp_err_to_name(ret));
      return ret;
   }

   return ESP_OK;
}

bool tmp102_read_alert_status(void)
{
   esp_err_t ret;
   uint16_t register_data = 0; /**< Variable to hold the configuration register value */

   /* Read the configuration register (16 bits) */
   ret = TMP102_I2C_Read16(TMP102_CONFIG_REG, &register_data);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "tmp102_alert: Failed to read config: %s", esp_err_to_name(ret));
      return false;
   }

   return (register_data & 0x0020) >> 5;
}

bool tmp102_one_shot(bool set)
{
   esp_err_t ret;
   uint8_t register_data = 0; /**< Variable to hold the configuration register value */

   /* Read the current configuration register value */
   ret = TMP102_I2C_Read(TMP102_CONFIG_REG, &register_data);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "tmp102_one_shot: Failed to read config: %s", esp_err_to_name(ret));
      return false;
   }

   if (set) /**< If enabling one-shot mode */
   {
      /* Set the one-shot bit (bit 7) */
      register_data |= (1 << 7);

      /* Write the updated configuration back to the register */
      esp_err_t ret = TMP102_I2C_Write(TMP102_CONFIG_REG, register_data);
      if (ret != ESP_OK)
      {
         ESP_LOGE(TAG, "tmp102_one_shot: Failed to set one-shot mode: %s", esp_err_to_name(ret));
         return ret;
      }

      ESP_LOGI(TAG, "One-shot mode set successfully");
      return true; /**< Indicate success */
   }

   /* Check if the one-shot bit (bit 7) is set */
   return ((register_data & 0x80) >> 7) != 0;
}

esp_err_t tmp102_set_low_temp_c(float temperature)
{
   esp_err_t ret;
   uint16_t register_data = 0; /**< Variable to hold the configuration register value */
   bool extended_mode = false; /**< Indicates whether extended (13-bit) mode is enabled */

   /* Clamp the temperature to the allowable range (-55.0°C to 150.0°C) */
   if (temperature > 150.0f)
      temperature = 150.0f;
   if (temperature < -55.0f)
      temperature = -55.0f;

   /* Read the current configuration register value */
   ret = TMP102_I2C_Read16(TMP102_CONFIG_REG, &register_data);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "tmp102_set_low_temp_c: Failed to read config: %s", esp_err_to_name(ret));
      return ret;
   }

   /* Determine the resolution mode (12-bit or 13-bit) */
   extended_mode = (register_data & 0x0010) != 0; /**< 0 - 12-bit temperature data */
                                                  /**< 1 - 13-bit temperature data */

   /* convert temperature from analog to digital*/
   temperature /= 0.0625f;

   if (extended_mode)
   {
      register_data = (int)temperature << 3; /**< 13-bit */
   }
   else
   {
      register_data = (int)temperature << 4; /**< 12-bit */
   }

   /* Write the calculated value to the TMP102's low-temperature register */
   ret = TMP102_I2C_Write16(TMP102_T_LOW_REG, register_data);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "tmp102_set_low_temp_c: Failed to set low temperature threshold: %s", esp_err_to_name(ret));
      return ret;
   }

   ESP_LOGI(TAG, "Successfully set low temperature threshold to %.2f°C", temperature * 0.0625f);
   return ESP_OK;
}

esp_err_t tmp102_set_high_temp_c(float temperature)
{
   esp_err_t ret;
   uint16_t register_data = 0; /**< Variable to hold the configuration register value */
   bool extended_mode = false; /**< Indicates whether extended (13-bit) mode is enabled */

   /* Clamp the temperature to the allowable range (-55.0°C to 150.0°C) */
   if (temperature > 150.0f)
      temperature = 150.0f;
   if (temperature < -55.0f)
      temperature = -55.0f;

   /* Read the current configuration register value */
   ret = TMP102_I2C_Read16(TMP102_CONFIG_REG, &register_data);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "tmp102_set_high_temp_c: Failed to read config: %s", esp_err_to_name(ret));
      return ret;
   }

   /* Determine the resolution mode (12-bit or 13-bit) */
   extended_mode = (register_data & 0x0010) != 0; /**< 0 - 12-bit temperature data */
                                                  /**< 1 - 13-bit temperature data */

   /* convert temperature from analog to digital*/
   temperature /= 0.0625f;

   if (extended_mode)
   {
      register_data = (int)temperature << 3; /**< 13-bit */
   }
   else
   {
      register_data = (int)temperature << 4; /**< 12-bit */
   }

   /* Write the calculated value to the TMP102's high-temperature register */
   ret = TMP102_I2C_Write16(TMP102_T_HIGH_REG, register_data);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "tmp102_set_high_temp_c: Failed to set high temperature threshold: %s", esp_err_to_name(ret));
      return ret;
   }

   ESP_LOGI(TAG, "Successfully set high temperature threshold to %.2f°C", temperature * 0.0625f);
   return ESP_OK;
}

esp_err_t tmp102_set_low_temp_f(float temperature)
{
   esp_err_t ret;

   /* Convert Fahrenheit to Celsius */
   temperature = (temperature - 32) * 5 / 9;

   ret = tmp102_set_low_temp_c(temperature);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "tmp102_set_low_temp_f: Failed to set low temperature threshold: %s", esp_err_to_name(ret));
      return ret;
   }

   return ESP_OK;
}

esp_err_t tmp102_set_high_temp_f(float temperature)
{
   esp_err_t ret;

   /* Convert Fahrenheit to Celsius */
   temperature = (temperature - 32) * 5 / 9;

   ret = tmp102_set_high_temp_c(temperature);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "tmp102_set_high_temp_f: Failed to set high temperature threshold: %s", esp_err_to_name(ret));
      return ret;
   }

   return ESP_OK;
}

float tmp102_read_low_temp_c(void)
{
   esp_err_t ret;
   uint16_t register_data = 0; /**< Variable to hold the register data */
   int16_t digital_temp = 0;   /**< Holds the digital representation of temperature */
   bool extended_mode = false; /**< Indicates whether extended (13-bit) mode is enabled */

   /* Read the configuration register to determine the extented mode */
   ret = TMP102_I2C_Read16(TMP102_CONFIG_REG, &register_data);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "tmp102_read_low_temp_c: Failed to read config: %s", esp_err_to_name(ret));
      return NAN;
   }

   /* Check the extended mode bit (bit 4) */
   extended_mode = register_data & 0x0010; /**< 0 - 12-bit temperature data */
                                           /**< 1 - 13-bit temperature data */

   /* Read the low-temperature threshold register */
   ret = TMP102_I2C_Read16(TMP102_T_LOW_REG, &register_data);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "tmp102_read_low_temp_c: Failed to read low temperature threshold: %s", esp_err_to_name(ret));
      return NAN;
   }

   if (extended_mode)
   {
      digital_temp = register_data >> 3; /**< Extract the 13-bit temperature data */
      if (digital_temp & 0x1000)         /**< Check if the temperature is negative */
      {
         digital_temp |= 0xE000; /**< Sign-extend the 13-bit value to 16 bits */
      }
   }
   else
   {
      digital_temp = register_data >> 4; /**< Extract the 12-bit temperature data */
      if (digital_temp & 0x0800)         /**< Check if the temperature is negative */
      {
         digital_temp |= 0xF000; /**< Sign-extend the 12-bit value to 16 bits */
      }
   }

   /* Convert digital value to Celsius and return */
   return digital_temp * 0.0625f;
}

float tmp102_read_high_temp_c(void)
{
   esp_err_t ret;
   uint16_t register_data = 0; /**< Variable to hold the register data */
   int16_t digital_temp = 0;   /**< Holds the digital representation of temperature */
   bool extended_mode = false; /**< Indicates whether extended (13-bit) mode is enabled */

   /* Read the configuration register to determine the extented mode */
   ret = TMP102_I2C_Read16(TMP102_CONFIG_REG, &register_data);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "tmp102_read_high_temp_c: Failed to read config: %s", esp_err_to_name(ret));
      return NAN;
   }

   /* Check the extended mode bit (bit 4) */
   extended_mode = register_data & 0x0010; /**< 0 - 12-bit temperature data */
                                           /**< 1 - 13-bit temperature data */

   /* Read the low-temperature threshold register */
   ret = TMP102_I2C_Read16(TMP102_T_HIGH_REG, &register_data);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "tmp102_read_high_temp_c: Failed to read high temperature threshold: %s", esp_err_to_name(ret));
      return NAN;
   }

   if (extended_mode)
   {
      digital_temp = register_data >> 3; /**< Extract the 13-bit temperature data */
      if (digital_temp & 0x1000)         /**< Check if the temperature is negative */
      {
         digital_temp |= 0xE000; /**< Sign-extend the 13-bit value to 16 bits */
      }
   }
   else
   {
      digital_temp = register_data >> 4; /**< Extract the 13-bit temperature data */
      if (digital_temp & 0x0800)         /**< Check if the temperature is negative */
      {
         digital_temp |= 0xF000; /**< Sign-extend the 13-bit value to 16 bits */
      }
   }

   /* Convert digital value to Celsius and return */
   return digital_temp * 0.0625f;
}

float tmp102_read_low_temp_f(void)
{
   float temperature_c = 0;                   /**< Variable to hold the temperature in Celsius */
   const float conversion_factor = 9.0 / 5.0; /**< Conversion factor for Celsius to Fahrenheit */

   /* Read the low-temperature threshold in Celsius */
   temperature_c = tmp102_read_low_temp_c();
   if (isnan(temperature_c))
   {
      ESP_LOGE(TAG, "tmp102_read_low_temp_f: Failed to read low temperature threshold in Celcius");
      return NAN;
   }

   /* Convert to Fahrenheit and return */
   return (temperature_c * conversion_factor) + 32;
}

float tmp102_read_high_temp_f(void)
{
   float temperature_c = 0;                   /**< Variable to hold the temperature in Celsius */
   const float conversion_factor = 9.0 / 5.0; /**< Conversion factor for Celsius to Fahrenheit */

   /* Read the low-temperature threshold in Celsius */
   temperature_c = tmp102_read_high_temp_c();
   if (isnan(temperature_c))
   {
      ESP_LOGE(TAG, "tmp102_read_high_temp_f: Failed to read high temperature threshold in Celcius");
      return NAN;
   }

   /* Convert to Fahrenheit and return */
   return (temperature_c * conversion_factor) + 32;
}

esp_err_t tmp102_set_conversion_rate(uint8_t rate)
{
   /* Validate input range */
   if (rate > 3)
   {
      ESP_LOGE(TAG, "tmp102_set_conversion_rate: Invalid rate value: %d", rate);
      return ESP_ERR_INVALID_ARG;
   }

   esp_err_t ret;
   uint16_t register_data = 0; /**< Variable to hold the register data */

   /* Read current configuration register */
   ret = TMP102_I2C_Read16(TMP102_CONFIG_REG, &register_data);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "tmp102_set_conversion_rate: Failed to read config: %s", esp_err_to_name(ret));
      return ret;
   }

   /* Clear CR bits (bit 6 and 7) and set new rate */
   register_data = (register_data & 0xFF3F) | (rate << 6);

   /* Write updated configuration back */
   ret = TMP102_I2C_Write16(TMP102_CONFIG_REG, register_data);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "tmp102_set_conversion_rate: Failed to set conversion rate: %s", esp_err_to_name(ret));
      return ret;
   }

   ESP_LOGI(TAG, "Successfully set conversion rate to: %d (0: 0.25Hz, 1: 1Hz, 2: 4Hz, 3: 8Hz)", rate);
   return ESP_OK;
}

esp_err_t tmp102_set_extended_mode(tmp102_extended_mode_t mode)
{
   /* Validate input range */
   if (mode < 0 || mode > 3)
   {
      ESP_LOGE(TAG, "tmp102_set_extended_mode: Invalid fault_setting value: %d", mode);
      return ESP_ERR_INVALID_ARG;
   }

   esp_err_t ret;
   uint16_t register_data = 0; /**< Variable to hold the register data */

   /* Read current configuration register */
   ret = TMP102_I2C_Read16(TMP102_CONFIG_REG, &register_data);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "tmp102_set_extended_mode: Failed to read config: %s", esp_err_to_name(ret));
      return ret;
   }

   /* Clear EM bits (bit 4) and set extended mode */
   register_data = (register_data & 0xFFEF) | (mode << 4);

   /* Write updated configuration back */
   ret = TMP102_I2C_Write16(TMP102_CONFIG_REG, register_data);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "tmp102_set_extended_mode: Failed to set extended mode: %s", esp_err_to_name(ret));
      return ret;
   }

   ESP_LOGI(TAG, "Successfully set extended mode: %s", mode ? "ENABLE" : "DISABLE");
   return ESP_OK;
}

esp_err_t tmp102_set_alert_polarity(bool polarity)
{
   esp_err_t ret;
   uint8_t register_data = 0; /**< Variable to hold the register data */

   /* Read the current configuration register */
   ret = TMP102_I2C_Read(TMP102_CONFIG_REG, &register_data);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "tmp102_set_alert_polarity: Failed to read config: %s", esp_err_to_name(ret));
      return ret;
   }

   /* Update the register data to set the correct alert polarity */
   register_data = polarity ? (register_data | 0x04) : (register_data & ~0x04);

   /* Write the updated register data back to the configuration register */
   ret = TMP102_I2C_Write(TMP102_CONFIG_REG, register_data);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "tmp102_set_alert_polarity: Failed to set alert polarity: %s", esp_err_to_name(ret));
      return ret;
   }

   ESP_LOGI(TAG, "Alert polarity set to: Active %s", polarity ? "HIGH" : "LOW");
   return ESP_OK;
}

esp_err_t tmp102_set_fault(uint8_t fault_setting)
{
   /* Validate input range */
   if (fault_setting > 3)
   {
      ESP_LOGE(TAG, "tmp102_set_fault: Invalid fault_setting value: %d", fault_setting);
      return ESP_ERR_INVALID_ARG;
   }

   esp_err_t ret;
   uint8_t register_data = 0; /**< Variable to hold the register data */

   /* Read current configuration register */
   ret = TMP102_I2C_Read(TMP102_CONFIG_REG, &register_data);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "tmp102_set_fault: Failed to read config: %s", esp_err_to_name(ret));
      return ret;
   }

   /* Clear F0/1 bits (bit 3 and 4) and set number of consecutive faults */
   register_data = (register_data & 0xE7) | (fault_setting << 3);

   /* Write updated configuration back */
   ret = TMP102_I2C_Write(TMP102_CONFIG_REG, register_data);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "tmp102_set_fault: Failed to set fault: %s", esp_err_to_name(ret));
      return ret;
   }

   ESP_LOGI(TAG, "Successfully set conversion rate to: %d (0: 1 fault, 1: 2 fault, 2: 4 fault, 3: 6 fault)",
            fault_setting);
   return ESP_OK;
}

esp_err_t tmp102_set_alert_mode(tmp102_alert_mode_t mode)
{
   /* Validate input range */
   if (mode < 0 || mode > 1)
   {
      ESP_LOGE(TAG, "tmp102_set_alert_mode: Invalid fault_setting value: %d", mode);
      return ESP_ERR_INVALID_ARG;
   }

   esp_err_t ret;
   uint8_t register_data = 0; /**< Variable to hold the register data */

   /* Read current configuration register */
   ret = TMP102_I2C_Read(TMP102_CONFIG_REG, &register_data);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "tmp102_set_alert_mode: Failed to read config: %s", esp_err_to_name(ret));
      return ret;
   }

   /* Clear TM bits (bit 1) and set alert mode */
   register_data = (register_data & 0xFD) | (mode << 1);

   /* Write updated configuration back */
   ret = TMP102_I2C_Write(TMP102_CONFIG_REG, register_data);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "tmp102_set_alert_mode: Failed to set alert mode: %s", esp_err_to_name(ret));
      return ret;
   }

   ESP_LOGI(TAG, "Successfully set alert mode to: %d (0: Comparator, 1: Interrupt)", mode);
   return ESP_OK;
}