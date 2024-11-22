#include "esp_log.h"
#include "tmp102_i2c.h"

/** I2C master configurations */
#define I2C_MASTER_NUM I2C_NUM_0  /**< I2C master port number */
#define I2C_MASTER_FREQ_HZ 100000 /**< I2C master clock frequency in Hz */
#define I2C_TIMEOUT_MS 1000       /**< I2C operation timeout in milliseconds */
#define I2C_PULLUP_ENABLE 1       /**< Enable internal pull-ups for SDA and SCL lines */
#define I2C_GLITCH_IGNORE_CNT 7   /**< Glitch filter count for noise suppression */

static const char *TAG = "TMP102_I2C";

static i2c_master_bus_handle_t bus_handle; /**< Handle for the I2C master bus */
static i2c_master_dev_handle_t dev_handle; /**< Handle to the TMP102 device on the I2C bus */

esp_err_t TMP102_I2C_Init(uint8_t addr, int pin_sda, int pin_scl)
{
   if (pin_scl < 0 || pin_sda < 0)
   {
      ESP_LOGE(TAG, "Invalid GPIO pins: SDA=%d, SCL=%d",
               pin_sda, pin_scl);
      return ESP_ERR_INVALID_ARG;
   }

   if (addr == 0)
   {
      ESP_LOGE(TAG, "Invalid TMP102 address: 0x%02x", addr);
      return ESP_ERR_INVALID_ARG;
   }

   esp_err_t ret;

   i2c_master_bus_config_t i2c_mst_conf = {
       .clk_source = I2C_CLK_SRC_DEFAULT,                 /**< Use the default clock source for I2C */
       .i2c_port = I2C_MASTER_NUM,                        /**< Use the defined I2C master port number */
       .scl_io_num = pin_scl,                             /**< Assign the SCL pin from the configuration */
       .sda_io_num = pin_sda,                             /**< Assign the SDA pin from the configuration */
       .glitch_ignore_cnt = I2C_GLITCH_IGNORE_CNT,        /**< Set the glitch filter count for noise reduction */
       .flags.enable_internal_pullup = I2C_PULLUP_ENABLE, /**< Enable internal pull-ups on I2C lines */
   };

   ret = i2c_new_master_bus(&i2c_mst_conf, &bus_handle);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "Failed to install I2C master bus on port %d: %s",
               I2C_MASTER_NUM, esp_err_to_name(ret));
      return ret;
   }

   ESP_LOGI(TAG, "I2C master bus initialized successfully on SDA=%d, SCL=%d",
            pin_sda, pin_scl);

   i2c_device_config_t dev_conf = {
       .dev_addr_length = I2C_ADDR_BIT_7,  /**< Set the device address length to 7 bits */
       .device_address = addr,             /**< Assign the TMP102 device address */
       .scl_speed_hz = I2C_MASTER_FREQ_HZ, /**< Set the I2C clock frequency */
   };

   ret = i2c_master_bus_add_device(bus_handle, &dev_conf, &dev_handle);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "Failed to install TMP102 device: %s", esp_err_to_name(ret));
      i2c_del_master_bus(bus_handle);
      return ret;
   }

   return ESP_OK;
}

esp_err_t TMP102_I2C_Read(uint8_t reg, uint8_t *data)
{
   if (data == NULL)
   {
      ESP_LOGE(TAG, "TMP102_I2C_Read: Invalid argument (NULL pointer)");
      return ESP_ERR_INVALID_ARG;
   }

   esp_err_t ret;
   uint8_t tx_buffer[1] = {reg}; /**< Transmission buffer to specify the register address */

   /* Perform an I2C transmit-receive operation */
   ret = i2c_master_transmit_receive(dev_handle, tx_buffer, 1, data, 1, I2C_TIMEOUT_MS);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "TMP102_I2C_Read: Failed reg:0x%02x, ret:%s", reg, esp_err_to_name(ret));
      return ret;
   }

   return ESP_OK;
}

esp_err_t TMP102_I2C_Write(uint8_t reg, uint8_t data)
{
   esp_err_t ret;
   uint8_t tx_buffer[2]; /**< Transmission buffer to hold the register address and data */
   tx_buffer[0] = reg;   /**< Set the register address */
   tx_buffer[1] = data;  /**< Set the data to be written */

   /* Perform an I2C write operation */
   ret = i2c_master_transmit(dev_handle, tx_buffer, 2, I2C_TIMEOUT_MS);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "TMP102_I2C_Write: Failed reg:0x%02x, data:0x%02x, ret:%s",
               reg, data, esp_err_to_name(ret));

      return ret;
   }

   return ESP_OK;
}

esp_err_t TMP102_I2C_Read16(uint8_t reg, uint16_t *data)
{
   if (data == NULL)
   {
      ESP_LOGE(TAG, "TMP102_I2C_Read16: Invalid argument (NULL pointer)");
      return ESP_ERR_INVALID_ARG;
   }

   esp_err_t ret;
   uint8_t tx_buffer[1] = {reg}; /**< Transmission buffer to specify the register address */
   uint8_t rx_buffer[2] = {0};   /**< Reception buffer to hold the read data */

   /* Perform an I2C transmit-receive operation */
   ret = i2c_master_transmit_receive(dev_handle, tx_buffer, 1, rx_buffer, 2, I2C_TIMEOUT_MS);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "TMP102_I2C_Read16: Failed reg:0x%02x, ret:%s", reg, esp_err_to_name(ret));
      return ret;
   }

   /* Combine the two bytes into a 16-bit value */
   *data = ((uint16_t)rx_buffer[0] << 8) | ((uint16_t)rx_buffer[1]);
   return ESP_OK;
}

esp_err_t TMP102_I2C_Write16(uint8_t reg, uint16_t data)
{
   esp_err_t ret;
   uint8_t tx_buffer[3];         /**< Transmission buffer to hold the register address and data */
   tx_buffer[0] = reg;           /**< Set the register address */
   tx_buffer[1] = data >> 8;     /**< Set the upper byte of the data */
   tx_buffer[2] = data & 0x00FF; /**< Set the lower byte of the data */

   // Perform an I2C write operation
   ret = i2c_master_transmit(dev_handle, tx_buffer, 3, I2C_TIMEOUT_MS);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "TMP102_I2C_Write16: Failed reg:0x%02x, data:0x%02x, ret:%s",
               reg, data, esp_err_to_name(ret));

      return ret;
   }

   return ESP_OK;
}