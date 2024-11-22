#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "tmp102.h"

static const char *TAG = "tmp102_example";

void app_main(void)
{
    /* Initialize the TMP102 sensor configuration */
    tmp102_config_t conf = {
        .addr = 0x48,                /**< Set the I2C address of the TMP102 sensor (0x48) */
        .pin_scl = GPIO_NUM_14,      /**< Set the GPIO pin for I2C clock (SCL) to GPIO 14 */
        .pin_sda = GPIO_NUM_13,      /**< Set the GPIO pin for I2C data (SDA) to GPIO 13 */
    };

    /* Initialize the TMP102 sensor with the configuration */
    ESP_ERROR_CHECK(tmp102_init(&conf));

    /* Put the TMP102 sensor to sleep mode to save power */
    ESP_ERROR_CHECK(tmp102_sleep());

    while (1)
    {
        /* Start a one-shot temperature measurement by triggering the TMP102 sensor */
        tmp102_one_shot(1);

        while (tmp102_one_shot(0) == 0) /**< Wait for conversion to be ready */
        {
            /* Read the temperature from the TMP102 sensor in Celsius */
            float temp = tmp102_read_temp_c();
            ESP_LOGI(TAG, "Temp in Celcius: %.2f", temp);
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}