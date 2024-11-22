#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "tmp102.h"

#define ALERT_PIN GPIO_NUM_10    /**< Define the GPIO pin number (10) used for the ALERT signal from TMP102 */

#define T_HIGH_C 32.0       /**< Define the high temperature threshold (T_HIGH) in Celsius (32°C) */
#define T_LOW_C 30.0        /**< Define the low temperature threshold (T_LOW) in Celsius (30°C) */

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

    /* Configure the GPIO pin for the ALERT signal */
    gpio_config_t gpio_conf = {
        .pin_bit_mask = (1ULL << ALERT_PIN),   /**< Specify the ALERT pin (GPIO 10) */
        .mode = GPIO_MODE_INPUT,                    /**< Set the GPIO pin mode to input (for reading ALERT signal) */
        .pull_up_en = GPIO_PULLUP_ENABLE,           /**< Enable the internal pull-up resistor */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,      /**< Disable the internal pull-down resistor */
        .intr_type = GPIO_INTR_DISABLE,             /**< Disable interrupts for this pin */
    };

    /* Apply the GPIO pin configuration */
    ESP_ERROR_CHECK(gpio_config(&gpio_conf));

    /* Set TMP102 sensor settings */
    ESP_ERROR_CHECK(tmp102_set_fault(0));                               /**< Set the fault tolerance to 0 (1 tolerance) */
    ESP_ERROR_CHECK(tmp102_set_alert_polarity(1));                      /**< Set alert polarity to active HIGH */
    ESP_ERROR_CHECK(tmp102_set_alert_mode(TMP102_COMPARATOR_MODE));     /**< Set the alert mode to comparator */
    ESP_ERROR_CHECK(tmp102_set_conversion_rate(2));                     /**< Set the conversion rate */
    ESP_ERROR_CHECK(tmp102_set_extended_mode(TMP102_EXTENDED_DISABLE)); /**< Disable the extended mode */
    ESP_ERROR_CHECK(tmp102_set_high_temp_c(T_HIGH_C));                  /**< Set the high temperature threshold to 32°C */
    ESP_ERROR_CHECK(tmp102_set_low_temp_c(T_LOW_C));                    /**< Set the low temperature threshold to 30°C */

    while (1)
    {
        float temp = 0;
        bool alert_pin_state = 0;
        bool alert_register_state = 0;

        /* Wake up the TMP102 sensor to start temperature measurement */
        ESP_ERROR_CHECK(tmp102_wakeup());

        /* Read the temperature in Celsius from TMP102 */
        temp = tmp102_read_temp_c();

        /* Read the state of the ALERT pin */
        alert_pin_state = gpio_get_level(ALERT_PIN);

        /* Read the alert status from TMP102's internal register */
        alert_register_state = tmp102_read_alert_status();

        /* Put the TMP102 sensor to sleep mode to save power */
        ESP_ERROR_CHECK(tmp102_sleep());

        /* Log the temperature, alert pin state, and alert register state */
        ESP_LOGI(TAG, "Temp in Celcius: %.2f", temp);
        ESP_LOGI(TAG, "Alert Pin: %d", alert_pin_state);
        ESP_LOGI(TAG, "Alert Register: %d", alert_register_state);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}