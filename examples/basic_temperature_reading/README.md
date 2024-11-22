# TMP102 Basic Temperature Reading Example

This example demonstrates how to use the TMP102 temperature sensor with the ESP32 using the ESP-IDF framework. The application initializes the TMP102 sensor, configures it, and periodically logs temperature readings along with alert pin and register states.

## Features

- **Initialization**: Configures the TMP102 sensor and GPIO for the ALERT pin.
- **Temperature Reading**: Reads temperature in Celsius.
- **Alert Monitoring**: Reads ALERT pin state and internal alert register status.
- **Power Management**: Utilizes sleep and wakeup modes to conserve power.
- **Custom Settings**: Configures fault tolerance, alert polarity, conversion rate, extended mode, and temperature thresholds.

## Hardware Requirements

- **ESP32 Development Board**
- **TMP102 Temperature Sensor**
- **I2C Pins**:
  - SCL connected to GPIO 14
  - SDA connected to GPIO 13
- **ALERT Pin**:
  - Connected to GPIO 10 with a pull-up resistor.

## Connections

| TMP102 Pin | ESP32 Pin       | Description          |
|------------|-----------------|----------------------|
| VCC        | 3.3V           | Power supply         |
| GND        | GND            | Ground              |
| SCL        | GPIO 14        | I2C Clock Line       |
| SDA        | GPIO 13        | I2C Data Line        |
| ALERT      | GPIO 10        | Alert Output (input to ESP32) |

## Software Setup

### Prerequisites

- ESP-IDF v5.3 or later
- TMP102 library integrated into your project.

### Example Code Overview

```c
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "tmp102.h"

#define ALERT_PIN GPIO_NUM_10 
#define T_HIGH_C 32.0
#define T_LOW_C 30.0 

static const char *TAG = "tmp102_example";

void app_main(void)
{
    tmp102_config_t conf = {
        .addr = 0x48,
        .pin_scl = GPIO_NUM_14,
        .pin_sda = GPIO_NUM_13,
    };

    ESP_ERROR_CHECK(tmp102_init(&conf));

    gpio_config_t gpio_conf = {
        .pin_bit_mask = (1ULL << ALERT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&gpio_conf));

    ESP_ERROR_CHECK(tmp102_set_fault(0));
    ESP_ERROR_CHECK(tmp102_set_alert_polarity(1));
    ESP_ERROR_CHECK(tmp102_set_alert_mode(TMP102_COMPARATOR_MODE));
    ESP_ERROR_CHECK(tmp102_set_conversion_rate(2));
    ESP_ERROR_CHECK(tmp102_set_extended_mode(TMP102_EXTENDED_DISABLE));
    ESP_ERROR_CHECK(tmp102_set_high_temp_c(T_HIGH_C));
    ESP_ERROR_CHECK(tmp102_set_low_temp_c(T_LOW_C));

    while (1)
    {
        float temp = 0;
        bool alert_pin_state = 0;
        bool alert_register_state = 0;

        ESP_ERROR_CHECK(tmp102_wakeup());

        temp = tmp102_read_temp_c();
        alert_pin_state = gpio_get_level(ALERT_PIN);
        alert_register_state = tmp102_read_alert_status();

        ESP_ERROR_CHECK(tmp102_sleep());

        ESP_LOGI(TAG, "Temp in Celsius: %.2f", temp);
        ESP_LOGI(TAG, "Alert Pin: %d", alert_pin_state);
        ESP_LOGI(TAG, "Alert Register: %d", alert_register_state);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}****