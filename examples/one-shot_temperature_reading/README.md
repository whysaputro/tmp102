# TMP102 One-Shot Temperature Measurement Example

This example demonstrates how to use the TMP102 temperature sensor in one-shot mode with the ESP32 using the ESP-IDF framework. The application initializes the TMP102 sensor, triggers a one-shot temperature measurement, and logs the temperature readings.

## Features

- **Initialization**: Configures the TMP102 sensor for I2C communication.
- **One-Shot Mode**: Triggers single temperature measurements to save power.
- **Power Management**: Places the TMP102 sensor in sleep mode between measurements.

## Hardware Requirements

- **ESP32 Development Board**
- **TMP102 Temperature Sensor**
- **I2C Pins**:
  - SCL connected to GPIO 14
  - SDA connected to GPIO 13

## Connections

| TMP102 Pin | ESP32 Pin       | Description          |
|------------|-----------------|----------------------|
| VCC        | 3.3V           | Power supply         |
| GND        | GND            | Ground              |
| SCL        | GPIO 14        | I2C Clock Line       |
| SDA        | GPIO 13        | I2C Data Line        |

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

static const char *TAG = "tmp102_example";

void app_main(void)
{
    tmp102_config_t conf = {
        .addr = 0x48,
        .pin_scl = GPIO_NUM_14,
        .pin_sda = GPIO_NUM_13,
    };

    ESP_ERROR_CHECK(tmp102_init(&conf));
    ESP_ERROR_CHECK(tmp102_sleep());

    while (1)
    {
        tmp102_one_shot(1);

        while (tmp102_one_shot(0) == 0)
        {
            float temp = tmp102_read_temp_c();
            ESP_LOGI(TAG, "Temp in Celsius: %.2f", temp);
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
