# TMP102 Driver Library for ESP-IDF

[![Component Registry](https://components.espressif.com/components/whysaputro/tmp102/badge.svg)](https://components.espressif.com/components/whysaputro/tmp102)

A comprehensive driver library for the TMP102 temperature sensor with support for ESP-IDF. This library allows you to interact with the TMP102 sensor to read temperature data, configure thresholds, and adjust operational settings.

## Features

- **Temperature Readings**
  - Get temperature in Celsius or Fahrenheit.
- **Threshold Configuration**
  - Set and read high and low temperature thresholds in Celsius or Fahrenheit.
- **Power Management**
  - Sleep and wake-up functionality.
- **One-Shot Mode**
  - Perform single measurements on demand.
- **Alert Pin Configuration**
  - Set alert mode, polarity, and fault queue settings.
- **Extended Features**
  - Configure extended mode (12-bit or 13-bit resolution) and conversion rates.

## Installation

To add the TMP102 component to your ESP-IDF project, use the command `add-dependency` inside your project folder.

```bash
idf.py add-dependency "whysaputro/tmp102"
```

## Getting Started

### Include the Header
Include the `tmp102.h` into your code.
```c
#include "tmp102.h"
```

### Initialize the Sensor
Initialize TMP102 sensor with the desired configuration:
```c
#define TMP102_SDA_GPIO 17
#define TMP102_SCL_GPIO 18
#define TMP102_ADDR 0x48

tmp102_config_t config = {
    .sda_io_num = TMP102_SDA_GPIO,
    .scl_io_num = TMP102_SCL_GPIO,
    .addr = TMP102_ADDR
};

if (tmp102_init(&config) == ESP_OK) {
    ESP_LOGI("TMP102", "Sensor initialized successfully");
} else {
    ESP_LOGE("TMP102", "Failed to initialize TMP102");
}
```

### Read Temperature
Read temperature values in Celsius or Fahrenheit:
```c
float temp_c = tmp102_read_temp_c();
float temp_f = tmp102_read_temp_f();

ESP_LOGI("TMP102", "Temperature: %.2f °C, %.2f °F", temp_c, temp_f);
```

### Configure Thresholds
Set high and low temperature thresholds:
```c
tmp102_set_high_temp_c(50.0);  // Set high threshold to 50°C
tmp102_set_low_temp_c(10.0);   // Set low threshold to 10°C
```

Read configured thresholds:
```c
float high_threshold = tmp102_read_high_temp_c();
float low_threshold = tmp102_read_low_temp_c();

ESP_LOGI("TMP102", "High Threshold: %.2f °C, Low Threshold: %.2f °C", high_threshold, low_threshold);
```

### Power Management
Put the sensor to sleep or wake it up:
```c
tmp102_sleep();
tmp102_wakeup();
```

### One-Shot Mode
Enable one-shot mode for a single measurement:
```c
if (tmp102_one_shot(true)) {
    ESP_LOGI("TMP102", "One-shot mode enabled");
}
```

### Alert Pin Configuration
Set alert polarity and mode:
```c
tmp102_set_alert_polarity(true);    // Set alert pin to active-high
tmp102_set_alert_mode(0);          // Set alert mode to comparator
```

Configure fault queue:
```c
tmp102_set_fault(2);  // Set fault queue to 4 faults
```

### Extended Features
Set extended mode and conversion rate:
```c
tmp102_set_extended_mode(1);  // Enable 13-bit resolution
tmp102_set_conversion_rate(2); // Set conversion rate to 4Hz
```

## API Reference

### Initialization
```c  
esp_err_t tmp102_init(tmp102_config_t *config)
```

### Temperature Readings
```c
float tmp102_read_temp_c(void)
```
```c
float tmp102_read_temp_f(void)
```

### Threshold Configuration
```c
esp_err_t tmp102_set_low_temp_c(float temperature);
```
```c
esp_err_t tmp102_set_high_temp_c(float temperature)
```
```c
float tmp102_read_low_temp_c(void)
```
```c
float tmp102_read_high_temp_c(void)
```
```c
float tmp102_read_high_temp_f(void)
```
```c
float tmp102_low_high_temp_f(void)
```

### Power Management
```c
esp_err_t tmp102_sleep(void)
```
```c
esp_err_t tmp102_wakeup(void)
```

### Alert Pin Configuration
```c
bool tmp102_read_alert_status(void)
```
```c
esp_err_t tmp102_set_alert_polarity(bool polarity)
```
```c
esp_err_t tmp102_set_alert_mode(tmp102_alert_mode_t mode)
```
```c
esp_err_t tmp102_set_fault(uint8_t fault_setting)
```

### Extended Features
```c
esp_err_t tmp102_set_conversion_rate(uint8_t rate)
```
```c
esp_err_t tmp102_set_extended_mode(tmp102_extended_mode_t mode)
```

## Examples

Check the examples **folder** for complete usage samples.

## Resources

- [TMP102 Datasheet](https://www.ti.com/lit/ds/symlink/tmp102.pdf)