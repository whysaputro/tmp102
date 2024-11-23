/**
 * @file tmp102.h
 * @brief Driver for the TMP102 temperature sensor using ESP-IDF framework.
 *
 * This header file provides functions for initializing and interacting with the TMP102 sensor,
 * including reading temperature, configuring alerts, and adjusting various settings.
 *
 * This code is adapted from the SparkFun TMP102 Arduino Library. For details, see @ref sparkfun_library.
 *
 * @author Hendra Wahyu <hendra.wahyu.s26@gmail.com>
 * @date November 9, 2024
 *
 * @ref sparkfun_library https://github.com/sparkfun/SparkFun_TMP102_Arduino_Library
 */

#ifndef __TMP102_H__
#define __TMP102_H__

#include <stdbool.h>

#include "esp_err.h"

/**
 * @brief TMP102 configuration structure.
 *
 * This structure is used to configure the SDA and SCL pins, as well as the I2C address of the TMP102 sensor.
 */
typedef struct
{
   uint8_t addr; /*!< TMP102 address */
   int pin_scl;  /*!< GPIO pin for TMP102 SCL pin */
   int pin_sda;  /*!< GPIO pin for TMP102 SDA pin */
} tmp102_config_t;

/**
 * @brief Enumeration for TMP102 Alert Modes.
 *
 * This enum defines the possible modes for the TMP102 alert functionality.
 * The TMP102 sensor can operate in two modes: Comparator Mode or Thermostat Mode.
 *
 * @note In Comparator Mode, the alert pin is activated based on the temperature threshold.
 *       In Interrupt Mode, the alert pin is activated based on the temperature and the settings of high and low thresholds.
 */
typedef enum
{
   TMP102_COMPARATOR_MODE, /**< Comparator mode: The alert pin is asserted when the temperature crosses a threshold */
   TMP102_INTERRUPT_MODE, /**< Interrupt mode: The alert pin is asserted based on both high and low temperature thresholds */
} tmp102_alert_mode_t;

/**
 * @brief Enumeration for TMP102 Extended Mode.
 *
 * This enum defines whether the TMP102 operates in extended mode or not.
 * Extended mode allows for higher precision in temperature measurements (13-bit vs. 12-bit).
 *
 * @note In Extended Mode (13-bit), the temperature resolution is increased, but the update rate may be slower.
 */
typedef enum
{
   TMP102_EXTENDED_DISABLE, /**< Disable extended mode: Operates with 12-bit temperature resolution */
   TMP102_EXTENDED_ENABLE,  /**< Enable extended mode: Operates with 13-bit temperature resolution */
} tmp102_extended_mode_t;

/**
 * @brief Initialize the TMP102 device on the specified I2C bus.
 *
 * This function sets up the I2C bus and configures the TMP102 temperature sensor
 * for communication based on the provided configuration structure.
 *
 * @param[in] config Pointer to the TMP102 configuration structure containing I2C pins and address.
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_INVALID_ARG: Invalid argument (e.g., NULL pointer or invalid GPIO pins)
 *    - Other: Errors during I2C initialization or device configuration.
 */
esp_err_t tmp102_init(tmp102_config_t *config);

/**
 * @brief Reads the temperature from the TMP102 sensor in Celsius.
 *
 * This function reads the raw temperature data from the TMP102 sensor, processes it,
 * and converts it to a Celsius value based on the sensor's configuration.
 *
 * @return Temperature in degrees Celsius, or NaN if an error occurs.
 */
float tmp102_read_temp_c(void);

/**
 * @brief Reads the temperature from the TMP102 sensor in Fahrenheit.
 *
 * This function uses the `tmp102_read_temp_c` function to get the temperature in Celsius,
 * then converts it to Fahrenheit using the formula:
 * Fahrenheit = (Celsius * 9/5) + 32
 *
 * @return Temperature in degrees Fahrenheit, or NaN if an error occurs.
 */
float tmp102_read_temp_f(void);

/**
 * @brief Puts the TMP102 sensor into sleep mode.
 *
 * This function sets the sleep bit in the configuration register of the TMP102 sensor,
 * reducing its power consumption.
 *
 * @return
 * - `ESP_OK` on success.
 * - Error code otherwise.
 */
esp_err_t tmp102_sleep(void);

/**
 * @brief Wakes up the TMP102 sensor from sleep mode.
 *
 * This function clears the sleep bit in the configuration register of the TMP102 sensor,
 * allowing it to resume normal operation.
 *
 * @return
 * - `ESP_OK` on success.
 * - Error code otherwise.
 */
esp_err_t tmp102_wakeup(void);

/**
 * @brief Checks the alert status of the TMP102 sensor.
 *
 * This function reads the configuration register and checks if the alert bit (bit 5) is set,
 * indicating a triggered alert condition.
 *
 * @return 
 *  - `true` if the alert bit (bit 5) is set, indicating an active alert condition.
 *  - `false` if the alert bit is not set, indicating no alert condition.
 */
bool tmp102_read_alert_status(void);

/**
 * @brief Sets or checks the one-shot mode of the TMP102 sensor.
 *
 * In one-shot mode, the TMP102 takes a single temperature measurement when the mode is enabled.
 * This function can set the one-shot mode or query its status.
 *
 * @param[in]  set     Set to `true` to enable one-shot mode, `false` to query the status.
 * 
 * @return 
 *  - `true` if the one-shot mode is successfully set or the status is `true` (enabled).
 *  - `false` if the one-shot mode is not set, or the query indicates that it is not enabled.
 */
bool tmp102_one_shot(bool set);

/**
 * @brief Sets the low-temperature threshold in degrees Celsius for the TMP102 sensor.
 *
 * This function writes the specified low-temperature threshold to the TMP102's low-temperature register.
 * It ensures the value is within the allowable range and adapts to 12-bit or 13-bit resolution based on the
 * sensor's configuration.
 *
 * @param[in] temperature The low-temperature threshold in degrees Celsius (-55.0°C to 150.0°C).
 * @return
 * - `ESP_OK` on success.
 * - Error code otherwise.
 */
esp_err_t tmp102_set_low_temp_c(float temperature);

/**
 * @brief Sets the high-temperature threshold in degrees Celsius for the TMP102 sensor.
 *
 * This function writes the specified high-temperature threshold to the TMP102's high-temperature register.
 * It ensures the value is within the allowable range and adapts to 12-bit or 13-bit resolution based on the
 * sensor's configuration.
 *
 * @param[in] temperature The high-temperature threshold in degrees Celsius (-55.0°C to 150.0°C).
 * @return
 * - `ESP_OK` on success.
 * - Error code otherwise.
 */
esp_err_t tmp102_set_high_temp_c(float temperature);

/**
 * @brief Sets the low-temperature threshold in degrees Fahrenheit for the TMP102 sensor.
 *
 * This function converts the specified Fahrenheit temperature to Celsius and calls
 * `tmp102_set_low_temp_c` to update the low-temperature register.
 *
 * @param[in] temperature The low-temperature threshold in degrees Fahrenheit.
 * @return
 * - `ESP_OK` on success.
 * - Error code otherwise.
 */
esp_err_t tmp102_set_low_temp_f(float temperature);

/**
 * @brief Sets the high-temperature threshold in degrees Fahrenheit for the TMP102 sensor.
 *
 * This function converts the specified Fahrenheit temperature to Celsius and calls
 * `tmp102_set_high_temp_c` to update the high-temperature register.
 *
 * @param[in] temperature The high-temperature threshold in degrees Fahrenheit.
 * @return
 * - `ESP_OK` on success.
 * - Error code otherwise.
 */
esp_err_t tmp102_set_high_temp_f(float temperature);

/**
 * @brief Reads the low-temperature threshold in degrees Celsius from the TMP102 sensor.
 *
 * This function retrieves the low-temperature threshold stored in the TMP102's low-temperature register.
 * It adapts to the resolution mode (12-bit or 13-bit) based on the sensor's configuration.
 *
 * @return
 * - The low-temperature threshold in degrees Celsius on success.
 * - `NAN` on failure.
 */
float tmp102_read_low_temp_c(void);

/**
 * @brief Reads the high-temperature threshold in degrees Celsius from the TMP102 sensor.
 *
 * This function retrieves the high-temperature threshold stored in the TMP102's high-temperature register.
 * It adapts to the resolution mode (12-bit or 13-bit) based on the sensor's configuration.
 *
 * @return
 * - The high-temperature threshold in degrees Celsius on success.
 * - `NAN` on failure.
 */
float tmp102_read_high_temp_c(void);

/**
 * @brief Reads the low-temperature threshold in degrees Fahrenheit from the TMP102 sensor.
 *
 * This function retrieves the low-temperature threshold in Celsius using `tmp102_read_low_temp_c`,
 * converts it to Fahrenheit, and returns the result.
 *
 * @return
 * - The low-temperature threshold in degrees Fahrenheit on success.
 * - `NAN` on failure.
 */
float tmp102_read_low_temp_f(void);

/**
 * @brief Reads the high-temperature threshold in degrees Fahrenheit from the TMP102 sensor.
 *
 * This function retrieves the high-temperature threshold in Celsius using `tmp102_read_high_temp_c`,
 * converts it to Fahrenheit, and returns the result.
 *
 * @return
 * - The high-temperature threshold in degrees Fahrenheit on success.
 * - `NAN` on failure.
 */
float tmp102_read_high_temp_f(void);

/**
 * @brief Set the conversion rate of the TMP102 sensor.
 *
 * This function updates the TMP102's configuration register to set the temperature
 * conversion rate. The conversion rate determines how often the sensor updates its temperature reading.
 *
 * @param rate Conversion rate value:
 *             - 0: 0.25Hz
 *             - 1: 1Hz
 *             - 2: 4Hz
 *             - 3: 8Hz
 * @return
 * - `ESP_OK`: Successfully set the conversion rate.
 * - `ESP_ERR_INVALID_ARG`: Invalid `rate` value.
 * - `ESP_FAIL`: Failed to communicate with the sensor.
 */
esp_err_t tmp102_set_conversion_rate(uint8_t rate);

/**
 * @brief Enable or disable extended mode on the TMP102 sensor.
 *
 * Extended mode allows for 13-bit resolution temperature readings (instead of 12-bit).
 *
 * @param mode Extended mode value:
 *             - 0: Disable (12-bit resolution)
 *             - 1: Enable (13-bit resolution)
 * @return
 * - `ESP_OK`: Successfully set the extended mode.
 * - `ESP_ERR_INVALID_ARG`: Invalid `mode` value.
 * - `ESP_FAIL`: Failed to communicate with the sensor.
 */
esp_err_t tmp102_set_extended_mode(tmp102_extended_mode_t mode);

/**
 * @brief Set the polarity of the TMP102 alert pin.
 *
 * This function configures the alert pin polarity for the TMP102 sensor.
 * The alert pin can be configured to be active-high or active-low.
 *
 * @param polarity The desired alert polarity.
 *                 - true: Active HIGH (Alert pin is high when threshold is crossed).
 *                 - false: Active LOW (Alert pin is low when threshold is crossed).
 *
 * @return ESP_OK on success, or an appropriate error code:
 *         - ESP_ERR_INVALID_ARG if an invalid argument is passed.
 *         - ESP_ERR_* codes for any failure encountered during I2C operations.
 */
esp_err_t tmp102_set_alert_polarity(bool polarity);

/**
 * @brief Configure the fault queue of the TMP102 sensor.
 *
 * The fault queue determines the number of consecutive faults required before triggering
 * an alert. This helps to prevent false alerts due to transient conditions.
 *
 * @param fault_setting Number of consecutive faults:
 *                      - 0: 1 fault
 *                      - 1: 2 faults
 *                      - 2: 4 faults
 *                      - 3: 6 faults
 * @return
 * - `ESP_OK`: Successfully set the fault queue.
 * - `ESP_ERR_INVALID_ARG`: Invalid `fault_setting` value.
 * - `ESP_FAIL`: Failed to communicate with the sensor.
 */
esp_err_t tmp102_set_fault(uint8_t fault_setting);

/**
 * @brief Set the alert mode for the TMP102 temperature sensor.
 *
 * This function configures the alert mode of the TMP102 sensor. It allows the user
 * to choose between two alert modes:
 * - Comparator mode (0)
 * - Thermostat mode (1)
 *
 * In Comparator mode, the ALERT pin is active when the temperature crosses a threshold.
 * In Thermostat mode, the ALERT pin is active when the temperature is outside a specified range.
 *
 * @param mode The desired alert mode to set.
 *              - 0: Comparator mode
 *              - 1: Thermostat mode
 *
 * @return ESP_OK on success, or an appropriate error code:
 *         - ESP_ERR_INVALID_ARG if the mode is not within the valid range.
 *         - ESP_ERR_* codes for any other failure encountered during I2C operations.
 *
 */
esp_err_t tmp102_set_alert_mode(tmp102_alert_mode_t mode);

#endif
