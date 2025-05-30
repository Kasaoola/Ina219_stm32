/**
 * @brief INA219 driver using I2C2 for STM32F103.
 * @cite ina219.pdf, rm008.pdf
 * @date 2024-06-28
 * @note Must calibrate for current/power measurements.
 */

#ifndef _INA219_H_
#define _INA219_H_

#include <inttypes.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"

enum ina219_status
{
  INA219_OK,    // I2C operation successful
  INA219_ERROR, // I2C error
  INA219_OVF    // Math overflow in current/power calculation
};

// instance based structure
typedef struct {
  uint8_t address; // I2C address
  float current_lsb; // Current LSB value
  uint16_t config_val;
  bool power_on; // Power on/off status
} INA219_t;

// Configuration and utility functions.

// bool ina219_init(void);
// bool ina219_reset(void);
// bool ina219_get_status(void);
// void ina219_save_power(void);
// bool ina219_configure(const uint16_t val);
// bool ina219_calibrate(const float max_current, const float r_shunt);

bool ina219_init(INA219_t *sensor);
bool ina219_reset(INA219_t *sensor);
bool ina219_get_status(INA219_t *sensor);
void ina219_save_power(INA219_t *sensor);
bool ina219_configure(INA219_t *sensor, const uint16_t val);
bool ina219_calibrate(INA219_t *sensor, const float max_current, const float r_shunt);


// Functions for measurements.

// float ina219_get_current(void);
// float ina219_get_bus_voltage(void);
// float ina219_get_shunt_voltage(void);

float ina219_get_current(INA219_t *sensor);
float ina219_get_bus_voltage(INA219_t *sensor);
float ina219_get_shunt_voltage(INA219_t *sensor);
float ina219_get_power(INA219_t *sensor);

#endif // ina219.h
