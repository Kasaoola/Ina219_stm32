#include "ina219.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>

// extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c1;
// INA219 macros
// #define INA219_I2C_SLAVE_ADD 0x40
#define INA219_REG_CONFIGURATION 0x00
#define INA219_REG_SHUNT_VOLTAGE 0x01
#define INA219_REG_BUS_VOLTAGE 0x02
#define INA219_REG_POWER 0x03
#define INA219_REG_CURRENT 0x04
#define INA219_REG_CALIBRATION 0x05

#define INA219_CONFIG_MODE_POWERDOWN 0x0000
#define INA219_CONFIG_MODE_SHUNT_AND_BUS_CONT 0x0007

uint16_t config_val = 0x399F; // Configuration value tracker.
float current_lsb = 0.0; // Please calibrate for current/power measurements.

uint16_t i2c_read(INA219_t *sensor, uint8_t reg);
bool i2c_write(INA219_t *sensor, uint8_t reg, uint16_t val);

bool ina219_init(INA219_t *sensor)
{
  if(ina219_reset(sensor))
  {
    return true;
  }

  return false;
}

void ina219_save_power(INA219_t *sensor)
{
  uint16_t config = i2c_read(sensor, INA219_REG_CONFIGURATION);

  if (sensor->power_on){
    config = (config & 0xFFF8) | INA219_CONFIG_MODE_POWERDOWN;
  }
  else {
    config = (config & 0xFFF8) | INA219_CONFIG_MODE_SHUNT_AND_BUS_CONT;
  }
  i2c_write (sensor, INA219_REG_CONFIGURATION, config);
}

// Reset sensor to startup state.
bool ina219_reset(INA219_t *sensor)
{
  if(i2c_write(sensor, INA219_REG_CONFIGURATION, 0x8000)){
    printf("INA219 reset successful!\n");
  }
  else{
    printf("INA219 reset failed!\n");
  }

  if (i2c_read(sensor, INA219_REG_CONFIGURATION) == 0x399F)
  {
    sensor->config_val = 0x399F;
    sensor->current_lsb = 0.0;
    printf("INA219 reset successful!\n");
    return true;
  }
  else{
    printf("INA219 reset failed!\n");
    return false;
  }
}

/**
 * @brief Populates calibration register (0x05) for current and power measurements.
 * @param max_current Maximum expected current, Amp.
 * @param r_shunt Shunt resistance, Ohm.
 * @cite Page 13, ina219.pdf.
 */
bool ina219_calibrate(INA219_t *sensor, const float max_current, const float r_shunt)
{
  float temp_current_lsb = max_current * 3.0517578125e-5;
  const uint16_t calibration_val = (uint16_t)(0.04096 / (temp_current_lsb * r_shunt));
  i2c_write(sensor, INA219_REG_CALIBRATION, calibration_val);

  // The least significant bit is always 0.
  if(i2c_read(sensor, INA219_REG_CALIBRATION) == (calibration_val & 0xFFFE))
  {
    sensor->current_lsb = temp_current_lsb;
    return true;
  }

  return false;
}

// See ina219_regcal.cpp to compute config_val.
bool ina219_configure(INA219_t *sensor, const uint16_t val)
{
  i2c_write(sensor, INA219_REG_CONFIGURATION, val);

  if(i2c_read(sensor, INA219_REG_CONFIGURATION) == val)
  {
    sensor->config_val = val;
    return true;
  }

  return false;
}

// Voltage across in- and gnd, V.
float ina219_get_bus_voltage(INA219_t *sensor)
{
  uint16_t reg_val = i2c_read(sensor, INA219_REG_BUS_VOLTAGE);
  const float bus_voltage_lsb = 4e-3; // 4mV
  return (reg_val >> 3) * bus_voltage_lsb;
}

// Current through the shunt resistor, mA.
float ina219_get_current(INA219_t *sensor)
{
  uint16_t reg_val = i2c_read(sensor, INA219_REG_CURRENT);
  return reg_val * sensor->current_lsb;
}

// Voltage across shunt resistor, V.
float ina219_get_shunt_voltage(INA219_t *sensor)
{
  uint16_t reg_val = i2c_read(sensor, INA219_REG_SHUNT_VOLTAGE);
  const float shunt_voltage_lsb = 1e-5; // 10uV
  return reg_val * shunt_voltage_lsb;
}

// Power consumed, Watts.
float ina219_get_power(INA219_t *sensor)
{
  uint16_t reg_val = i2c_read(sensor, INA219_REG_POWER);
  return reg_val * 20 * sensor->current_lsb;
}

// I2C test by reading config register.
bool ina219_get_status(INA219_t *sensor)
{
  if(i2c_read(sensor, INA219_REG_CONFIGURATION) == sensor->config_val)
  {
    return true;
  }
  return false;
}

uint16_t i2c_read(INA219_t *sensor, uint8_t reg) {
  uint8_t data[2] = {0};
  // HAL_I2C_Master_Transmit(&hi2c2, sensor->address << 1, &reg, 1, HAL_MAX_DELAY);
  // HAL_I2C_Master_Receive(&hi2c2, sensor->address << 1, data, 2, HAL_MAX_DELAY);
  HAL_I2C_Master_Transmit(&hi2c1, sensor->address << 1, &reg, 1, HAL_MAX_DELAY);
  HAL_I2C_Master_Receive(&hi2c1, sensor->address << 1, data, 2, HAL_MAX_DELAY);
  return (data[0] << 8) | data[1];
}

bool i2c_write(INA219_t *sensor, const uint8_t reg, const uint16_t val){
  uint8_t data[3] = {reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF)};
  // if (HAL_I2C_Master_Transmit(&hi2c2, sensor->address << 1, data, 3, HAL_MAX_DELAY) == HAL_OK)
  // {
  //   return true;
  // }
  // return false;
  if (HAL_I2C_Master_Transmit(&hi2c1, sensor->address << 1, data, 3, HAL_MAX_DELAY) == HAL_OK)
  {
    return true;
  }
  return false;
}