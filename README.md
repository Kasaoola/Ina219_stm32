# STM32 INA219 Dual Power Monitoring System

This project uses an **STM32F103C8T6** microcontroller to interface with **two INA219 sensors** via **I2C**. It measures the voltage, current, and power across both a **battery source** and a **system load**, and displays the values through **UART (USART1)**.

## 🚀 Features

- Dual INA219 sensor support (Battery + System line)
- Real-time monitoring of:
  - Bus Voltage (V)
  - Shunt Voltage (V)
  - Current (A)
  - Power (W)
- UART logging for easy debugging/viewing
- Includes sensor initialization, calibration, and configuration
- Uses STM32 HAL libraries

## 📷 Hardware Setup

| Component     | Description                      |
|---------------|----------------------------------|
| STM32 Board   | STM32F103C8T6 (Blue Pill)         |
| INA219 Sensor | x2 (I2C addresses 0x40 & 0x41)    |
| Power Source  | Battery or DC supply             |
| Load          | System circuit or resistor load  |
| UART Monitor  | Serial terminal (baud: 9600)     |

### 🧰 Wiring (Example)

| STM32 Pin | INA219 (Battery) | INA219 (System) |
|-----------|------------------|-----------------|
| PB6       | SDA              | SDA             |
| PB7       | SCL              | SCL             |
| GND       | GND              | GND             |
| 3.3V/5V   | VCC              | VCC             |

Ensure the INA219s have different I2C addresses (default 0x40, can set second to 0x41 via address pin).

## 📄 Code Overview

- **`main.c`**: Sets up peripherals and runs the sensor polling loop.
- **`ina219.c/h`**: Driver code to initialize, configure, and read from the INA219 sensors.
- Data is printed to the serial terminal every second for both sensors.

## 🧪 Output Example (via UART)

