# BMP280 Library

This library provides an interface for the Bosch BMP280 digital pressure sensor. It supports both I2C and SPI communication protocols.

> NOTE: This driver has not been tested on any simulation or circuit. However, all the logic and algorithms have been compared with those of other working drivers.

## Features

* Communication mode selection (I2C or SPI)
* Initialization of the BMP280 sensor
* Device ID verification
* Resetting device settings
* Retrieving calibration data
* Setting temperature and pressure oversampling
* Setting power mode
* Getting sensor status
* Reading temperature and pressure data

## Files

* `BMP280.h`: Header file containing definitions, constants, and function prototypes.
* `BMP280.c`: Source file containing the implementation of the library functions.

## Usage

To use the library, the user needs to:

1.  **Include the header file:** `#include "BMP280.h"`
2.  **Create a BMP280 handle:** Declare a `bmp280_handle_t` structure.
3.  **Configure the handle:** Set the communication mode, I2C address (if applicable), and HAL handles for I2C or SPI.
4.  **Initialize the sensor:** Call the `bmp280_init()` function.
5.  **Configure sensor settings:** Use functions like `set_temp_osrs()`, `set_press_osrs()`, and `set_power_mode()` to set the desired measurement parameters.
6.  **Read sensor data:** Use functions like `get_temperature()` and `get_pressure()` to retrieve compensated temperature and pressure values.

## Communication

The library supports both I2C and SPI communication protocols. The communication mode is selected during initialization.

* **I2C**: The library uses the HAL library functions for I2C communication.
* **SPI**: The library uses the HAL library functions for SPI communication.

## Error Handling

The library uses a consistent error-handling mechanism. Most functions return a status code (`uint8_t`) to indicate the success or failure of the operation.
