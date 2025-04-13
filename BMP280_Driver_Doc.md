#   BMP280 Library Description

##   1\. Introduction

This document describes the software library designed to interface with the Bosch BMP280 digital pressure sensor. The library consists of two primary files: `BMP280.h` (header file) and `BMP280.c` (source file). This library provides a set of functions to initialize, configure, and retrieve data from the BMP280 sensor using either I2C or SPI communication protocols.

The BMP280 is a high-precision barometric pressure and temperature sensor suitable for various applications, including weather forecasting, altitude measurement, and indoor navigation.

##   2\. Library Overview

The library is designed to provide an abstraction layer between the user application and the BMP280 sensor's hardware interface. It encapsulates the necessary communication protocols (I2C or SPI) and register operations, presenting a simplified API to the user.

###   2.1. Key Features

* Communication mode selection (I2C or SPI)
* Initialization of the BMP280 sensor
* Device ID verification
* Resetting device settings
* Retrieving calibration data
* Setting temperature and pressure oversampling
* Setting power mode
* Getting sensor status
* Reading temperature and pressure data

##   3\. Header File (`BMP280.h`)

The header file defines the data types, constants, and function prototypes required to use the BMP280 library.

###   3.1. Constants

The header file defines various constants used for interacting with the BMP280 sensor. These include:

* **Sensor Information**
    * `BMP280_RESET`: Software reset value.
    * `BMP280_ID`: Device ID of the BMP280.
    * `BMP280_TIMEOUT`: Timeout value for communication operations (in milliseconds).
* **Registers Addresses**
    * `BMP280_ID_REG`: ID register address.
    * `BMP280_RESET_REG`: Reset register address.
    * `BMP280_STATUS_REG`: Status register address.
    * `BMP280_CTRL_MEAS_REG`: Control measurement register address.
    * `BMP280_CONFIG_REG`: Configuration register address.
    * `BMP280_PRESSURE_REG`: Pressure data register address.
    * `BMP280_TEMP_REG`: Temperature data register address.
* **Calibration Data Addresses**

    * Defines addresses for calibration parameters stored in the sensor's non-volatile memory (NVM). These parameters are used to compensate for sensor variations and improve accuracy.
        * `BMP280_NVM_T1_LSB`
        * `BMP280_NVM_T2_LSB`
        * `BMP280_NVM_T3_LSB`
        * `BMP280_NVM_P1_LSB`
        * `BMP280_NVM_P2_LSB`
        * `BMP280_NVM_P3_LSB`
        * `BMP280_NVM_P4_LSB`
        * `BMP280_NVM_P5_LSB`
        * `BMP280_NVM_P6_LSB`
        * `BMP280_NVM_P7_LSB`
        * `BMP280_NVM_P8_LSB`
        * `BMP280_NVM_P9_LSB`
* **Oversampling Modes**

    * Defines constants for different oversampling modes for temperature and pressure measurements. These modes allow the user to balance power consumption and measurement accuracy.
        * `BMP280_OSRS_SKIP`
        * `BMP280_OSRS_1`
        * `BMP280_OSRS_2`
        * `BMP280_OSRS_4`
        * `BMP280_OSRS_8`
        * `BMP280_OSRS_16`
* **Power Modes**

    * Defines constants for the sensor's power modes: sleep mode, forced mode, and normal mode.
        * `BMP280_POWER_SLEEP`
        * `BMP280_POWER_FORCED`
        * `BMP280_POWER_NORMAL`
* **I2C Addresses**

    * Defines the possible I2C addresses for the BMP280 sensor. The sensor can have one of two I2C addresses.
        * `BMP280_ADDR_0x76`
        * `BMP280_ADDR_0x77`
* **Communication Modes**

    * Defines the communication interface modes supported by the library
        * `BMP280_MODE_I2C`
        * `BMP280_MODE_SPI`

###   3.2. Data Types

The header file also defines several data types used in the library:

* `bmp280_handle_t`: A structure to hold the BMP280 sensor's handle, including communication interface details (I2C or SPI), addresses, and HAL handles.
* `bmp280_comm_mode_t`: An enumeration defining the communication mode (I2C or SPI).
* `bmp280_addr_t`: An enumeration defining the I2C address of the sensor.
* `bmp280_osrs_t`: An enumeration defining the oversampling settings for temperature and pressure.
* `bmp280_power_mode_t`: An enumeration defining the power mode of the sensor.
* `bmp280_calibration_t`: A structure to hold the calibration data read from the sensor's NVM.

###   3.3. Function Prototypes

The header file declares the function prototypes for the library's API. These functions provide the user interface for interacting with the BMP280 sensor.

* **Initialization and Configuration**
    * `uint8_t bmp280_init(bmp280_handle_t *bmp280, bmp280_calibration_t *calib_data)`: Initializes the BMP280 sensor.
    * `uint8_t set_comm_mode(bmp280_handle_t *bmp280, bmp280_comm_mode_t bmp280_comm_mode)`: Sets the communication mode (I2C or SPI).
    * `uint8_t set_i2c_addr(bmp280_handle_t *bmp280, bmp280_addr_t bmp280_addr)`: Sets the I2C address of the sensor.
* **Sensor Control**
    * `uint8_t reset_device(bmp280_handle_t *bmp280)`: Resets the sensor to its default settings.
    * `uint8_t set_temp_osrs(bmp280_handle_t *bmp280, bmp280_osrs_t osrs_t)`: Sets the temperature oversampling.
    * `uint8_t set_press_osrs(bmp280_handle_t *bmp280, bmp280_osrs_p osrs_p)`: Sets the pressure oversampling.
    * `uint8_t set_power_mode(bmp280_handle_t *bmp280, bmp280_power_mode_t pmode)`: Sets the power mode of the sensor.
* **Data Retrieval**
    * `uint8_t check_device_id(bmp280_handle_t *bmp280)`: Checks the device ID of the sensor.
    * `uint8_t get_status(bmp280_handle_t *bmp280, uint8_t *status)`: Gets the status of the sensor.
    * `uint8_t get_temperature(bmp280_handle_t *bmp280, float *temperature)`: Reads the compensated temperature value.
    * `uint8_t get_pressure(bmp280_handle_t *bmp280, float *pressure)`: Reads the compensated pressure value.

##   4\. Source File (`BMP280.c`)

The source file implements the functions declared in the header file, providing the actual logic for interacting with the BMP280 sensor.

###   4.1. Function Implementation

The source file includes implementations for the following functions:

* `bmp280_init(bmp280_handle_t *bmp280, bmp280_calibration_t *calib_data)`
    * Initializes the sensor driver.
    * Checks if the I2C device is ready (if I2C mode is selected).
    * Checks the device ID.
    * Resets the device.
    * Retrieves calibration data from the sensor.
* `set_comm_mode(bmp280_handle_t *bmp280, bmp280_comm_mode_t bmp280_comm_mode)`
    * Sets the communication mode (I2C or SPI).
* `set_i2c_addr(bmp280_handle_t *bmp280, bmp280_addr_t bmp280_addr)`
    * Sets the I2C address.
* `check_device_id(bmp280_handle_t *bmp280)`
    * Checks the device ID by reading the `ID_REG` and comparing it with the expected `BMP280_ID`.
* `reset_device(bmp280_handle_t *bmp280)`
    * Resets the device by writing the `BMP280_RESET` value to the `RESET_REG`.
* `get_status(bmp280_handle_t *bmp280, uint8_t *status)`
    * Retrieves the sensor status by reading the `STATUS_REG`.
    * Checks the register for measuring and updating status.
* `set_temp_osrs(bmp280_handle_t *bmp280, bmp280_osrs_t osrs_t)`
    * Sets the temperature oversampling by modifying the `CTRL_MEAS_REG`.
* `set_press_osrs(bmp280_handle_t *bmp280, bmp280_osrs_p osrs_p)`
    * Sets the pressure oversampling by modifying the `CTRL_MEAS_REG`.
* `set_power_mode(bmp280_handle_t *bmp280, bmp280_power_mode_t pmode)`
    * Sets the power mode by modifying the `CTRL_MEAS_REG`.
* `get_calibration_datas(bmp280_handle_t *bmp280, bmp280_calibration_t *calib_data)`
    * Retrieves calibration parameters from the sensor's NVM.
* `get_temperature(bmp280_handle_t *bmp280, float *temperature)`
    * Reads the raw temperature data from the sensor and compensates it using calibration parameters.
* `get_pressure(bmp280_handle_t *bmp280, float *pressure)`
    * Reads the raw pressure data from the sensor and compensates it using calibration parameters.

##   5\. Communication

The library supports both I2C and SPI communication protocols. The communication mode is selected during initialization.

* **I2C**: The library uses the HAL library functions for I2C communication. It checks for device readiness and uses memory read/write functions to access the sensor's registers.
* **SPI**: The library uses the HAL library functions for SPI communication. It transmits and receives data to/from the sensor's registers.

##   6\. Error Handling

The library uses a consistent error-handling mechanism. Most functions return a status code (`uint8_t`) to indicate the success or failure of the operation. A return value of 0 typically indicates success, while non-zero values indicate specific error conditions.

##   7\. BMP280 Sensor

The BMP280 is a digital pressure sensor from Bosch Sensortec. It's designed for mobile applications and provides accurate pressure and temperature measurements.

###   7.1. Key Features of BMP280

* Pressure range: 300 to 1100 hPa [cite: 1, 2, 3, 4, 5]
* Temperature range: -40 to +85 °C [cite: 1, 2, 3, 4, 5]
* Digital interfaces: I2C (up to 3.4 MHz), SPI (3 and 4-wire, up to 10 MHz) [cite: 1, 2, 3, 904]
* Low power consumption

###   7.2. Communication Interfaces

The BMP280 supports both I2C and SPI communication protocols.

* I2C: Up to 3.4 MHz [cite: 1, 2, 3, 904]
* SPI: 3 and 4-wire, up to 10 MHz [cite: 1, 2, 3, 904]

###   7.3. Measurement Modes

The BMP280 can operate in different power modes to optimize power consumption and measurement speed.

* **Sleep Mode**: Lowest power consumption, no measurements. [cite: 499, 500, 501, 502, 971, 972, 973]
* **Forced Mode**: Single measurement on demand. [cite: 499, 500, 501, 502, 971, 972, 973]
* **Normal Mode**: Periodic measurements. [cite: 499, 500, 501, 502, 971, 972, 973]

###   7.4 Oversampling

The BMP280 allows for oversampling to improve measurement accuracy. Oversampling involves taking multiple measurements and averaging them. The sensor supports different oversampling settings for both pressure and temperature measurements.

###   7.5 Calibration

The BMP280 stores calibration data in its non-volatile memory (NVM). This calibration data is used to compensate for sensor variations and improve the accuracy of pressure and temperature measurements.

##   8\. Using the Library

To use the library, the user needs to:

1.  **Include the header file:** `#include "BMP280.h"`
2.  **Create a BMP280 handle:** Declare a `bmp280_handle_t` structure.
3.  **Configure the handle:** Set the communication mode, I2C address (if applicable), and HAL handles for I2C or SPI.
4.  **Initialize the sensor:** Call the `bmp280_init()` function.
5.  **Configure sensor settings:** Use functions like `set_temp_osrs()`, `set_press_osrs()`, and `set_power_mode()` to set the desired measurement parameters.
6.  **Read sensor data:** Use functions like `get_temperature()` and `get_pressure()` to retrieve compensated temperature and pressure values.

##   9\. Conclusion

This BMP280 library provides a comprehensive set of functions for interfacing with the BMP280 pressure sensor. It simplifies the process of configuring the sensor, retrieving data, and handling communication protocols. By using this library, developers can easily integrate the BMP280 sensor into their applications.
