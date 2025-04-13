/*
 * BMP280.h
 *
 *  Created on: Apr 8, 2025
 *      Author: Furkan
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#include "main.h"

/* SENSOR INFOS */

#define BMP280_RESET							0xB6
#define BMP280_ID								0x58
#define BMP280_TIMEOUT							1000	// ms

/* REGISTERS */
#define BMP280_ID_REG							0xD0	// Read Only, the id is 0x58
#define BMP280_RESET_REG						0xE0	// Write Only, 0xB6 is the only accepted value
#define BMP280_STATUS_REG						0xF3	// Read Only
#define BMP280_CTRL_MEAS_REG					0xF4	// R/W
#define BMP280_CONFIG_REG						0xF5	// R/W

#define BMP280_PRESSURE_REG						0xF7 	// F7-F9, Read Only
#define BMP280_TEMP_REG							0xFA	// FA-FC, Read Only

/* Calibration Data Addresses. For more information check p. 21 */
#define BMP280_NVM_T1_LSB						0x88
#define BMP280_NVM_T2_LSB						0x8A
#define BMP280_NVM_T3_LSB						0x8C

#define BMP280_NVM_P1_LSB						0x8E
#define BMP280_NVM_P2_LSB						0x90
#define BMP280_NVM_P3_LSB						0x92
#define BMP280_NVM_P4_LSB						0x94
#define BMP280_NVM_P5_LSB						0x96
#define BMP280_NVM_P6_LSB						0x98
#define BMP280_NVM_P7_LSB						0x9A
#define BMP280_NVM_P8_LSB						0x9C
#define BMP280_NVM_P9_LSB						0x9E

typedef enum {
    BMP280_MODE_I2C = 0x00,
    BMP280_MODE_SPI = 0x01
}bmp280_comm_mode_t;


/* I2C Slave ID can be changed during your SDO pin connection.
 * Check p. 28, section 5.2 for more information.
 */
typedef enum {
	SDO_2_GND = (0x76 << 1),
	SDO_2_VDDIO = (0x77 << 1)
}bmp280_addr_t;

/* Sensor Power Modes. Check p. 15 */
typedef enum {
	SLEEP_MODE = 0x00,
	FORCED_MODE = 0x01,
	NORMAL_MODE = 0x03
}bmp280_power_mode_t;

/* Temperature Oversampling Modes. Check p. 13 */
typedef enum {
	OSRS_T_DISABLE,								// Disabled
	OSRS_T_X1,									// x1 - 16 bit / 0.0050 °C
	OSRS_T_X2,									// x2 - 17 bit / 0.0025 °C
	OSRS_T_X4,									// x4 - 18 bit / 0.0012 °C
	OSRS_T_X8,									// x8 - 19 bit / 0.0006 °C
	OSRS_T_X16									// x16 - 20 bit / 0.0003 °C
}bmp280_osrs_t;

/* Pressure Oversampling Modes. Check p. 12 */
typedef enum {
	OSRS_P_DISABLE,								// Disabled
	OSRS_P_ULTRA_LOW_POWER,						// x1 - 20 bit / 2.62 Pa
	OSRS_P_LOW_POWER,							// x2 - 20 bit / 1.31 Pa
	OSRS_P_STANDART,							// x4 - 20 bit / 0.66 Pa
	OSRS_P_HIGH,								// x8 - 20 bit / 0.33 Pa
	OSRS_P_ULTRA_HIGH							// x16 - 20 bit / 0.16 Pa
}bmp280_osrs_p;

/* IIR Filter. Check p. 14 */
typedef enum {
	FILTER_DISABLED,
	FILTER_2,
	FILTER_4,
	FILTER_8,
	FILTER_16
}bmp280_iir_filter_t;

/* Standby Times. Check p. 17 */
typedef enum {
	STANDBY_0_POINT_5_MS,
	STANDBY_62_POINT_5_MS,
	STANDBY_125_MS,
	STANDBY_250_MS,
	STANDBY_500_MS,
	STANDBY_1000_MS,
	STANDBY_2000_MS,
	STANDBY_4000_MS
}bmp280_standby_time_t;

typedef enum {
	SPI_3_WIRE_DISABLED,
	SPI_3_WIRE_ENABLED
}bmp280_spi_mode_t;

typedef struct {
	I2C_HandleTypeDef *hi2c;
	SPI_HandleTypeDef *hspi;
	uint8_t comm_mode;
	uint8_t i2c_addr;
	float temperature;
	float pressure;
}bmp280_handle_t;

typedef struct {
	uint8_t data[2];
	uint8_t status;
	uint8_t reset;
}device_ops_t;

typedef struct {
	uint16_t t1;
	int16_t  t2;
	int16_t  t3;
	uint16_t p1;
	int16_t  p2;
	int16_t  p3;
	int16_t  p4;
	int16_t  p5;
	int16_t  p6;
	int16_t  p7;
	int16_t  p8;
	int16_t  p9;
	int32_t  t_fine;
}bmp280_calibration_t;

uint8_t bmp280_init(bmp280_handle_t *bmp280, bmp280_calibration_t *calib_data);

uint8_t set_comm_mode(bmp280_handle_t *bmp280, bmp280_comm_mode_t bmp280_comm_mode);
uint8_t set_i2c_addr(bmp280_handle_t *bmp280, bmp280_addr_t bmp280_addr);

uint8_t check_device_id(bmp280_handle_t *bmp280);
uint8_t reset_device(bmp280_handle_t *bmp280);
uint8_t get_status(bmp280_handle_t *bmp280, uint8_t *status);

/* Oversampling Register Set Functions p. 25 */

uint8_t set_temp_osrs(bmp280_handle_t *bmp280, bmp280_osrs_t osrs_t);
uint8_t set_press_osrs(bmp280_handle_t *bmp280, bmp280_osrs_p osrs_p);
uint8_t set_power_mode(bmp280_handle_t *bmp280, bmp280_power_mode_t pmode);

/* Config Register Set Functions p. 26 */

uint8_t set_standby_time(bmp280_handle_t *bmp280, bmp280_standby_time_t standby_time);
uint8_t set_iir_filter(bmp280_handle_t *bmp280, bmp280_iir_filter_t iir_filter);
uint8_t set_spi_mode(bmp280_handle_t *bmp280, bmp280_spi_mode_t spi_mode);


uint8_t get_temperature(bmp280_handle_t *bmp280, bmp280_calibration_t *calib_data);
uint8_t get_pressure(bmp280_handle_t *bmp280, bmp280_calibration_t *calib_data);

static uint8_t compansate_temperature(bmp280_handle_t *bmp280, uint32_t raw_temp, bmp280_calibration_t *calib_data);
static uint8_t compansate_pressure(bmp280_handle_t *bmp280, uint32_t raw_press, bmp280_calibration_t *calib_data);
static uint8_t get_calibration_datas(bmp280_handle_t *bmp280, bmp280_calibration_t *calib_data);



#endif /* INC_BMP280_H_ */
