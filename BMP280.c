/*
 * BMP280.c
 *
 *  Created on: Apr 8, 2025
 *      Author: Furkan
 */

#include "BMP280.h"

/*
 * @brief		init the sensor driver
 * @param		bmp280 pointer to a bmp280 handle structure
 * @return		status code
 * 				- 0 success
 *				- 1 i2c device not ready
 */
uint8_t bmp280_init(bmp280_handle_t *bmp280, bmp280_calibration_t *calib_data) {
	uint8_t err;

	if(bmp280->comm_mode == BMP280_MODE_I2C) {
		if(HAL_I2C_IsDeviceReady(bmp280->hi2c, bmp280->i2c_addr, 1, BMP280_TIMEOUT) != 0) {
			return 1;
		}
	}

	/* Check Device ID */
	err = check_device_id(bmp280);
	if(err != 0) {
		return err;
	}

	/* Reset Device Settings to Default */
	err = reset_device(bmp280);
	if(err != 0) {
		return err;
	}

	/* Get Calibration Datas and Save Them */
	err = get_calibration_datas(bmp280, calib_data);
	if(err != 0) {
		return err;
	}

	return 0;
}

/*
 * @brief		sets the sensor communication type
 * @param		bmp280 pointer to a bmp280 handle structure
 * @param		bmp280_comm_mode to to get user's communication mode
 * @return		status code
 * 				- 0 success
 *				- 1 null handle error
 */
uint8_t set_comm_mode(bmp280_handle_t *bmp280, bmp280_comm_mode_t bmp280_comm_mode){
	if(bmp280 == NULL) {
		return 1;
	}

	bmp280->comm_mode = bmp280_comm_mode;

	return 0;
}

/*
 * @brief		sets the sensor i2c address
 * @param		bmp280 pointer to a bmp280 handle structure
 * @param		bmp280_addr to to get user's i2c address
 * @return		status code
 * 				- 0 success
 *				- 1 null handle error
 */
uint8_t set_i2c_addr(bmp280_handle_t *bmp280, bmp280_addr_t bmp280_addr) {
	if(bmp280 == NULL) {
		return 1;
	}

	bmp280->i2c_addr = bmp280_addr;

	return 0;
}

/*
 * @brief		checks the sensor device id
 * @param		bmp280 pointer to a bmp280 handle structure
 * @return		status code
 * 				- 0 success
 *				- 1 i2c memory read error
 * 				- 2 spi write/read error
 * 				- 3 device id not matched
 */
uint8_t check_device_id(bmp280_handle_t *bmp280) {
	uint8_t dev_id = 0;

	if(bmp280->comm_mode == BMP280_MODE_I2C) {
		if(HAL_I2C_Mem_Read(bmp280->hi2c, bmp280->i2c_addr, BMP280_ID_REG, I2C_MEMADD_SIZE_8BIT, &dev_id, 1, BMP280_TIMEOUT) != 0) {
			return 1;
		}
	}
	else {
		if(HAL_SPI_TransmitReceive(bmp280->hspi, (uint8_t *)BMP280_ID_REG, &dev_id, 1, BMP280_TIMEOUT) != 0) {
			return 2;
		}
	}

	if(dev_id != BMP280_ID) {
		return 3;
	}

	return 0;
}

/*
 * @brief		resets sensor settings
 * @param		bmp280 pointer to a bmp280 handle structure
 * @return		status code
 * 				- 0 success
 *				- 1 i2c memory write error
 * 				- 2 spi write/read error
 */
uint8_t reset_device(bmp280_handle_t *bmp280) {
	uint8_t data = BMP280_RESET;

	if(bmp280->comm_mode == BMP280_MODE_I2C) {
		if(HAL_I2C_Mem_Write(bmp280->hi2c, bmp280->i2c_addr, BMP280_RESET_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, BMP280_TIMEOUT) != 0) {
			return 1;
		}
	}
	else {
		if(HAL_SPI_Transmit(bmp280->hspi, &data, 1, BMP280_TIMEOUT) != 0) {
			return 2;
		}
	}

	return 0;
}

/*
 * @brief		gets status of sensor state (measuring/updating)
 * @param		bmp280 pointer to a bmp280 handle structure
 * @return		status code
 * 				- 0 success
 *				- 1 i2c memory write error
 * 				- 2 spi write/read error
 */
uint8_t get_status(bmp280_handle_t *bmp280, uint8_t *status) {
	uint8_t statusReg = 0;

	if(bmp280->comm_mode == BMP280_MODE_I2C) {
		if(HAL_I2C_Mem_Read(bmp280->hi2c, bmp280->i2c_addr, BMP280_STATUS_REG, I2C_MEMADD_SIZE_8BIT, &statusReg, 1, BMP280_TIMEOUT) != 0) {
			return 1;
		}
	}

	else {
		if(HAL_SPI_TransmitReceive(bmp280->hspi, (uint8_t *)BMP280_STATUS_REG, &statusReg, 1, BMP280_TIMEOUT) != 0) {
			return 2;
		}
	}

	/* Check Status Register */
	if(statusReg & 0x01) {	// Updating data
		*status = 0;
	}

	if(statusReg & 0x08) {	// Measuring Data
		*status = 1;
	}

	return 0;
}

/*
 * @brief		sets temperature oversampling setting
 * @param		bmp280 pointer to a bmp280 handle structure
 * @param		osrs_t to get user's setting
 * @return		status code
 * 				- 0 success
 *				- 1 i2c memory read/write error
 * 				- 2 spi read/write error
 */
uint8_t set_temp_osrs(bmp280_handle_t *bmp280, bmp280_osrs_t osrs_t) {
	uint8_t ctrl_meas;
	uint8_t package[2] = {0, 0};

	if(bmp280->comm_mode == BMP280_MODE_I2C) {
		/* Get current ctrl_meas register */
		if(HAL_I2C_Mem_Read(bmp280->hi2c, bmp280->i2c_addr, BMP280_CTRL_MEAS_REG, I2C_MEMADD_SIZE_8BIT, &ctrl_meas, 1, BMP280_TIMEOUT) != 0) {
			return 1;
		}

		/* Clear current setting */
		ctrl_meas &= (0x1F);

		/* Add user's setting to current ctrl_meas register */
		ctrl_meas |= ((uint8_t)osrs_t << 5);

		/* Send new ctrl_meas to sensor */
		if(HAL_I2C_Mem_Write(bmp280->hi2c, bmp280->i2c_addr, BMP280_CTRL_MEAS_REG, I2C_MEMADD_SIZE_8BIT, &ctrl_meas, 1, BMP280_TIMEOUT) != 0) {
			return 1;
		}
	}

	else {
		/* Get current ctrl_meas register */
		if(HAL_SPI_TransmitReceive(bmp280->hspi, (uint8_t *)BMP280_CTRL_MEAS_REG, &ctrl_meas, 1, BMP280_TIMEOUT) != 0) {
			return 2;
		}

		/* Clear current setting */
		ctrl_meas &= (0x1F);

		/* Add user's setting to current ctrl_meas register */
		ctrl_meas |= ((uint8_t)osrs_t << 5);
		package[0] = BMP280_CTRL_MEAS_REG;
		package[1] = ctrl_meas;

		/* Send new ctrl_meas to sensor */
		if(HAL_SPI_Transmit(bmp280->hspi, package, 2, BMP280_TIMEOUT) != 0) {
			return 2;
		}
	}

	return 0;
}

/*
 * @brief		sets pressure oversampling setting
 * @param		bmp280 pointer to a bmp280 handle structure
 * @param		osrs_p to get user's setting
 * @return		status code
 * 				- 0 success
 *				- 1 i2c memory read/write error
 * 				- 2 spi read/write error
 */
uint8_t set_press_osrs(bmp280_handle_t *bmp280, bmp280_osrs_p osrs_p) {
	uint8_t ctrl_meas;
	uint8_t package[2] = {0, 0};

	if(bmp280->comm_mode == BMP280_MODE_I2C) {
		/* Get current ctrl_meas register */
		if(HAL_I2C_Mem_Read(bmp280->hi2c, bmp280->i2c_addr, BMP280_CTRL_MEAS_REG, I2C_MEMADD_SIZE_8BIT, &ctrl_meas, 1, BMP280_TIMEOUT) != 0) {
			return 1;
		}

		/* Clear current setting */
		ctrl_meas &= (0xE3);

		/* Add user's setting to current ctrl_meas register */
		ctrl_meas |= ((uint8_t)osrs_p << 2);

		/* Send new ctrl_meas to sensor */
		if(HAL_I2C_Mem_Write(bmp280->hi2c, bmp280->i2c_addr, BMP280_CTRL_MEAS_REG, I2C_MEMADD_SIZE_8BIT, &ctrl_meas, 1, BMP280_TIMEOUT) != 0) {
			return 1;
		}
	}

	else {
		/* Get current ctrl_meas register */
		if(HAL_SPI_TransmitReceive(bmp280->hspi, (uint8_t *)BMP280_CTRL_MEAS_REG, &ctrl_meas, 1, BMP280_TIMEOUT) != 0) {
			return 2;
		}

		/* Clear current setting */
		ctrl_meas &= (0xE3);

		/* Add user's setting to current ctrl_meas register */
		ctrl_meas |= ((uint8_t)osrs_p << 2);
		package[0] = BMP280_CTRL_MEAS_REG;
		package[1] = ctrl_meas;

		/* Send new ctrl_meas to sensor */
		if(HAL_SPI_Transmit(bmp280->hspi, package, 2, BMP280_TIMEOUT) != 0) {
			return 2;
		}
	}

	return 0;
}

/*
 * @brief		sets power mode setting
 * @param		bmp280 pointer to a bmp280 handle structure
 * @param		pmode to get user's power mode setting
 * @return		status code
 * 				- 0 success
 *				- 1 i2c memory read/write error
 * 				- 2 spi read/write error
 */
uint8_t set_power_mode(bmp280_handle_t *bmp280, bmp280_power_mode_t pmode) {
	uint8_t ctrl_meas;
	uint8_t package[2] = {0, 0};

	if(bmp280->comm_mode == BMP280_MODE_I2C) {
		/* Get current ctrl_meas register */
		if(HAL_I2C_Mem_Read(bmp280->hi2c, bmp280->i2c_addr, BMP280_CTRL_MEAS_REG, I2C_MEMADD_SIZE_8BIT, &ctrl_meas, 1, BMP280_TIMEOUT) != 0) {
			return 1;
		}

		/* Clear current setting */
		ctrl_meas &= (0xFC);

		/* Add user's setting to current ctrl_meas register */
		ctrl_meas |= (uint8_t)pmode;

		/* Send new ctrl_meas to sensor */
		if(HAL_I2C_Mem_Write(bmp280->hi2c, bmp280->i2c_addr, BMP280_CTRL_MEAS_REG, I2C_MEMADD_SIZE_8BIT, &ctrl_meas, 1, BMP280_TIMEOUT) != 0) {
			return 1;
		}
	}

	else {
		/* Get current ctrl_meas register */
		if(HAL_SPI_TransmitReceive(bmp280->hspi, (uint8_t *)BMP280_CTRL_MEAS_REG, &ctrl_meas, 1, BMP280_TIMEOUT) != 0) {
			return 2;
		}

		/* Clear current setting */
		ctrl_meas &= (0xFC);

		/* Add user's setting to current ctrl_meas register */
		ctrl_meas |= (uint8_t)pmode;
		package[0] = BMP280_CTRL_MEAS_REG;
		package[1] = ctrl_meas;

		/* Send new ctrl_meas to sensor */
		if(HAL_SPI_Transmit(bmp280->hspi, package, 2, BMP280_TIMEOUT) != 0) {
			return 2;
		}
	}

	return 0;
}

/*
 * @brief		sets standby time setting
 * @param		bmp280 pointer to a bmp280 handle structure
 * @param		standby_time to get user's standby time selection
 * @return		status code
 * 				- 0 success
 *				- 1 i2c memory read/write error
 * 				- 2 spi read/write error
 */
uint8_t set_standby_time(bmp280_handle_t *bmp280, bmp280_standby_time_t standby_time) {
	uint8_t config;
	uint8_t package[2] = {0, 0};

	if(bmp280->comm_mode == BMP280_MODE_I2C) {
		/* Get current config register from sensor */
		if(HAL_I2C_Mem_Read(bmp280->hi2c, bmp280->i2c_addr, BMP280_CONFIG_REG, I2C_MEMADD_SIZE_8BIT, &config, 1, BMP280_TIMEOUT) != 0) {
			return 1;
		}

		/* Clear current standby time settings */
		config &= (0x1F);

		/* Set user's stanby time setting */
		config |= (standby_time << 5);

		/* Send new config to sensor */
		if(HAL_I2C_Mem_Write(bmp280->hi2c, bmp280->i2c_addr, BMP280_CONFIG_REG, I2C_MEMADD_SIZE_8BIT, &config, 1, BMP280_TIMEOUT) != 0) {
			return 1;
		}
	}

	else {
		/* Get current config register from sensor */
		if(HAL_SPI_TransmitReceive(bmp280->hspi, (uint8_t *)BMP280_CONFIG_REG, &config, 1, BMP280_TIMEOUT) != 0) {
			return 2;
		}

		/* Clear current standby time settings */
		config &= (0x1F);

		/* Set user's stanby time setting */
		config |= (standby_time << 5);
		package[0] = BMP280_CONFIG_REG;
		package[1] = config;

		/* Send new config to sensor */
		if(HAL_SPI_Transmit(bmp280->hspi, package, 2, BMP280_TIMEOUT) != 0) {
			return 2;
		}
	}

	return 0;
}

/*
 * @brief		sets iir filter setting
 * @param		bmp280 pointer to a bmp280 handle structure
 * @param		iir_filter to get user's iir filter selection
 * @return		status code
 * 				- 0 success
 *				- 1 i2c memory read/write error
 * 				- 2 spi read/write error
 */
uint8_t set_iir_filter(bmp280_handle_t *bmp280, bmp280_iir_filter_t iir_filter) {
	uint8_t config;
	uint8_t package[2] = {0, 0};

	if(bmp280->comm_mode == BMP280_MODE_I2C) {
		/* Get current config register from sensor */
		if(HAL_I2C_Mem_Read(bmp280->hi2c, bmp280->i2c_addr, BMP280_CONFIG_REG, I2C_MEMADD_SIZE_8BIT, &config, 1, BMP280_TIMEOUT) != 0) {
			return 1;
		}

		/* Clear current standby time settings */
		config &= (0xE3);

		/* Set user's stanby time setting */
		config |= (iir_filter << 2);

		/* Send new config to sensor */
		if(HAL_I2C_Mem_Write(bmp280->hi2c, bmp280->i2c_addr, BMP280_CONFIG_REG, I2C_MEMADD_SIZE_8BIT, &config, 1, BMP280_TIMEOUT) != 0) {
			return 1;
		}
	}

	else {
		/* Get current config register from sensor */
		if(HAL_SPI_TransmitReceive(bmp280->hspi, (uint8_t *)BMP280_CONFIG_REG, &config, 1, BMP280_TIMEOUT) != 0) {
			return 2;
		}

		/* Clear current standby time settings */
		config &= (0xE3);

		/* Set user's stanby time setting */
		config |= (iir_filter << 2);
		package[0] = BMP280_CONFIG_REG;
		package[1] = config;

		/* Send new config to sensor */
		if(HAL_SPI_Transmit(bmp280->hspi, package, 2, BMP280_TIMEOUT) != 0) {
			return 2;
		}
	}

	return 0;
}

/*
 * @brief		sets spi 3 wire mode setting
 * @param		bmp280 pointer to a bmp280 handle structure
 * @param		spi_mode to get user's spi 3 wire mode selection
 * @return		status code
 * 				- 0 success
 *				- 1 i2c memory read/write error
 * 				- 2 spi read/write error
 */
uint8_t set_spi_mode(bmp280_handle_t *bmp280, bmp280_spi_mode_t spi_mode) {
	uint8_t config;
	uint8_t package[2] = {0, 0};

	if(bmp280->comm_mode == BMP280_MODE_I2C) {
		/* Get current config register from sensor */
		if(HAL_I2C_Mem_Read(bmp280->hi2c, bmp280->i2c_addr, BMP280_CONFIG_REG, I2C_MEMADD_SIZE_8BIT, &config, 1, BMP280_TIMEOUT) != 0) {
			return 1;
		}

		/* Clear current standby time settings */
		config &= (0xFC);

		/* Set user's stanby time setting */
		config |= (spi_mode);

		/* Send new config to sensor */
		if(HAL_I2C_Mem_Write(bmp280->hi2c, bmp280->i2c_addr, BMP280_CONFIG_REG, I2C_MEMADD_SIZE_8BIT, &config, 1, BMP280_TIMEOUT) != 0) {
			return 1;
		}
	}

	else {
		/* Get current config register from sensor */
		if(HAL_SPI_TransmitReceive(bmp280->hspi, (uint8_t *)BMP280_CONFIG_REG, &config, 1, BMP280_TIMEOUT) != 0) {
			return 2;
		}

		/* Clear current standby time settings */
		config &= (0xFC);

		/* Set user's stanby time setting */
		config |= (spi_mode);
		package[0] = BMP280_CONFIG_REG;
		package[1] = config;

		/* Send new config to sensor */
		if(HAL_SPI_Transmit(bmp280->hspi, package, 2, BMP280_TIMEOUT) != 0) {
			return 2;
		}
	}

	return 0;
}

/*
 * @brief		gets temperature values from sensor's related registers
 * @param		bmp280 pointer to a bmp280 handle structure
 * @return		status code
 * 				- 0 success
 *				- 1 i2c memory read/write error
 * 				- 2 spi read/write error
 * 				- 3 compansate calculation error
 */
uint8_t get_temperature(bmp280_handle_t *bmp280, bmp280_calibration_t *calib_data) {
	uint8_t err;
	uint8_t ctrl_meas;
	uint8_t temp_data[3] = {0, 0, 0};
	uint32_t raw_temp;
	uint8_t package[2] = {0, 0};

	if(bmp280->comm_mode == BMP280_MODE_I2C) {
		if(HAL_I2C_Mem_Read(bmp280->hi2c, bmp280->i2c_addr, BMP280_CTRL_MEAS_REG, I2C_MEMADD_SIZE_8BIT, &ctrl_meas, 1, BMP280_TIMEOUT) != 0) {
			return 1;
		}

		/* Check device mode */

		/* Normal Mode */
		if((ctrl_meas & 0x03) == 0x03) {
			/* Read Raw Temperature Data from Sensor */
			if(HAL_I2C_Mem_Read(bmp280->hi2c, bmp280->i2c_addr, BMP280_TEMP_REG, I2C_MEMADD_SIZE_8BIT, temp_data, 3, BMP280_TIMEOUT) != 0) {
				return 1;
			}

			/* Set Raw Temperature Value */
			raw_temp = ((uint32_t)temp_data[0] << 12) | ((uint32_t)temp_data[1] << 4) | ((uint32_t)temp_data[3] >> 4);
		}

		/* Forced Mode */
		if((ctrl_meas & 0x03) == 0x01 || (ctrl_meas & 0x03) == 0x02) {
			if(HAL_I2C_Mem_Read(bmp280->hi2c, bmp280->i2c_addr, BMP280_TEMP_REG, I2C_MEMADD_SIZE_8BIT, temp_data, 3, BMP280_TIMEOUT) != 0) {
				return 1;
			}

			/* Set Raw Temperature Value */
			raw_temp = ((uint32_t)temp_data[0] << 12) | ((uint32_t)temp_data[1] << 4) | ((uint32_t)temp_data[3] >> 4);

			/* Set Forced Mode to Sleep Mode */
			ctrl_meas &= (0x00);

			if(HAL_I2C_Mem_Write(bmp280->hi2c, bmp280->i2c_addr, BMP280_CTRL_MEAS_REG, I2C_MEMADD_SIZE_8BIT, &ctrl_meas, 1, BMP280_TIMEOUT) != 0) {
				return 1;
			}
		}

		/* Sleep Mode or Other Wrong Inputs */
		else {
			if(HAL_I2C_Mem_Read(bmp280->hi2c, bmp280->i2c_addr, BMP280_TEMP_REG, I2C_MEMADD_SIZE_8BIT, temp_data, 3, BMP280_TIMEOUT) != 0) {
				return 1;
			}

			/* Set Raw Temperature Value */
			raw_temp = ((uint32_t)temp_data[0] << 12) | ((uint32_t)temp_data[1] << 4) | ((uint32_t)temp_data[3] >> 4);
		}
	}

	else {
		if(HAL_SPI_TransmitReceive(bmp280->hspi, (uint8_t *)BMP280_CTRL_MEAS_REG, &ctrl_meas, 1, BMP280_TIMEOUT) != 0) {
			return 2;
		}

		/* Check device mode */

		/* Normal Mode */
		if((ctrl_meas & 0x03) == 0x03) {
			/* Read Raw Temperature Data from Sensor */
			if(HAL_SPI_TransmitReceive(bmp280->hspi, (uint8_t *)BMP280_TEMP_REG, temp_data, 3, BMP280_TIMEOUT) != 0) {
				return 1;
			}

			/* Set Raw Temperature Value */
			raw_temp = ((uint32_t)temp_data[0] << 12) | ((uint32_t)temp_data[1] << 4) | ((uint32_t)temp_data[3] >> 4);
		}

		/* Forced Mode */
		if((ctrl_meas & 0x03) == 0x01 || (ctrl_meas & 0x03) == 0x02) {
			/* Set Forced Mode */

			if(HAL_SPI_TransmitReceive(bmp280->hspi, (uint8_t *)BMP280_TEMP_REG, temp_data, 3, BMP280_TIMEOUT) != 0) {
				return 1;
			}

			/* Set Raw Temperature Value */
			raw_temp = ((uint32_t)temp_data[0] << 12) | ((uint32_t)temp_data[1] << 4) | ((uint32_t)temp_data[3] >> 4);

			/* Set Forced Mode to Sleep Mode */
			ctrl_meas &= (0x00);
			package[0] = BMP280_TEMP_REG;
			package[1] = ctrl_meas;

			if(HAL_SPI_Transmit(bmp280->hspi, package, 2, BMP280_TIMEOUT) != 0) {
				return 1;
			}
		}

		/* Sleep Mode */
		else {
			if(HAL_SPI_TransmitReceive(bmp280->hspi, (uint8_t *)BMP280_TEMP_REG, temp_data, 3, BMP280_TIMEOUT) != 0) {
				return 1;
			}

			/* Set Raw Temperature Value */
			raw_temp = ((uint32_t)temp_data[0] << 12) | ((uint32_t)temp_data[1] << 4) | ((uint32_t)temp_data[3] >> 4);
		}
	}

	/* Compansate Raw Temperature Value */
	err = compansate_temperature(bmp280, raw_temp, calib_data);
	if(err != 0) {
		return err;
	}

	return 0;
}

/*
 * @brief		gets pressure values from sensor's related registers
 * @param		bmp280 pointer to a bmp280 handle structure
 * @return		status code
 * 				- 0 success
 *				- 1 i2c memory read/write error
 * 				- 2 spi read/write error
 * 				- 3 compansate calculation error
 */
uint8_t get_pressure(bmp280_handle_t *bmp280, bmp280_calibration_t *calib_data) {
	uint8_t err;
	uint8_t ctrl_meas;
	uint8_t temp_data[3] = {0, 0, 0};
	uint32_t raw_press;
	uint8_t package[2];

	if(bmp280->comm_mode == BMP280_MODE_I2C) {
		if(HAL_I2C_Mem_Read(bmp280->hi2c, bmp280->i2c_addr, BMP280_CTRL_MEAS_REG, I2C_MEMADD_SIZE_8BIT, &ctrl_meas, 1, BMP280_TIMEOUT) != 0) {
			return 1;
		}

		/* Check device mode */

		/* Normal Mode */
		if((ctrl_meas & 0x03) == 0x03) {
			/* Read Raw Pressure Data from Sensor */
			if(HAL_I2C_Mem_Read(bmp280->hi2c, bmp280->i2c_addr, BMP280_PRESSURE_REG, I2C_MEMADD_SIZE_8BIT, temp_data, 3, BMP280_TIMEOUT) != 0) {
				return 1;
			}

			/* Set Raw Pressure Value */
			raw_press = ((uint32_t)temp_data[0] << 12) | ((uint32_t)temp_data[1] << 4) | ((uint32_t)temp_data[3] >> 4);
		}

		/* Forced Mode */
		if((ctrl_meas & 0x03) == 0x01 || (ctrl_meas & 0x03) == 0x02) {
			if(HAL_I2C_Mem_Read(bmp280->hi2c, bmp280->i2c_addr, BMP280_PRESSURE_REG, I2C_MEMADD_SIZE_8BIT, temp_data, 3, BMP280_TIMEOUT) != 0) {
				return 1;
			}

			/* Set Raw Pressure Value */
			raw_press = ((uint32_t)temp_data[0] << 12) | ((uint32_t)temp_data[1] << 4) | ((uint32_t)temp_data[3] >> 4);

			/* Set Forced Mode to Sleep Mode */
			ctrl_meas &= (0x00);

			if(HAL_I2C_Mem_Write(bmp280->hi2c, bmp280->i2c_addr, BMP280_CTRL_MEAS_REG, I2C_MEMADD_SIZE_8BIT, &ctrl_meas, 1, BMP280_TIMEOUT) != 0) {
				return 1;
			}
		}

		/* Sleep Mode */
		else {
			if(HAL_I2C_Mem_Read(bmp280->hi2c, bmp280->i2c_addr, BMP280_PRESSURE_REG, I2C_MEMADD_SIZE_8BIT, temp_data, 3, BMP280_TIMEOUT) != 0) {
				return 1;
			}

			/* Set Raw Pressure Value */
			raw_press = ((uint32_t)temp_data[0] << 12) | ((uint32_t)temp_data[1] << 4) | ((uint32_t)temp_data[3] >> 4);
		}
	}

	else {
		if(HAL_SPI_TransmitReceive(bmp280->hspi, (uint8_t *)BMP280_CTRL_MEAS_REG, &ctrl_meas, 1, BMP280_TIMEOUT) != 0) {
			return 2;
		}

		/* Check device mode */

		/* Normal Mode */
		if((ctrl_meas & 0x03) == 0x03) {
			/* Read Raw Pressure Data from Sensor */
			if(HAL_SPI_TransmitReceive(bmp280->hspi, (uint8_t *)BMP280_PRESSURE_REG, temp_data, 3, BMP280_TIMEOUT) != 0) {
				return 1;
			}

			/* Set Raw Pressure Value */
			raw_press = ((uint32_t)temp_data[0] << 12) | ((uint32_t)temp_data[1] << 4) | ((uint32_t)temp_data[3] >> 4);
		}

		/* Forced Mode */
		if((ctrl_meas & 0x03) == 0x01 || (ctrl_meas & 0x03) == 0x02) {
			/* Set Forced Mode */

			if(HAL_SPI_TransmitReceive(bmp280->hspi, (uint8_t *)BMP280_PRESSURE_REG, temp_data, 3, BMP280_TIMEOUT) != 0) {
				return 1;
			}

			/* Set Raw Pressure Value */
			raw_press = ((uint32_t)temp_data[0] << 12) | ((uint32_t)temp_data[1] << 4) | ((uint32_t)temp_data[3] >> 4);

			/* Set Forced Mode to Sleep Mode */
			ctrl_meas &= (0x00);
			package[0] = BMP280_TEMP_REG;
			package[1] = ctrl_meas;

			if(HAL_SPI_Transmit(bmp280->hspi, package, 2, BMP280_TIMEOUT) != 0) {
				return 1;
			}
		}

		/* Sleep Mode */
		else {
			if(HAL_SPI_TransmitReceive(bmp280->hspi, (uint8_t *)BMP280_PRESSURE_REG, temp_data, 3, BMP280_TIMEOUT) != 0) {
				return 1;
			}

			/* Set Raw Pressure Value */
			raw_press = ((uint32_t)temp_data[0] << 12) | ((uint32_t)temp_data[1] << 4) | ((uint32_t)temp_data[3] >> 4);
		}
	}

	/* Compansate Raw Pressure Value */
	err = compansate_pressure(bmp280, raw_press, calib_data);
	if(err != 0) {
		return err;
	}

	return 0;
}

/*
 * @brief		compansates raw uint32_t temperature value to fined version
 * @param		bmp280 pointer to a bmp280 handle structure
 * @param		raw_temp to raw temperature value from related registers
 * @return		status code
 * 				- 0 success
 *				- 1 compansate calculation error
 */
static uint8_t compansate_temperature(bmp280_handle_t *bmp280, uint32_t raw_temp, bmp280_calibration_t *calib_data) {

	float var1, var2, T;


	var1 = (((float)raw_temp) / 16384.0 - ((float)calib_data->t1) / 1024.0) * ((float)calib_data->t2);
	var2 = ((((float)raw_temp) / 131072.0 - ((float)calib_data->t1) / 8192.0) * (((float)raw_temp) / 131072.0 - ((float)calib_data->t1) / 8192.0)) * ((float)calib_data->t3);

	calib_data->t_fine = (int32_t)(var1 + var2);
	T = (var1 + var2) / 5120.0;

	/* Check is there any compansate error. Check p. 2 for sensor's temperature range */
	if(T < -40.0) {
		T = -40.0;
		bmp280->temperature = T;

		return 1;
	}

	if(T > 85.0) {
		T = 85.0;
		bmp280->temperature = T;

		return 1;
	}

	bmp280->temperature = T;

	return 0;

}

/*
 * @brief		compansates raw uint32_t pressure value to fined version
 * @param		bmp280 pointer to a bmp280 handle structure
 * @param		raw_press to raw pressure value from related registers
 * @return		status code
 * 				- 0 success
 *				- 1 compansate calculation error
 */
static uint8_t compansate_pressure(bmp280_handle_t *bmp280, uint32_t raw_press, bmp280_calibration_t *calib_data) {
	float var1, var2, P;

	var1 = ((float)calib_data->t_fine / 2.0) - 64000.0;
	var2 = var1 * var1 * ((float)calib_data->p6) / 32768.0;
	var2 = var2 + var1 * ((float)calib_data->p5) * 2.0;
	var2 = (var2 / 4.0) + (((float)calib_data->p4) * 65536.0);
	var1 = (((float)calib_data->p3) * var1 * var1 / 524288.0 + ((float)calib_data->p2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * ((float)calib_data->p1);

	/* Check var1 to avoid divison by zero exception */
	if(var1 == 0.0) {
		P = 0.0;
		bmp280->pressure = P;

		return 1;
	}

	P = 1048576.0 - (float)raw_press;
	P = (P - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((float)calib_data->p9) * P * P / 2147483648.0;
	var2 = P * ((float)calib_data->p8) / 32768.0;
	P = P + (var1 + var2 + ((float)calib_data->p7)) / 16.0;
	bmp280->pressure = P;

	return 0;
}


/*
 * @brief		get the calibration datas and write the related structure
 * @param		*bmp280 pointer to a bmp280 handle structure
 * @param		*calib_data pointer to a bmp280 calibration data structure
 * @return		status code
 * 				- 0 success
 *				- 1 i2c memory read error
 * 				- 2 spi write/read error
 * 				- 3 device id not matched
 */
static uint8_t get_calibration_datas(bmp280_handle_t *bmp280, bmp280_calibration_t *calib_data) {
	uint8_t data[2];

	if(bmp280->comm_mode == BMP280_MODE_I2C) {
		if(HAL_I2C_Mem_Read(bmp280->hi2c, bmp280->i2c_addr, BMP280_NVM_T1_LSB, I2C_MEMADD_SIZE_8BIT, data, 2, BMP280_TIMEOUT) != 0) {
			return 1;
		}
		calib_data->t1 = (uint16_t)data[1] << 8 | data[0];

		if(HAL_I2C_Mem_Read(bmp280->hi2c, bmp280->i2c_addr, BMP280_NVM_T2_LSB, I2C_MEMADD_SIZE_8BIT, data, 2, BMP280_TIMEOUT) != 0) {
			return 1;
		}
		calib_data->t2 = (int16_t)((uint16_t)data[1] << 8 | data[0]);

		if(HAL_I2C_Mem_Read(bmp280->hi2c, bmp280->i2c_addr, BMP280_NVM_T3_LSB, I2C_MEMADD_SIZE_8BIT, data, 2, BMP280_TIMEOUT) != 0) {
			return 1;
		}
		calib_data->t3 = (int16_t)((uint16_t)data[1] << 8 | data[0]);

		if(HAL_I2C_Mem_Read(bmp280->hi2c, bmp280->i2c_addr, BMP280_NVM_P1_LSB, I2C_MEMADD_SIZE_8BIT, data, 2, BMP280_TIMEOUT) != 0) {
			return 1;
		}
		calib_data->p1 = (int16_t)((uint16_t)data[1] << 8 | data[0]);

		if(HAL_I2C_Mem_Read(bmp280->hi2c, bmp280->i2c_addr, BMP280_NVM_P2_LSB, I2C_MEMADD_SIZE_8BIT, data, 2, BMP280_TIMEOUT) != 0) {
			return 1;
		}
		calib_data->p2 = (int16_t)((uint16_t)data[1] << 8 | data[0]);

		if(HAL_I2C_Mem_Read(bmp280->hi2c, bmp280->i2c_addr, BMP280_NVM_P3_LSB, I2C_MEMADD_SIZE_8BIT, data, 2, BMP280_TIMEOUT) != 0) {
			return 1;
		}
		calib_data->p3 = (int16_t)((uint16_t)data[1] << 8 | data[0]);

		if(HAL_I2C_Mem_Read(bmp280->hi2c, bmp280->i2c_addr, BMP280_NVM_P4_LSB, I2C_MEMADD_SIZE_8BIT, data, 2, BMP280_TIMEOUT) != 0) {
			return 1;
		}
		calib_data->p4 = (int16_t)((uint16_t)data[1] << 8 | data[0]);

		if(HAL_I2C_Mem_Read(bmp280->hi2c, bmp280->i2c_addr, BMP280_NVM_P5_LSB, I2C_MEMADD_SIZE_8BIT, data, 2, BMP280_TIMEOUT) != 0) {
			return 1;
		}
		calib_data->p5 = (int16_t)((uint16_t)data[1] << 8 | data[0]);

		if(HAL_I2C_Mem_Read(bmp280->hi2c, bmp280->i2c_addr, BMP280_NVM_P6_LSB, I2C_MEMADD_SIZE_8BIT, data, 2, BMP280_TIMEOUT) != 0) {
			return 1;
		}
		calib_data->p6 = (int16_t)((uint16_t)data[1] << 8 | data[0]);

		if(HAL_I2C_Mem_Read(bmp280->hi2c, bmp280->i2c_addr, BMP280_NVM_P7_LSB, I2C_MEMADD_SIZE_8BIT, data, 2, BMP280_TIMEOUT) != 0) {
			return 1;
		}
		calib_data->p7 = (int16_t)((uint16_t)data[1] << 8 | data[0]);

		if(HAL_I2C_Mem_Read(bmp280->hi2c, bmp280->i2c_addr, BMP280_NVM_P8_LSB, I2C_MEMADD_SIZE_8BIT, data, 2, BMP280_TIMEOUT) != 0) {
			return 1;
		}
		calib_data->p8 = (int16_t)((uint16_t)data[1] << 8 | data[0]);

		if(HAL_I2C_Mem_Read(bmp280->hi2c, bmp280->i2c_addr, BMP280_NVM_P9_LSB, I2C_MEMADD_SIZE_8BIT, data, 2, BMP280_TIMEOUT) != 0) {
			return 1;
		}
		calib_data->p9 = (int16_t)((uint16_t)data[1] << 8 | data[0]);
	}

	else {
		if(HAL_SPI_TransmitReceive(bmp280->hspi, (uint8_t *)BMP280_NVM_T1_LSB, data, 2, BMP280_TIMEOUT) != 0) {
			return 1;
		}
		calib_data->t1 = (uint16_t)data[1] << 8 | data[0];

		if(HAL_SPI_TransmitReceive(bmp280->hspi, (uint8_t *)BMP280_NVM_T2_LSB, data, 2, BMP280_TIMEOUT) != 0) {
			return 1;
		}
		calib_data->t2 = (int16_t)((uint16_t)data[1] << 8 | data[0]);

		if(HAL_SPI_TransmitReceive(bmp280->hspi, (uint8_t *)BMP280_NVM_T3_LSB, data, 2, BMP280_TIMEOUT) != 0) {
			return 1;
		}
		calib_data->t3 = (int16_t)((uint16_t)data[1] << 8 | data[0]);

		if(HAL_SPI_TransmitReceive(bmp280->hspi, (uint8_t *)BMP280_NVM_P1_LSB, data, 2, BMP280_TIMEOUT) != 0) {
			return 1;
		}
		calib_data->p1 = (int16_t)((uint16_t)data[1] << 8 | data[0]);

		if(HAL_SPI_TransmitReceive(bmp280->hspi, (uint8_t *)BMP280_NVM_P2_LSB, data, 2, BMP280_TIMEOUT) != 0) {
			return 1;
		}
		calib_data->p2 = (int16_t)((uint16_t)data[1] << 8 | data[0]);

		if(HAL_SPI_TransmitReceive(bmp280->hspi, (uint8_t *)BMP280_NVM_P3_LSB, data, 2, BMP280_TIMEOUT) != 0) {
			return 1;
		}
		calib_data->p3 = (int16_t)((uint16_t)data[1] << 8 | data[0]);

		if(HAL_SPI_TransmitReceive(bmp280->hspi, (uint8_t *)BMP280_NVM_P4_LSB, data, 2, BMP280_TIMEOUT) != 0) {
			return 1;
		}
		calib_data->p4 = (int16_t)((uint16_t)data[1] << 8 | data[0]);

		if(HAL_SPI_TransmitReceive(bmp280->hspi, (uint8_t *)BMP280_NVM_P5_LSB, data, 2, BMP280_TIMEOUT) != 0) {
			return 1;
		}
		calib_data->p5 = (int16_t)((uint16_t)data[1] << 8 | data[0]);

		if(HAL_SPI_TransmitReceive(bmp280->hspi, (uint8_t *)BMP280_NVM_P6_LSB, data, 2, BMP280_TIMEOUT) != 0) {
			return 1;
		}
		calib_data->p6 = (int16_t)((uint16_t)data[1] << 8 | data[0]);

		if(HAL_SPI_TransmitReceive(bmp280->hspi, (uint8_t *)BMP280_NVM_P7_LSB, data, 2, BMP280_TIMEOUT) != 0) {
			return 1;
		}
		calib_data->p7 = (int16_t)((uint16_t)data[1] << 8 | data[0]);

		if(HAL_SPI_TransmitReceive(bmp280->hspi, (uint8_t *)BMP280_NVM_P8_LSB, data, 2, BMP280_TIMEOUT) != 0) {
			return 1;
		}
		calib_data->p8 = (int16_t)((uint16_t)data[1] << 8 | data[0]);

		if(HAL_SPI_TransmitReceive(bmp280->hspi, (uint8_t *)BMP280_NVM_P9_LSB, data, 2, BMP280_TIMEOUT) != 0) {
			return 1;
		}
		calib_data->p9 = (int16_t)((uint16_t)data[1] << 8 | data[0]);
	}

	return 0;
}


