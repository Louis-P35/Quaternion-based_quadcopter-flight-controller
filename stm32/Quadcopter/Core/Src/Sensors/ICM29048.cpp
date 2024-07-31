/*
 * ICM29048.cpp
 *
 *  Created on: Jul 20, 2024
 *      Author: Louis
 */

// STL
#include <math.h>

// Project
#include "Sensors/ICM29048.hpp"



#define RAD_TO_DEGREE 57.295779513082321


/*
 * The constructor initialize the SPI
 * related stuff.
 * hspi: The handle of the SPI device
 * spi_cs_pin: The chip select pin
 * spi_cs_gpio_port: The GPIO port
 */
ICM29048::ICM29048(SPI_HandleTypeDef hspi, uint16_t spi_cs_pin, GPIO_TypeDef* spi_cs_gpio_port) :
m_hspi(hspi), m_spi_cs_pin(spi_cs_pin), m_spi_cs_gpio_port(spi_cs_gpio_port)
{

}



/*
 * Reset the master I2C
 */
void ICM29048::reset_i2c_master_magnetometer()
{
	uint8_t reg;

	// I2C master reset
	read_register(0, ICM29048_USER_CTRL, &reg, 1);
	reg |= 0x02;
	write_register(0, ICM29048_USER_CTRL, reg);
	HAL_Delay(100);
}


/*
 * Init the magnetometer.
 *
 * The AK8963 magnetometer inside the ICM20948 can be accessed via the SPI interface
 * of the MPU9250. However, the communication between the ICM20948 and the AK8963 internally
 * uses I2C.
 * When using SPI to interface with the MPU9250, the magnetometer data are accessed through
 * the MPU9250's registers. The MPU9250 acts as an intermediary that handles the I2C
 * communication with the AK8963 magnetometer and exposes the magnetometer data through
 * its own SPI interface.
 */
void ICM29048::init_magnetometer()
{
	HAL_Delay(10);

	uint8_t reg;

	// I2C master reset
	reset_i2c_master_magnetometer();
	// I2C master enable
	read_register(0, ICM29048_USER_CTRL, &reg, 1);
	reg |= 0x20;
	write_register(0, ICM29048_USER_CTRL, reg);
	HAL_Delay(10);

	// I2C master clock 400 KHz
	write_register(3, ICM20948_I2C_MST_CTRL, 0x07);
	HAL_Delay(10);

	//Set data rate, around 136 Hz
	write_register(0, ICM29048_LP_CONFIG, 0x40);
	HAL_Delay(10);
	write_register(3, ICM29048_I2C_MST_ODR_CONFIG, 0x03);
	HAL_Delay(10);


	// Reset the magnetometer
	ak09916_write(AK09916_CNTL3, 0x01);
	HAL_Delay(100);

	// Read the "Who Am I" register
	/*reg = read_ak8963_who_am_i();
	if (reg != 0x09)
	{
		// TODO: Handle error
	}*/

	// Set the magnetometer to continuous measurement mode
	ak09916_write(AK09916_CNTL2, 0x08);
	HAL_Delay(100);
	// Reading test
	ak09916_read(AK09916_CNTL2, 1);
	read_register(0, ICM29048_EXT_SLV_SENS_DATA_00, &reg, 1);
	HAL_Delay(10);

	// Start reading data
	// Read 8 bytes: 6 for 3 axis data + 2 for status (otherwise it will not sample next data)
	// It will now sample data automatically from here
	ak09916_read(AK09916_HXL, 8);
	select_user_bank(0);
}

void ICM29048::ak09916_write(uint8_t reg, uint8_t data)
{
	write_register(3, ICM20948_I2C_SLV0_ADDR, AK09916_I2C_ADDR);
	write_register(3, ICM20948_I2C_SLV0_REG, reg);
	write_register(3, ICM20948_I2C_SLV0_DO, data);
	// Enable a signe byte of data write
	// Set the least significant bit
	write_register(3, ICM20948_I2C_SLV0_CTRL, 0x80 | 0x01);
	HAL_Delay(50);
}

void ICM29048::ak09916_read(uint8_t reg, uint8_t len)
{
	write_register(3, ICM20948_I2C_SLV0_ADDR, 0x80 | AK09916_I2C_ADDR);
	write_register(3, ICM20948_I2C_SLV0_REG, reg);

	// Number of bytes to be read
	write_register(3, ICM20948_I2C_SLV0_CTRL, 0x80 | len);
	HAL_Delay(50);

	// Data stored in ICM20948_EXT_SLV_SENS_DATA_00 register
}


/*
 * This method communicate with the sensor,
 * configure and calibrate it.
 */
void ICM29048::init(const AccScale& accScaleConf, const GyroScale& gyroScaleConf)
{
	// First, read the "who am I" register
	uint8_t who_am_i = read_icm29048_who_am_i();
	if (who_am_i != 234)
	{
		// TODO: Handle error
	}

	// Reset the device to ensure we always start in the same configuration
	resetDevice();

	// Set clock source and enable accelerometer and gyroscope
	configurePowerManagement();

	// Configure the IMU sensitivity
	configureAccelerometer(accScaleConf);
	configureGyroscope(gyroScaleConf);

	// Init magnetometer
	init_magnetometer();

	// Read the Who Am I register
	who_am_i = read_ak8963_who_am_i();
	if (who_am_i != 0x09)
	{
		// TODO: Handle error
	}



	// Calibrate accelerometer and gyroscope offsets
	calibrate();
}


/*
 * Set the clock source to internal oscillator for the IMU
 * Enable accelerometer and gyroscope.
 */
void ICM29048::configurePowerManagement()
{
    // Set clock source to PLL and enable accelerometer and gyroscope
    uint8_t pwr_mgmt_1_value = 0x01; // Set clock source to PLL
    write_register(0, ICM29048_PWR_MGMT_1, pwr_mgmt_1_value);
    HAL_Delay(10);

    // Enable accelerometer and gyroscope
    uint8_t pwr_mgmt_2_value = 0x00; // Enable both accelerometer and gyroscope
    write_register(0, ICM29048_PWR_MGMT_2, pwr_mgmt_2_value);
    HAL_Delay(10);

    // Verify configuration
    uint8_t regValue = 0;
    read_register(0, ICM29048_PWR_MGMT_1, &regValue, 1);
    if (regValue != pwr_mgmt_1_value)
    {
        // TODO: Handle error
    }
    read_register(0, ICM29048_PWR_MGMT_2, &regValue, 1);
    if (regValue != pwr_mgmt_2_value)
    {
        // TODO: Handle error
    }
}


/*
 * Reset all registers of the ICM-29048
 */
void ICM29048::resetDevice()
{
	uint8_t resetCommand = 0x80; // Set DEVICE_RESET bit (bit 7) in PWR_MGMT_1

	// Write the reset command to PWR_MGMT_1
	write_register(0, ICM29048_PWR_MGMT_1, resetCommand);

	// Wait for the reset to complete
	HAL_Delay(100); // Delay for 100ms to ensure the device has time to reset

	// Confirm reset by reading the PWR_MGMT_1 register
	uint8_t regValue = 0;
	read_register(0, ICM29048_PWR_MGMT_1, &regValue, 1);
	if (regValue != 0x41)
	{
		//TODO: Handle error
	}
}


/*
 * Select a different registers bank in the chip
 */
void ICM29048::select_user_bank(uint8_t bank)
{
	// Return if the right bank is already selected
	if (m_currentBank == bank)
	{
		return;
	}

	uint8_t bankCommand = (bank << 4) & 0x30; // Only the upper two bits are used to select the bank

	// ICM29048_REG_BANK_SEL is accessible from any bank
	// Using m_currentBank to avoid infinite loop
    write_register(m_currentBank, ICM29048_REG_BANK_SEL, bankCommand);
    HAL_Delay(10); // Small delay to ensure bank switch TODO remove that

    // Confirm bank switch by reading the REG_BANK_SEL register
	uint8_t regValue = 0;
	read_register(m_currentBank, ICM29048_REG_BANK_SEL, &regValue, 1);
	if (regValue != bankCommand)
	{
		// TODO: Handle error
		return;
	}

	m_currentBank = bank;
}


/*
 * Setup the accelerometer scale.
 * Values are: 2g, 4g, 8g or 16g
 */
void ICM29048::configureAccelerometer(const AccScale& accScaleConf)
{
	m_accScaleConf = accScaleConf;

	HAL_Delay(10);

	uint8_t accelConfig1Value = 0;

	// Read the current value of ACCEL_CONFIG_1
	read_register(2, ICM29048_ACCEL_CONFIG_1, &accelConfig1Value, 1);

	// Clear the FS_SEL bits [2:1] and ACCEL_FCHOICE bit [0]
	accelConfig1Value &= ~0b00000111;

	// Register 28 is accelerometer configuration
	switch(m_accScaleConf)
	{
		case AccScale::_2G:
			// Write 00 on bits [4:3] for +-2g configuration
			accelConfig1Value |= (0b00 << 1);
			// Configure the scaler value
			m_accScale = 16384.0;
			break;

		case AccScale::_4G:
			// Write 01 on bits [4:3] for +-4g configuration
			accelConfig1Value |= (0b01 << 1);
			// Configure the scaler value
			m_accScale = 8192.0;
			break;

		case AccScale::_8G:
			// Write 10 on bits [4:3] for +-8g configuration
			accelConfig1Value |= (0b10 << 1);
			// Configure the scaler value
			m_accScale = 4096.0;
			break;

		case AccScale::_16G:
			// Write 11 on bits [4:3] for +-16g configuration
			accelConfig1Value |= (0b11 << 1);
			// Configure the scaler value
			m_accScale = 2048.0;
			break;

		default:
			// Default values will be use
			// TODO: log error
			break;
	}

	// Ensure DLPF is enabled (ACCEL_FCHOICE = 1)
	accelConfig1Value |= 0x01;

	// Write back the new configuration to GYRO_CONFIG_1
	write_register(2, ICM29048_ACCEL_CONFIG_1, accelConfig1Value);

	HAL_Delay(10);

	// Verify configuration
	uint8_t regValue = 0;
	read_register(2, ICM29048_ACCEL_CONFIG_1, &regValue, 1);
	if (regValue != accelConfig1Value)
	{
		// TODO: Handle error
	}

	HAL_Delay(10);
}


/*
 * Setup the gyroscope scale.
 * Values are: 250 dps, 500 dps, 1000 dps or 2000 dps
 */
void ICM29048::configureGyroscope(const GyroScale& gyroScaleConf)
{
	m_gyroScaleConf = gyroScaleConf;

	HAL_Delay(10);

	uint8_t gyroConfig1Value = 0;

	// Read the current value of GYRO_CONFIG_1
	read_register(2, ICM29048_GYRO_CONFIG_1, &gyroConfig1Value, 1);

	// Clear the FS_SEL bits [2:1] and GYRO_FCHOICE bit [0]
	gyroConfig1Value &= ~0b00000111;

	// Register 27 is gyroscope configuration
	switch(m_gyroScaleConf)
	{
		case GyroScale::DPS250:
			// Write 00 on bits [4:3] for 250 dps configuration
			gyroConfig1Value |= (0b00 << 1);
			// Configure the scaler value
			m_gyroScale = 131.0;
			break;

		case GyroScale::DPS500:
			// Write 01 on bits [4:3] for 500 dps configuration
			gyroConfig1Value |= (0b01 << 1);
			// Configure the scaler value
			m_gyroScale = 65.5;
			break;

		case GyroScale::DPS1000:
			// Write 10 on bits [4:3] for 1000 dps configuration
			gyroConfig1Value |= (0b10 << 1);
			// Configure the scaler value
			m_gyroScale = 32.8;
			break;

		case GyroScale::DPS2000:
			// Write 11 on bits [4:3] for 2000 dps configuration
			gyroConfig1Value |= (0b11 << 1);
			// Configure the scaler value
			m_gyroScale = 16.4;
			break;

		default:
			// Default values will be use
			// TODO: log error
			break;
	}

	// Ensure DLPF is enabled (GYRO_FCHOICE = 1)
	gyroConfig1Value |= 0x01;

	// Write back the new configuration to GYRO_CONFIG_1
	write_register(2, ICM29048_GYRO_CONFIG_1, gyroConfig1Value);

	HAL_Delay(20);

	// Verify configuration
	uint8_t regValue = 0;
	read_register(2, ICM29048_GYRO_CONFIG_1, &regValue, 1);
	if (regValue != gyroConfig1Value)
	{
		// TODO: Handle error
	}

	HAL_Delay(10);
}



/*
 * This method handle SPI errors
 */
void ICM29048::handle_spi_error(HAL_StatusTypeDef result)
{
	if(result != HAL_OK)
	{
		// TODO: Log the error on SD card

		if(result == HAL_TIMEOUT)
		{
			// Handling timeout
			// Reset SPI
			HAL_SPI_DeInit(&m_hspi);
			HAL_SPI_Init(&m_hspi);
		}
		else
		{
			// Handling other errors
		}
	}
}


/*
 * Read 'len' byte on the register 'regAddr' of the MPU9250
 * through the SPI interface.
 */
void ICM29048::read_register(const uint8_t bank, uint8_t regAddr, uint8_t* pData, uint8_t len)
{
	HAL_StatusTypeDef result;

	// First select the right bank if not already
	select_user_bank(bank);

	// MSB is a read/write bit and the 7 LSB are the register
	// 0b10000000 = 0x80 for read operation
	uint8_t cmd = 0x80 | regAddr;
	// Set CS to low, selecting the chip
	HAL_GPIO_WritePin(m_spi_cs_gpio_port, m_spi_cs_pin, GPIO_PIN_RESET);
	// Write the read bit and register to read
	result = HAL_SPI_Transmit(&m_hspi, &cmd, 1, 100);
	if(result != HAL_OK)
	{
		handle_spi_error(result);
	}
	// Read 'len' byte
	result = HAL_SPI_Receive(&m_hspi, pData, len, 100);
	if(result != HAL_OK)
	{
		handle_spi_error(result);
	}
	// Set CS to high, de-selecting the chip
	HAL_GPIO_WritePin(m_spi_cs_gpio_port, m_spi_cs_pin, GPIO_PIN_SET);
}


/*
 * Write data on the register 'regAddr' of the MPU9250
 * through the SPI interface.
 */
void ICM29048::write_register(const uint8_t bank, uint8_t regAddr, uint8_t data)
{
	HAL_StatusTypeDef result;

	// First select the right bank if not already
	select_user_bank(bank);

	// MSB is a read/write bit and the 7 LSB are the register
	// regAddr already have the MSB to 0

	// 0x7F = Ob01111111
	regAddr = regAddr & 0x7F;
	uint8_t txData[2] = {regAddr, data}; // Ensure MSB is cleared for write operation

	// Set CS to low, selecting the chip
	HAL_GPIO_WritePin(m_spi_cs_gpio_port, m_spi_cs_pin, GPIO_PIN_RESET);
	// Write the write bit and register to write in
	result = HAL_SPI_Transmit(&m_hspi, txData, 2, 100);
	if(result != HAL_OK)
	{
		handle_spi_error(result);
	}

	// Set CS to high, de-selecting the chip
	HAL_GPIO_WritePin(m_spi_cs_gpio_port, m_spi_cs_pin, GPIO_PIN_SET);
}


/*
 * Read the 3 axis accelerometer of the MPU9250
 * and the 3 axis gyroscope of the MPU9250
 * through the SPI interface
 */
void ICM29048::read_gyro_acc_data()
{
	const uint8_t nbByte = 22; // accel, gyro, temp, mag
	uint8_t data[nbByte];

	// Read 12 bytes of data starting from ACCEL_XOUT_H (0x2D) to GYRO_ZOUT_L (0x38)
	read_register(0, ICM29048_ACCEL_XOUT_H, data, nbByte);

	// Accelerometer high byte and low byte combination
	m_rawAcc(0) = (static_cast<int16_t>(data[0]) << 8) + data[1];
	m_rawAcc(1) = (static_cast<int16_t>(data[2]) << 8) + data[3];
	m_rawAcc(2) = (static_cast<int16_t>(data[4]) << 8) + data[5];
	// Scale
	m_rawScaledAcc(0) = static_cast<double>(m_rawAcc(0)) / m_accScale;
	m_rawScaledAcc(1) = static_cast<double>(m_rawAcc(1)) / m_accScale;
	m_rawScaledAcc(2) = static_cast<double>(m_rawAcc(2)) / m_accScale;

	// Gyroscope high byte and low byte combination
	m_rawGyro(0) = (static_cast<int16_t>(data[6]) << 8) + data[7];
	m_rawGyro(1) = (static_cast<int16_t>(data[8]) << 8) + data[9];
	m_rawGyro(2) = (static_cast<int16_t>(data[10]) << 8) + data[11];
	// Scale
	m_rawScaledGyro(0) = static_cast<double>(m_rawGyro(0)) / m_gyroScale;
	m_rawScaledGyro(1) = static_cast<double>(m_rawGyro(1)) / m_gyroScale;
	m_rawScaledGyro(2) = static_cast<double>(m_rawGyro(2)) / m_gyroScale;
	//m_rawScaledGyro *= -1.0; // To match accelerometer 'direction'

	// Temperature high byte and low byte combination
	//m_rawTemp = ((int16_t)data[6] << 8) + data[7];

	// Magnetometer
	// Combine high and low bytes
	m_rawMag(0) = (static_cast<int16_t>(data[15]) << 8) | data[14];
	m_rawMag(1) = (static_cast<int16_t>(data[17]) << 8) | data[16];
	m_rawMag(2) = (static_cast<int16_t>(data[19]) << 8) | data[18];

	// Apply scale factor
	m_rawScaledMag(0) = static_cast<double>(m_rawMag(0)) * m_magScale;
	m_rawScaledMag(1) = static_cast<double>(m_rawMag(1)) * m_magScale;
	m_rawScaledMag(2) = static_cast<double>(m_rawMag(2)) * m_magScale;
}



/*
 * configure the I2C slave settings for the ICM-20948 sensor, enabling it
 * to communicate with external I2C devices (such as the AK09916 magnetometer)
 * through the ICM-20948's auxiliary I2C bus.
 */
void ICM29048::setup_i2c_slave(uint8_t address, uint8_t reg, uint8_t length, bool read)
{
	write_register(3, ICM20948_I2C_SLV0_ADDR, address | (read ? 0x80 : 0x00));
	write_register(3, ICM20948_I2C_SLV0_REG, reg);
	write_register(3, ICM20948_I2C_SLV0_CTRL, 0x80 | length);
}


uint8_t ICM29048::read_icm29048_who_am_i()
{
    uint8_t who_am_i;

    read_register(0, ICM29048_WHO_AM_I, &who_am_i, 1);

    return who_am_i;
}

uint8_t ICM29048::read_ak8963_who_am_i()
{
	uint8_t who_am_i;

	// Set up I2C slave for reading WHO_AM_I register from AK09916
	setup_i2c_slave(AK09916_I2C_ADDR, AK09916_WHO_AM_I, 1, true);

	HAL_Delay(10); // Wait for the I2C transaction to complete

	// Read the WHO_AM_I register value
	read_register(0, ICM29048_EXT_SLV_SENS_DATA_00, &who_am_i, 1);

	return who_am_i;
}



/*
 * Compute the offset of the accelerometer and
 * gyroscope by averaging a large number of reading.
 */
void ICM29048::calibrate()
{
	// Initialize offsets
	m_accOffset = {0.0, 0.0, 0.0};
	m_gyroOffset = {0.0, 0.0, 0.0};

	const int range = 100; // Number of reading

	HAL_Delay(10);

	for (int i = 0; i < range; ++i)
	{
		// Read IMU
		read_gyro_acc_data();

		// Sum all readings
		// For accelerometer
		m_accOffset += m_rawScaledAcc;

		// For gyroscope
		m_gyroOffset += m_rawScaledGyro;

		HAL_Delay(1);
	}

	/* Divide the sum to get the average value */
	// For accelerometer
	m_accOffset /= static_cast<double>(range);

	// For gyroscope
	m_gyroOffset /= static_cast<double>(range);

	// Find the g vector and normalize it
	const Eigen::Vector3d normalizedAcc = m_accOffset.normalized();

	// Remove the gravity vector from the offset
	m_accOffset -= normalizedAcc;

	HAL_Delay(10);
}


/*
 * Apply a low pass filter on the data.
 * Remove the offset found during calibration.
 */
void ICM29048::filter_and_calibrate_data()
{
	// Filtering accelerometer data
	m_filteredAcceloremeter = m_previousAcc * m_lpf_acc_gain + m_rawScaledAcc * (1.0 - m_lpf_acc_gain);
	m_previousAcc = m_filteredAcceloremeter;

	// Remove offset
	m_filteredAcceloremeter -= m_accOffset;

	// Filtering gyroscope data
	m_filteredGyro = m_previousGyro * m_lpf_gyro_gain + m_rawScaledGyro * (1.0 - m_lpf_gyro_gain);
	m_previousGyro = m_filteredGyro;

	// Remove offset
	m_filteredGyro -= m_gyroOffset;
}




