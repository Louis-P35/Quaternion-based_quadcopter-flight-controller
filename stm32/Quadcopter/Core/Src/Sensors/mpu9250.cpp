/*
 * mpu9250.cpp
 *
 *  Created on: Jun 11, 2024
 *      Author: louis
 */

// STL
#include <math.h>

// Project
#include "Sensors/mpu9250.hpp"



#define RAD_TO_DEGREE 57.295779513082321


/*
 * The constructor initialize the SPI
 * related stuff.
 * hspi: The handle of the SPI device
 * spi_cs_pin: The chip select pin
 * spi_cs_gpio_port: The GPIO port
 */
MPU9250::MPU9250(SPI_HandleTypeDef hspi, uint16_t spi_cs_pin, GPIO_TypeDef* spi_cs_gpio_port) :
m_hspi(hspi), m_spi_cs_pin(spi_cs_pin), m_spi_cs_gpio_port(spi_cs_gpio_port)
{

}


/*
 * This method communicate with the sensor,
 * configure and calibrate it.
 */
void MPU9250::init(const AccScale& accScaleConf, const GyroScale& gyroScaleConf)
{
	HAL_Delay(10);

	// Configure the IMU sensitivity
	configureAccelerometer(accScaleConf);

	HAL_Delay(10);

	configureGyroscope(gyroScaleConf);

	// 10 ms delay in case the MPU need time to update to its new calibration
	HAL_Delay(10);

	// Calibrate accelerometer and gyroscope offsets
	calibrate();

	HAL_Delay(10);
}


/*
 * Setup the accelerometer scale.
 * Values are: 2g, 4g, 8g or 16g
 */
void MPU9250::configureAccelerometer(const AccScale& accScaleConf)
{
	m_accScaleConf = accScaleConf;

	// Register 28 is accelerometer configuration
	switch(m_accScaleConf)
	{
		case AccScale::_2G:
			// Write 00 on bits [4:3] for +-2g configuration
			write_register(28, 0b00000000);
			// Configure the scaler value
			m_accScale = 16384.0;
			break;

		case AccScale::_4G:
			// Write 01 on bits [4:3] for +-4g configuration
			write_register(28, 0b00001000);
			// Configure the scaler value
			m_accScale = 8192.0;
			break;

		case AccScale::_8G:
			// Write 10 on bits [4:3] for +-8g configuration
			write_register(28, 0b00010000);
			// Configure the scaler value
			m_accScale = 4096.0;
			break;

		case AccScale::_16G:
			// Write 11 on bits [4:3] for +-16g configuration
			write_register(28, 0b00011000);
			// Configure the scaler value
			m_accScale = 2048.0;
			break;

		default:
			// Default values will be use
			// TODO: log error
			break;
	}
}


/*
 * Setup the gyroscope scale.
 * Values are: 250 dps, 500 dps, 1000 dps or 2000 dps
 */
void MPU9250::configureGyroscope(const GyroScale& gyroScaleConf)
{
	m_gyroScaleConf = gyroScaleConf;

	// Register 27 is gyroscope configuration
	switch(m_gyroScaleConf)
	{
		case GyroScale::DPS250:
			// Write 00 on bits [4:3] for 250 dps configuration
			write_register(27, 0b00000000);
			// Configure the scaler value
			m_gyroScale = 131.0;
			break;

		case GyroScale::DPS500:
			// Write 01 on bits [4:3] for 500 dps configuration
			write_register(27, 0b00001000);
			// Configure the scaler value
			m_gyroScale = 131.0;
			break;

		case GyroScale::DPS1000:
			// Write 10 on bits [4:3] for 1000 dps configuration
			write_register(27, 0b00010000);
			// Configure the scaler value
			m_gyroScale = 131.0;
			break;

		case GyroScale::DPS2000:
			// Write 11 on bits [4:3] for 2000 dps configuration
			write_register(27, 0b00011000);
			// Configure the scaler value
			m_gyroScale = 131.0;
			break;

		default:
			// Default values will be use
			// TODO: log error
			break;
	}
}



/*
 * This method handle SPI errors
 */
void MPU9250::handle_spi_error(HAL_StatusTypeDef result)
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
void MPU9250::read_register(uint8_t regAddr, uint8_t* pData, uint8_t len)
{
	HAL_StatusTypeDef result;

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
void MPU9250::write_register(uint8_t regAddr, uint8_t data)
{
	HAL_StatusTypeDef result;

	// MSB is a read/write bit and the 7 LSB are the register
	// regAddr already have the MSB to 0

	// Set CS to low, selecting the chip
	HAL_GPIO_WritePin(m_spi_cs_gpio_port, m_spi_cs_pin, GPIO_PIN_RESET);
	// Write the write bit and register to write in
	result = HAL_SPI_Transmit(&m_hspi, &regAddr, 1, 100);
	if(result != HAL_OK)
	{
		handle_spi_error(result);
	}
	// Write data
	result = HAL_SPI_Transmit(&m_hspi, &data, 1, 100);
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
void MPU9250::read_gyro_acc_data()
{
	uint8_t data[14];

	// Accelerometer data are stored from register 59 to 64
	// Temperature data are stored from register 65 to 66
	// Gyroscope data are stored from register 67 to 72
	// So read 14 bytes from register 59 to 72
	read_register(59, data, sizeof(data));

	// Accelerometer high byte and low byte combination
	m_rawAcc(0) = (static_cast<int16_t>(data[0]) << 8) + data[1];
	m_rawAcc(1) = (static_cast<int16_t>(data[2]) << 8) + data[3];
	m_rawAcc(2) = (static_cast<int16_t>(data[4]) << 8) + data[5];
	// Scale
	m_rawScaledAcc(0) = static_cast<double>(m_rawAcc(0)) / m_accScale;
	m_rawScaledAcc(1) = static_cast<double>(m_rawAcc(1)) / m_accScale;
	m_rawScaledAcc(2) = static_cast<double>(m_rawAcc(2)) / m_accScale;

	// Temperature high byte and low byte combination
	m_rawTemp = ((int16_t)data[6] << 8) + data[7];

	// Gyroscope high byte and low byte combination
	m_rawGyro(0) = (static_cast<int16_t>(data[8]) << 8) + data[9];
	m_rawGyro(1) = (static_cast<int16_t>(data[10]) << 8) + data[11];
	m_rawGyro(2) = (static_cast<int16_t>(data[12]) << 8) + data[13];
	// Scale
	m_rawScaledGyro(0) = static_cast<double>(m_rawGyro(0)) / m_gyroScale;
	m_rawScaledGyro(1) = static_cast<double>(m_rawGyro(1)) / m_gyroScale;
	m_rawScaledGyro(2) = static_cast<double>(m_rawGyro(2)) / m_gyroScale;
}

/*
 * Read the 3 axis magnetometer of the MPU9250
 * through the SPI interface
 */
void MPU9250::read_magnetometer_data()
{

}


/*
 * Compute the offset of the accelerometer and
 * gyroscope by averaging a large number of reading.
 */
void MPU9250::calibrate()
{
	// Initialize offsets
	m_accOffset = {0.0, 0.0, 0.0};
	m_gyroOffset = {0.0, 0.0, 0.0};

	const int range = 1000; // Number of reading

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
}


/*
 * Apply a low pass filter on the data.
 * Remove the offset found during calibration.
 */
void MPU9250::filter_and_calibrate_data()
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

	m_filteredGyro = m_rawScaledGyro - m_gyroOffset;
}


