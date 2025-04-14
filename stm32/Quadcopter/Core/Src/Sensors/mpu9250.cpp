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
 * Init the magnetometer of the mpu9250.
 *
 * The AK8963 magnetometer inside the MPU9250 can be accessed via the SPI interface
 * of the MPU9250. However, the communication between the MPU9250 and the AK8963 internally
 * uses I2C.
 * When using SPI to interface with the MPU9250, the magnetometer data are accessed through
 * the MPU9250's registers. The MPU9250 acts as an intermediary that handles the I2C
 * communication with the AK8963 magnetometer and exposes the magnetometer data through
 * its own SPI interface.
 */
void MPU9250::init_magnetometer()
{
	HAL_Delay(10);

	// I2C Master mode
	//write_register(m_mpu9250_USER_CTRL, 0x20);
	//HAL_Delay(10);

	// Configure MPU9250 I2C frequency 400KHz
	write_register(m_mpu9250_I2C_MST_CTRL, 0x0D);
	HAL_Delay(10);

	//Set the I2C slave addres of AK8963 and set for write.
	/*write_register(m_mpu9250_I2C_SLV0_ADDR, 0x0C);
	HAL_Delay(10);

	//I2C slave 0 register address from where to begin data transfer
	write_register(m_mpu9250_I2C_SLV0_REG, 0x0B);
	HAL_Delay(10);

	// Reset AK8963
	write_register(m_mpu9250_I2C_SLV0_DO, 0x01);
	HAL_Delay(10);
	//Enable I2C and set 1 byte
	write_register(m_mpu9250_I2C_SLV0_CTRL, 0x81);
	HAL_Delay(10);*/

	//I2C slave 0 register address from where to begin data transfer
	/*write_register(m_mpu9250_I2C_SLV0_REG, 0x0A);
	HAL_Delay(10);
	// Register value to continuous measurement in 16bit
	write_register(m_mpu9250_I2C_SLV0_DO, 0x12);
	HAL_Delay(10);
	//Enable I2C and set 1 byte
	write_register(m_mpu9250_I2C_SLV0_CTRL, 0x81);
	HAL_Delay(10);*/
	setup_i2c_slave(0x0C, m_ak8963_CNTL2, 1, false);
	write_register(m_mpu9250_I2C_SLV0_DO, 0x1);
	HAL_Delay(100);

	setup_i2c_slave(0x0C, m_ak8963_CNTL1, 1, false);
	write_register(m_mpu9250_I2C_SLV0_DO, 0x16);

	// Enable MPU9250 internal I2C bus
	/*uint8_t USER_CTRL_reg;
	read_register(m_mpu9250_USER_CTRL, &USER_CTRL_reg, 1);
	USER_CTRL_reg |= 0x20;
	write_register(m_mpu9250_USER_CTRL, USER_CTRL_reg);

	uint8_t USER_CTRL_reg2;
	read_register(m_mpu9250_USER_CTRL, &USER_CTRL_reg2, 1);

	// Configure MPU9250 I2C frequency 400KHz
	write_register(m_mpu9250_I2C_MST_CTRL, 0x0D);

	// reset the ak8963 in the mpu9250
	write_register(m_ak8963_CNTL2, 0x01);
	HAL_Delay(100);*/

    // Set the I2C master mode to pass through mode
    /*write_register(m_mpu9250_I2C_MST_CTRL, 0x0D); // I2C Master mode and 400 kHz
    HAL_Delay(10);

    // Enable bypass mode to allow direct access to the AK8963 registers through the MPU9250
    write_register(0x37, 0x02); // INT_PIN_CFG register: Enable bypass mode
    HAL_Delay(10);

    // Reset the magnetometer
    write_register(m_ak8963_CNTL2, 0x01);
    HAL_Delay(100);

    // Set the magnetometer to 16-bit output and continuous measurement mode 2 (100Hz)
    write_register(m_ak8963_CNTL1, 0x16);
    HAL_Delay(10);*/
}


/*
 * This method communicate with the sensor,
 * configure and calibrate it.
 */
void MPU9250::init(const AccScale& accScaleConf, const GyroScale& gyroScaleConf)
{
	//read_mpu9250_who_am_i();
	//read_ak8963_who_am_i();

	// Configure the IMU sensitivity
	configureAccelerometer(accScaleConf);

	configureGyroscope(gyroScaleConf);

	//init_magnetometer();

	// Calibrate accelerometer and gyroscope offsets
	calibrate();
}


/*
 * Setup the accelerometer scale.
 * Values are: 2g, 4g, 8g or 16g
 */
void MPU9250::configureAccelerometer(const AccScale& accScaleConf)
{
	m_accScaleConf = accScaleConf;

	HAL_Delay(10);

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

	HAL_Delay(10);
}


/*
 * Setup the gyroscope scale.
 * Values are: 250 dps, 500 dps, 1000 dps or 2000 dps
 */
void MPU9250::configureGyroscope(const GyroScale& gyroScaleConf)
{
	m_gyroScaleConf = gyroScaleConf;

	HAL_Delay(10);

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
			m_gyroScale = 65.5;
			break;

		case GyroScale::DPS1000:
			// Write 10 on bits [4:3] for 1000 dps configuration
			write_register(27, 0b00010000);
			// Configure the scaler value
			m_gyroScale = 32.8;
			break;

		case GyroScale::DPS2000:
			// Write 11 on bits [4:3] for 2000 dps configuration
			write_register(27, 0b00011000);
			// Configure the scaler value
			m_gyroScale = 16.4;
			break;

		default:
			// Default values will be use
			// TODO: log error
			break;
	}

	HAL_Delay(10);
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
	m_rawScaledGyro *= -1.0; // To match accelerometer 'direction'
}


void MPU9250::setup_i2c_slave(uint8_t address, uint8_t reg, uint8_t length, bool read)
{
    write_register(m_mpu9250_I2C_SLV0_ADDR, address | (read ? 0x80 : 0x00));
    write_register(m_mpu9250_I2C_SLV0_REG, reg);
    write_register(m_mpu9250_I2C_SLV0_CTRL, 0x80 | length);
}

/*
 * Read the 3 axis magnetometer of the MPU9250
 * through the SPI interface
 */
void MPU9250::read_magnetometer_data()
{
    uint8_t data[7]; // Read ST1, HXH, HXL, HYH, HYL, HZH, HZL

    //write_register(m_mpu925_I2C_SLV4_ADDR, 0x0C|0x80); //Set the I2C slave addres of AK8963 and set for read.
	//write_register(m_mpu925_I2C_SLV4_REG, 0x03); //I2C slave 0 register address from where to begin data transfer
	//write_register(m_mpu925_I2C_SLV4_CTRL, 0x87); //Read 6 bytes from the magnetometer
	setup_i2c_slave(0x0C, 0x03, 7, true);

	HAL_Delay(1);
	read_register(m_mpu9250_EXT_SENS_DATA_00, data, 7);


	// Check if data is ready and if data overflow bit is not set in ST2 register
	if ((data[0] & 0x01) && !(data[6] & 0x08))
	{
		// Combine high and low bytes
		m_rawMag(0) = (static_cast<int16_t>(data[1]) << 8) | data[0];
		m_rawMag(1) = (static_cast<int16_t>(data[3]) << 8) | data[2];
		m_rawMag(2) = (static_cast<int16_t>(data[5]) << 8) | data[4];

		// Apply scale factor
		m_rawScaledMag(0) = static_cast<double>(m_rawMag(0)) * m_magScale;
		m_rawScaledMag(1) = static_cast<double>(m_rawMag(1)) * m_magScale;
		m_rawScaledMag(2) = static_cast<double>(m_rawMag(2)) * m_magScale;
	}

    // Check if data is ready by reading ST1 register
    /*read_register(m_ak8963_ST1, data, 1);
    if (data[0] & 0x01) // Data ready bit
    {
        // Read magnetometer data
        read_register(m_ak8963_HXL, data, 7);

        // Check if data overflow bit is set in ST2 register
        if (!(data[6] & 0x08))
        {
            // Combine high and low bytes
            m_rawMag(0) = (static_cast<int16_t>(data[1]) << 8) | data[0];
            m_rawMag(1) = (static_cast<int16_t>(data[3]) << 8) | data[2];
            m_rawMag(2) = (static_cast<int16_t>(data[5]) << 8) | data[4];

            // Apply scale factor
            m_rawScaledMag(0) = static_cast<double>(m_rawMag(0)) * m_magScale;
            m_rawScaledMag(1) = static_cast<double>(m_rawMag(1)) * m_magScale;
            m_rawScaledMag(2) = static_cast<double>(m_rawMag(2)) * m_magScale;
        }
    }*/
}


uint8_t MPU9250::read_mpu9250_who_am_i()
{
    uint8_t who_am_i;

    read_register(m_mpu9250_WHO_AM_I, &who_am_i, 1);

    return who_am_i;
}

uint8_t MPU9250::read_ak8963_who_am_i()
{
    uint8_t who_am_i;

    setup_i2c_slave(m_ak8963_I2C_ADDR, m_ak8963_WHO_AM_I, 1, true);
    read_register(m_mpu9250_EXT_SENS_DATA_00, &who_am_i, 1);

    return who_am_i;
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
}


