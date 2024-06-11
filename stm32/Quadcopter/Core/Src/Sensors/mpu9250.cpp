/*
 * mpu9250.cpp
 *
 *  Created on: Jun 11, 2024
 *      Author: louis
 */


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
 * This method communicate with the sensor
 * and configure it.
 */
void MPU9250::init()
{
	// Register 28 is accelerometer configuration
	// Write 10 on bits [4:3] for +-8g configuration
	write_register(28, 0b00010000);

}


/*
 * Read 'len' byte on the register 'regAddr' of the MPU9250
 * through the SPI interface.
 */
void MPU9250::read_register(uint8_t regAddr, uint8_t* pData, uint8_t len)
{
	// MSB is a read/write bit and the 7 LSB are the register
	// 0b10000000 = 0x80 for read operation
	uint8_t cmd = 0x80 | regAddr;
	// Set CS to low, selecting the chip
	HAL_GPIO_WritePin(m_spi_cs_gpio_port, m_spi_cs_pin, GPIO_PIN_RESET);
	// Write the read bit and register to read
	HAL_SPI_Transmit(&m_hspi, &cmd, 1, 100);
	// Read 'len' byte
	HAL_SPI_Receive(&m_hspi, pData, len, 100);
	// Set CS to high, de-selecting the chip
	HAL_GPIO_WritePin(m_spi_cs_gpio_port, m_spi_cs_pin, GPIO_PIN_SET);
}


/*
 * Write data on the register 'regAddr' of the MPU9250
 * through the SPI interface.
 */
void MPU9250::write_register(uint8_t regAddr, uint8_t data)
{
	// MSB is a read/write bit and the 7 LSB are the register
	// regAddr already have the MSB to 0

	// Set CS to low, selecting the chip
	HAL_GPIO_WritePin(m_spi_cs_gpio_port, m_spi_cs_pin, GPIO_PIN_RESET);
	// Write the write bit and register to write in
	HAL_SPI_Transmit(&m_hspi, &regAddr, 1, 100);
	// Write data
	HAL_SPI_Transmit(&m_hspi, &data, 1, 100);
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
	m_rawAcc[0] = ((uint16_t)data[0] << 8) + data[1];
	m_rawAcc[1] = ((uint16_t)data[2] << 8) + data[3];
	m_rawAcc[2] = ((uint16_t)data[4] << 8) + data[5];

	// Temperature high byte and low byte combination
	m_rawTemp = ((uint16_t)data[6] << 8) + data[7];

	// Gyroscope high byte and low byte combination
	m_rawGyro[0] = ((uint16_t)data[8] << 8) + data[9];
	m_rawGyro[1] = ((uint16_t)data[10] << 8) + data[11];
	m_rawGyro[2] = ((uint16_t)data[12] << 8) + data[13];
}

/*
 * Read the 3 axis magnetometer of the MPU9250
 * through the SPI interface
 */
void MPU9250::read_magnetometer_data()
{

}


void MPU9250::calibrate()
{


}


void MPU9250::get_mpu9250_data()
{

}


