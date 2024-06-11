/*
 * mpu9250.hpp
 *
 *  Created on: Jun 11, 2024
 *      Author: Louis
 */

#pragma once

#include "stdint.h"

#include "stm32h7xx_hal.h"



/*
 * This class handle the MPU9250 IMU
 * through the SPI interface.
 * It configure it, read and filter the raw value.
 */
class MPU9250
{
private:
	// SPI
	SPI_HandleTypeDef m_hspi;
	uint16_t m_spi_cs_pin;
	GPIO_TypeDef* m_spi_cs_gpio_port;

	uint16_t m_rawAcc[3] = {0};
	uint16_t m_rawGyro[3] = {0};
	uint16_t m_rawTemp = 0;

	// For a range of +-8g, we need to divide the raw values by 4096
	double m_accScale = 4096.0;

	double m_gyroScale = 65.0;

	double m_accOffsetX = 0.0;
	double m_accOffsetY = 0.0;
	double m_accOffsetZ = 0.0;

	double m_gyroOffsetX = 1.56;
	double m_gyroOffsetY = 1.75;
	double m_gyroOffsetZ = -0.12;

public:
	double m_filteredAcceloremeterX = 0.0;
	double m_filteredAcceloremeterY = 0.0;
	double m_filteredAcceloremeterZ = 0.0;

	const double m_lpf_acc_gain = 0.1;

	double m_previousAccX = 0.0;
	double m_previousAccY = 0.0;
	double m_previousAccZ = 0.0;

	double m_angleAccX = 0.0;
	double m_angleAccY = 0.0;

	double m_filteredGyroX = 0.0;
	double m_filteredGyroY = 0.0;
	double m_filteredGyroZ = 0.0;

	double m_previousGyroX = 0.0;
	double m_previousGyroY = 0.0;
	double m_previousGyroZ = 0.0;

	const double m_lpf_gyro_gain = 0.01;

public:
	MPU9250(SPI_HandleTypeDef hspi, uint16_t spi_cs_pin, GPIO_TypeDef* spi_cs_gpio_port);
	void init();
	void calibrate();
	void read_register(uint8_t regAddr, uint8_t* pData, uint8_t len);
	void write_register(uint8_t regAddr, uint8_t data);
	void read_gyro_acc_data();
	void read_magnetometer_data();
	void get_mpu9250_data();
};








