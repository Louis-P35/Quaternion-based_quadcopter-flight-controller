/*
 * mpu9250.hpp
 *
 *  Created on: Jun 11, 2024
 *      Author: Louis
 */

#pragma once

#include "stdint.h"

#include "stm32h7xx_hal.h"


enum class AccScale
{
	_2G,
	_4G,
	_8G,
	_16G
};

enum class GyroScale
{
	DPS250,
	DPS500,
	DPS1000,
	DPS2000
};



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

	// Config
	AccScale m_accScaleConf = AccScale::_2G; // 2g is the default value
	GyroScale m_gyroScaleConf = GyroScale::DPS250; // 250 dps is the default value

	// For a range of +-2g, we need to divide the raw values by 16384
	double m_accScale = 16384.0;
	// For 250 dps (default), we need to divide the raw values by 131
	double m_gyroScale = 131.0;

	// Raw value scaled according to setup
	double m_rawScaledAcc[3] = {0.0};
	double m_rawScaledGyro[3] = {0.0};

	// Sensor offset that need to be substracted
	double m_accOffset[3] = {0.0};
	double m_gyroOffset[3] = {1.56, 1.75, -0.12};

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
	void init(const AccScale& accScaleConf, const GyroScale& gyroScaleConf);
	void calibrate();
	void read_gyro_acc_data();
	void read_magnetometer_data();
	void get_mpu9250_data();

private:
	void configureAccelerometer(const AccScale& accScaleConf);
	void configureGyroscope(const GyroScale& gyroScaleConf);
	void read_register(uint8_t regAddr, uint8_t* pData, uint8_t len);
	void write_register(uint8_t regAddr, uint8_t data);
	void handle_spi_error(HAL_StatusTypeDef result);
};








