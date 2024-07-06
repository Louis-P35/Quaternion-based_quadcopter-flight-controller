/*
 * mpu9250.hpp
 *
 *  Created on: Jun 11, 2024
 *      Author: Louis
 */

#pragma once

// STL
#include "stdint.h"

// Driver
#include "stm32h7xx_hal.h"

// Project
#include "Utils/Eigen/Dense"




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
public:
	// SPI
	SPI_HandleTypeDef m_hspi;
	uint16_t m_spi_cs_pin;
	GPIO_TypeDef* m_spi_cs_gpio_port;

	Eigen::Vector<int16_t, 3> m_rawAcc = {0, 0, 0};
	Eigen::Vector<int16_t, 3> m_rawGyro = {0, 0, 0};
	int16_t m_rawTemp = 0;

	// Config
	AccScale m_accScaleConf = AccScale::_2G; // 2g is the default value
	GyroScale m_gyroScaleConf = GyroScale::DPS250; // 250 dps is the default value

	// For a range of +-2g, we need to divide the raw values by 16384
	double m_accScale = 16384.0;
	// For 250 dps (default), we need to divide the raw values by 131
	double m_gyroScale = 131.0;

	// Raw value scaled according to setup
	Eigen::Vector3d m_rawScaledAcc = {0.0, 0.0, 0.0};
	Eigen::Vector3d m_rawScaledGyro = {0.0, 0.0, 0.0};

	// Sensor offset that need to be substracted
	Eigen::Vector3d m_accOffset = {0.0, 0.0, 0.0};
	Eigen::Vector3d m_gyroOffset = {1.56, 1.75, -0.12};

public:
	// Low pass filter for accelerometer
	Eigen::Vector3d m_filteredAcceloremeter = {0.0, 0.0, 0.0};
	Eigen::Vector3d m_previousAcc = {0.0, 0.0, 0.0};
	const double m_lpf_acc_gain = 0.1;

	// Low pass filter for gyroscope
	Eigen::Vector3d m_filteredGyro = {0.0, 0.0, 0.0};
	Eigen::Vector3d m_previousGyro = {0.0, 0.0, 0.0};
	const double m_lpf_gyro_gain = 0.01;

	double m_angleAccX = 0.0;
	double m_angleAccY = 0.0;

public:
	MPU9250(SPI_HandleTypeDef hspi, uint16_t spi_cs_pin, GPIO_TypeDef* spi_cs_gpio_port);
	void init(const AccScale& accScaleConf, const GyroScale& gyroScaleConf);
	void calibrate();
	void read_gyro_acc_data();
	void read_magnetometer_data();
	void filter_and_calibrate_data();

private:
	void configureAccelerometer(const AccScale& accScaleConf);
	void configureGyroscope(const GyroScale& gyroScaleConf);
	void read_register(uint8_t regAddr, uint8_t* pData, uint8_t len);
	void write_register(uint8_t regAddr, uint8_t data);
	void handle_spi_error(HAL_StatusTypeDef result);
};








