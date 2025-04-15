/*
 * quadcopter.hpp
 *
 *  Created on: Jun 11, 2024
 *      Author: louis
 */

#pragma once

// Driver
#include "stm32h7xx_hal.h"

// Project
#include "Sensors/icm20948.h"

// Includes from 3rd party
#include <AHRS/ESKF.h>


/*
 * This class is the main class of this flight controller
 * The 'mainSetup' method is called once by the main function
 * The 'mainLoop' is called in an infinite loop by the main function
 */
class DroneController
{
public:
	UART_HandleTypeDef m_huart_ext;

	// IMU data
	axises m_gyro;
	axises m_accel;
	axises m_mag;

	// AHRS
	//ComplementaryFilter m_complementaryFilter;
	//ExtendedKalmanFilter m_kalmanFilter;
	//AHRS m_ahrs;
	//AHRS m_ahrs3;

	// EKF
	IMU_EKF::ESKF<float> m_EKF;
	// magnetometer calibration
	Eigen::Matrix<float, 3, 3> m_W; // soft-iron
	Eigen::Matrix<float, 3, 1> m_V; // hard-iron
	float m_incl; // inclination
	float m_B; // geomagnetic field strength


public:
	DroneController(
			SPI_HandleTypeDef hspi,
			uint16_t spi_cs_pin,
			GPIO_TypeDef* spi_cs_gpio_port,
			UART_HandleTypeDef uart_ext
			);
	void mainSetup();
	void mainLoop(const float dt);

	void print(int val);
	void print(float val);
};
