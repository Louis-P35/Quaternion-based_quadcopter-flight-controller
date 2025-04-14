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
#include "AHRS/ahrs.hpp"
#include "AHRS/complementaryFilter.hpp"
#include "AHRS/kalman.hpp"


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
	ComplementaryFilter m_complementaryFilter;
	ExtendedKalmanFilter m_kalmanFilter;
	AHRS m_ahrs;
	AHRS m_ahrs3;


public:
	DroneController(
			SPI_HandleTypeDef hspi,
			uint16_t spi_cs_pin,
			GPIO_TypeDef* spi_cs_gpio_port,
			UART_HandleTypeDef uart_ext
			);
	void mainSetup();
	void mainLoop(const double dt);

	void print(int val);
	void print(float val);
};
