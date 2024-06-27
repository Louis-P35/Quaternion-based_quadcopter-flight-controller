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
#include "Sensors/mpu9250.hpp"
#include "AHRS/ahrs.hpp"
#include "AHRS/complementaryFilter.hpp"
#include "AHRS/madgwick.hpp"


/*
 * This class is the main class of this flight controller
 * The 'mainSetup' method is called once by the main function
 * The 'mainLoop' is called in an infinite loop by the main function
 */
class DroneController
{
public:
	MPU9250 m_imu;

	UART_HandleTypeDef m_huart_ext;

	// AHRS
	ComplementaryFilter m_complementaryFilter;
	MadgwickFilter m_madgwickFilter;
	AHRS m_ahrs;
	AHRS m_ahrs2;


public:
	DroneController(
			SPI_HandleTypeDef hspi,
			uint16_t spi_cs_pin,
			GPIO_TypeDef* spi_cs_gpio_port,
			UART_HandleTypeDef uart_ext
			);
	void mainSetup();
	void mainLoop();

	void print(int val);
	void print(float val);
};
