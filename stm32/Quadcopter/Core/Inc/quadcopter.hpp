/*
 * quadcopter.hpp
 *
 *  Created on: Jun 11, 2024
 *      Author: louis
 */

#pragma once

// Includes from driver
#include "stm32h7xx_hal.h"

// Includes from project
#include "Sensors/icm20948.h"
#include "PID/controlStrategy.hpp"

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

	// AHRS (EKF)
	IMU_EKF::ESKF<float> m_EKF;
	// Magnetometer calibration
	Eigen::Matrix<float, 3, 3> m_W; // Soft-iron
	Eigen::Matrix<float, 3, 1> m_V; // Hard-iron
	float m_incl; // Inclination
	float m_B; // Geomagnetic field strength


	Telemetry m_currentState;
	Telemetry m_targetState;

private:
	Eigen::Vector3f m_magBias = Eigen::Vector3f(-38.4f, 4.575f, 24.675f);


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



void magnetometerCalibration();
