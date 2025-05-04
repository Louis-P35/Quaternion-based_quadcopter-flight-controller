/*
 * scheduler.hpp
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
#include "AHRS/madgwick.hpp"
#include "radio.hpp"
#include "Motors/motorMixer.hpp"

// Includes from 3rd party
//#include <AHRS/ESKF.h>


enum class Motor {eMotor1, eMotor2, eMotor3, eMotor4};


/*
 * This class is the main class of this flight controller
 * The 'mainSetup' method is called once by the main function
 * The 'mainLoop' is called in an infinite loop by the main function
 */
class Scheduler
{
public:
	UART_HandleTypeDef m_huart_ext;
	TIM_HandleTypeDef& m_htim1;

	// IMU data
	axises m_gyro;
	axises m_accel;
	axises m_mag;

	// Radio
	Radio m_radio;
	Quaternion m_targetAttitude;

	// ARHR (Madgwick)
	MadgwickFilter m_madgwickFilter;

	// Motors power
	double m_thrust = 0.0;
	double m_torqueX = 0.0;
	double m_torqueY = 0.0;
	double m_torqueZ = 0.0;
	XquadMixer m_motorMixer;

	ControlStrategy m_ctrlStrat;

	// Scheduler
	double m_pidRateLoopDt = 0.0;
	double m_pidAngleLoopDt = 0.0;
	double m_pidPosLoopDt = 0.0;
	double m_escsLoopDt = 0.0;
	double m_radioLoopDt = 0.0;
	bool m_angleLoop = false;
	bool m_posLoop = false;

private:
	Eigen::Vector3f m_magBias = Eigen::Vector3f(-40.05, 12.15, 29.025);
	Eigen::Vector3f m_accelOffset = Eigen::Vector3f(0.0, 0.0, 0.0);
	Eigen::Vector3f m_gyroOffset = Eigen::Vector3f(0.0, 0.0, 0.0);

public:
	Scheduler(
			SPI_HandleTypeDef hspi,
			uint16_t spi_cs_pin,
			GPIO_TypeDef* spi_cs_gpio_port,
			UART_HandleTypeDef uart_ext,
			TIM_HandleTypeDef& htim1
			);
	void mainSetup();
	void mainLoop(const double dt);

	void setMotorPower(const Motor motor, const double power);

private:
	void gyroAccelCalibration();
};



void magnetometerCalibration();
