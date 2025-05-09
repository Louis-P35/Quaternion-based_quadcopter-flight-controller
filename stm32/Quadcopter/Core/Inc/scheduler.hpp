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
#include "Utils/vector.hpp"

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
	Quaternion<double> m_targetAttitude;

	// ARHR (Madgwick)
	MadgwickFilter<double> m_madgwickFilter;
	Quaternion<double> m_qAttitudeCorrected = Quaternion<double>::iddentity();
	Quaternion<double> m_qHoverOffset = Quaternion<double>(0.9999743, 0.0035298, -0.0062408, 0.0000220);
	Vector3<double> m_calibratedGyro{0.0};
	Vector3<double> m_averagedGyro{0.0};
	Vector3<double> m_sumGyro{0.0};
	int m_nbSumGyro = 0;
	Vector3<double> m_calibratedAccel{0.0};

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
	Vector3<double> m_magBias = Vector3<double>(-40.05, 12.15, 29.025);
	Vector3<double> m_accelOffset = Vector3<double>(0.0, 0.0, 0.0);
	Vector3<double> m_gyroOffset = Vector3<double>(0.0, 0.0, 0.0);

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
	void calibrateHoverOffset();
};



void magnetometerCalibration();
