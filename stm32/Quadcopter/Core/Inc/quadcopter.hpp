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
#include "AHRS/madgwick.hpp"
#include "radio.hpp"

// Includes from 3rd party
//#include <AHRS/ESKF.h>


enum class Motor {eMotor1, eMotor2, eMotor3, eMotor4};


/*
 * This class is the main class of this flight controller
 * The 'mainSetup' method is called once by the main function
 * The 'mainLoop' is called in an infinite loop by the main function
 */
class DroneController
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

	// MotorPower
	double m_motor1Power = 0.0;
	double m_motor2Power = 0.0;
	double m_motor3Power = 0.0;
	double m_motor4Power = 0.0;

	// AHRS (EKF)
	//IMU_EKF::ESKF<double> m_EKF;
	// Magnetometer calibration
	//Eigen::Matrix<double, 3, 3> m_W; // Soft-iron
	//Eigen::Matrix<double, 3, 1> m_V; // Hard-iron
	//float m_incl; // Inclination
	//float m_B; // Geomagnetic field strength

	ControlStrategy m_ctrlStrat;

private:
	Eigen::Vector3f m_magBias = Eigen::Vector3f(-40.05, 12.15, 29.025);

	Eigen::Vector3f m_accelOffset = Eigen::Vector3f(0.0, 0.0, 0.0);
	Eigen::Vector3f m_gyroOffset = Eigen::Vector3f(0.0, 0.0, 0.0);

public:
	DroneController(
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
