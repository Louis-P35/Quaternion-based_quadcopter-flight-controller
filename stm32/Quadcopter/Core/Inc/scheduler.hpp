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
#include "Sensors/IMU.hpp"
#include "PID/controlStrategy.hpp"
#include "AHRS/madgwick.hpp"
#include "radio.hpp"
#include "Motors/motorMixer.hpp"
#include "Utils/vector.hpp"


// DO not change this unless change the timer 2 settings accordingly
// IMU_SAMPLE_FREQUENCY must be a round multiple of 1000
#define IMU_SAMPLE_FREQUENCY 6000
#define RATE_DIVIDER 3
#define AHRS_DIVIDER 6
#define ESC_DIVIDER 12
#define POS_HOLD_DIVIDER 60
#define RADIO_DIVIDER 120


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

	// IMU
	IMU m_imu;
	Vector3<float> m_gyroCopy;
	Vector3<float> m_accelCopy;

	// Radio
	Radio m_radio;
	Quaternion<float> m_targetAttitude;

	// ARHR (Madgwick)
	MadgwickFilter<float> m_madgwickFilter;
	Quaternion<float> m_qAttitudeCorrected = Quaternion<float>::iddentity();
	Quaternion<float> m_qHoverOffset = Quaternion<float>(0.9999743f, 0.0035298f, -0.0062408f, 0.0000220f);

	// Motors power
	float m_thrust = 0.0f;
	float m_torqueX = 0.0f;
	float m_torqueY = 0.0f;
	float m_torqueZ = 0.0f;
	XquadMixer m_motorMixer;

	ControlStrategy m_ctrlStrat;

	uint32_t m_ahrsStartTime = 0;
	uint32_t m_rateStartTime = 0;
	uint32_t m_angleStartTime = 0;
	uint32_t m_posStartTime = 0;
	uint32_t m_radioStartTime = 0;
	float m_ahrsDt = 0.001f; // Non 0 init to avoid /0
	float m_rateDt = 0.001f;
	float m_angleDt = 0.001f;
	float m_posDt = 0.001f;
	float m_radioDt = 0.001f;

	bool m_angleLoop = false;
	bool m_posLoop = false;

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

	void setMotorPower(const Motor& motor, const float& power);

private:
	void readIMU();
	void gyroAccelCalibration();
	void calibrateHoverOffset();
};
