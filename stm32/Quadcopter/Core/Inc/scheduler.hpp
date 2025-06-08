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
#include "Radio/radio.hpp"
#include "Motors/motorMixer.hpp"
#include "Utils/vector.hpp"
#include "BlackboxSD/blackbox.hpp"
#include "setPoints.hpp"


// DO not change this unless change the timer 2 settings accordingly
// IMU_SAMPLE_FREQUENCY must be a round multiple of 1000
#define IMU_SAMPLE_FREQUENCY 4000
#define RATE_DIVIDER 2
#define AHRS_DIVIDER 4
#define ESC_DIVIDER 8
#define POS_HOLD_DIVIDER 40
#define RADIO_DIVIDER 80


enum class Motor {eMotor1, eMotor2, eMotor3, eMotor4};


/*
 * This class is the main class of this flight controller
 * The 'mainSetup' method is called once by the main function
 * The 'mainLoop' is called in an infinite loop by the main function
 */
class Scheduler
{
public:
	// Blackbox logger on SD card
	//Blackbox<36> m_18BytesBlackbox;

	// IMU
	IMU m_imu;

	// Radio
	Radio m_radio;
	Quaternion<float> m_targetAttitude; // TODO remove that

	// Target state (input of the PIDs controller)
	// Driven by the radio or autonomous control
	SetPoint<float> m_setPoint;

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

	volatile float m_batteryVoltage = 12.6;

	static constexpr float m_ahrsDt = 1.0f / (static_cast<float>(IMU_SAMPLE_FREQUENCY) / static_cast<float>(AHRS_DIVIDER));
	static constexpr float m_rateDt = 1.0f / (static_cast<float>(IMU_SAMPLE_FREQUENCY) / static_cast<float>(RATE_DIVIDER));
	static constexpr float m_angleDt = 1.0f / (static_cast<float>(IMU_SAMPLE_FREQUENCY) / static_cast<float>(AHRS_DIVIDER)); // Same as ahrs
	static constexpr float m_posDt = 1.0f / (static_cast<float>(IMU_SAMPLE_FREQUENCY) / static_cast<float>(POS_HOLD_DIVIDER));
	static constexpr float m_radioDt = 1.0f / (static_cast<float>(IMU_SAMPLE_FREQUENCY) / static_cast<float>(RADIO_DIVIDER));

	bool m_angleLoop = false;
	bool m_posLoop = false;

public:
	Scheduler(
			uint16_t spi_cs_pin,
			GPIO_TypeDef* spi_cs_gpio_port
			);
	void mainSetup();
	void mainLoop(const double dt);

	void pidRateLoop();
	void ahrsLoop();
	void escLoop();
	void radioLoop();

	void setMotorPower(const Motor& motor, const float& power);
	float readBatteryVoltage();

private:
	void readIMU();
	void gyroAccelCalibration();
	void calibrateHoverOffset();

	// Debug logging
	void pidDebugStream();
};
