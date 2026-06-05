/*
 * flightCore.hpp
 *
 *  Created on: Jul 10, 2025
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
#include "Sensors/mtf01.hpp"
#include "ESPInterface/espInterface.hpp"
#include "Filters/lpfBiquadButterworth.hpp"

//Includes from STL
#include <stdint.h>


/*
 * C wrapper functions.
 */
void mainLoop(const double dt);
void readIMU_task(const float& dt);
void AHRS_task(const float& dt);
void ESCs_task(const float& dt);
void pidPos_task(const float& dt);
void pidAtt_task(const float& dt);
void pidRate_task(const float& dt);
void mainFSM_task(const float& dt);
void subFSM_task(const float& dt);
void readBattery_task(const float& dt);
void readRadio_task(const float& dt);
void readOpticalFlow_task(const float& dt);
void debugPrint_task(const float& dt);

enum class Motor {eMotor1, eMotor2, eMotor3, eMotor4};


/*
 * This class is the main class of this flight controller
 * It is the scheduler of the tasks queue
 * The 'mainSetup' method is called once by the main function
 * The 'mainLoop' is called in an infinite loop by the main function
 */
class FlightCore
{
public:
	// Blackbox logger on SD card
	//Blackbox<36> m_18BytesBlackbox;

	// IMU
	IMU m_imu;

	// Radio
	Radio m_radio;

	// Optical flow sensor
	Mtf01 m_opticalflow;

	// ESP32 interface (WiFi/GCS bridge)
	EspInterface m_espInterface;

	// Target state (input of the PIDs controller)
	// Driven by the radio or autonomous control
	SetPoint<float> m_setPoint;

	// ARHR (Madgwick)
	MadgwickFilter<float> m_madgwickFilter;

	// Magnetometer low-pass filters (Butterworth 2nd order, applied at 100 Hz in ahrsLoop)
	BiquadLPF<float> m_lpfMagX, m_lpfMagY, m_lpfMagZ;

	// Last filtered IMU magnetometer values (updated at 100 Hz when useMARG is true)
	float m_magFiltX = 0.f, m_magFiltY = 0.f, m_magFiltZ = 0.f;


	// Motors power
	float m_thrust = 0.0f;
	float m_torqueX = 0.0f;
	float m_torqueY = 0.0f;
	float m_torqueZ = 0.0f;
	XquadMixer m_motorMixer;

	ControlStrategy m_ctrlStrat;

	volatile float m_batteryVoltage = 12.6f;

	bool m_isFlying = false;

	bool m_rateLoopEnable = false;
	bool m_angleLoopEnable = false;
	bool m_posLoopEnable = false;

public:
	FlightCore(
			uint16_t spi_cs_pin,
			GPIO_TypeDef* spi_cs_gpio_port,
			uint16_t esp_cs_pin = 0,
			GPIO_TypeDef* esp_cs_gpio_port = nullptr
			);
	void mainSetup();

	void ahrsLoop(const float& dt);
	void escLoop();
	void radioLoop(const float& dt);
	void pidRateLoop(const float& dt);
	void pidAttLoop(const float& dt);
	void pidPosLoop(const float& dt);
	void batteryLoop();
	void debugPrintLoop();

	void setMotorPower(const Motor& motor, const float& power);
	float readBatteryVoltage();

private:
	void readIMU();
	void gyroAccelCalibration();

	// Debug logging
	void pidDebugStream();
};
