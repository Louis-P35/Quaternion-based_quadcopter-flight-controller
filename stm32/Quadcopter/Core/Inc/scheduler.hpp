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
#include "Sensors/mtf01.hpp"

//Includes from STL
#include <stdint.h>
//#include <functional>


// DO not change this unless change the timer 2 settings accordingly
// IMU_SAMPLE_FREQUENCY must be a round multiple of 1000
#define IMU_SAMPLE_FREQUENCY 4000
#define RATE_DIVIDER 2
#define AHRS_DIVIDER 4
#define ESC_DIVIDER 8
#define POS_HOLD_DIVIDER 40
#define RADIO_DIVIDER 80


enum class Motor {eMotor1, eMotor2, eMotor3, eMotor4};
enum class TaskType {eNone, ePID_rate, ePID_att, ePID_pos, eRead_sensor, eMain_fsm};


/*
 * Definition of a task that can be handled by the scheduler
 */
struct Task
{
	// Is free or allocated
	bool isFree = true;

	// Task type
	TaskType taskType = TaskType::eNone;

	// Task id
	uint32_t taskId = 0;

	// Period in ticks
	uint32_t periodTicks;

	// The last time it was executed (in ticks)
	uint32_t lastRunTicks = 0;

	// Priority of the task ([0 - 255], 0 = higher priority)
	uint8_t priority = 0;

	// Task to be executed
	void (*fn)() = nullptr;

	// Next task in the list
	Task* pNext = nullptr;
};


/*
 * This class is the main class of this flight controller
 * It is the scheduler of the tasks queue
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

	// Optical flow sensor
	Mtf01 m_opticalflow;

	// Target state (input of the PIDs controller)
	// Driven by the radio or autonomous control
	SetPoint<float> m_setPoint;

	// ARHR (Madgwick)
	MadgwickFilter<float> m_madgwickFilter;
	Quaternion<float> m_qAttitudeCorrected = Quaternion<float>::identity();
	Quaternion<float> m_qHoverOffset = Quaternion<float>(0.9999743f, 0.0035298f, -0.0062408f, 0.0000220f);

	// Motors power
	float m_thrust = 0.0f;
	float m_torqueX = 0.0f;
	float m_torqueY = 0.0f;
	float m_torqueZ = 0.0f;
	XquadMixer m_motorMixer;

	ControlStrategy m_ctrlStrat;

	volatile float m_batteryVoltage = 12.6;

	bool m_isFlying = false;

	static constexpr float m_ahrsDt = 1.0f / (static_cast<float>(IMU_SAMPLE_FREQUENCY) / static_cast<float>(AHRS_DIVIDER));
	static constexpr float m_rateDt = 1.0f / (static_cast<float>(IMU_SAMPLE_FREQUENCY) / static_cast<float>(RATE_DIVIDER));
	static constexpr float m_angleDt = 1.0f / (static_cast<float>(IMU_SAMPLE_FREQUENCY) / static_cast<float>(AHRS_DIVIDER)); // Same as ahrs
	static constexpr float m_posDt = 1.0f / (static_cast<float>(IMU_SAMPLE_FREQUENCY) / static_cast<float>(POS_HOLD_DIVIDER));
	static constexpr float m_radioDt = 1.0f / (static_cast<float>(IMU_SAMPLE_FREQUENCY) / static_cast<float>(RADIO_DIVIDER));

	bool m_angleLoop = false;
	bool m_posLoop = false;

private:
	static constexpr uint16_t m_nbMaxTasks = 50;
	volatile uint32_t m_ticksCounter = 0;
	std::array<Task, m_nbMaxTasks> m_tasksMemory;
	Task* m_pTasksPool = nullptr;

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
	void runTasks();
	Task* allocateTask();
	bool addTask(Task* const pTask);
	Task* removeAndFreeTask(Task* const pTask);
	Task* removeAndFreeTask(const TaskType& type);
	Task* removeAndFreeTask(const uint32_t& taskId);

	void readIMU();
	void gyroAccelCalibration();
	void calibrateHoverOffset();

	// Debug logging
	void pidDebugStream();
};
