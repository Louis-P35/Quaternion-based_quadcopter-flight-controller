/*
 * task.hpp
 *
 *  Created on: Jul 11, 2025
 *      Author: louis
 */

#pragma once


//Includes from STL
#include <stdint.h>
#include <array>


#define MAX_TASKS_NUMBER (50)


enum class TaskType {
	eNone,
	ePID_rate,
	ePID_att,
	ePID_pos,
	eAHRS,
	eESCs,
	eRead_IMU,
	eRead_opticalFlow,
	eMain_fsm,
	eRead_radio,
	eRead_battery,
	eDebugPrint
};


/*
 * Definition of a task that can be handled by the scheduler.
 */
class Task
{
public:
	// Task type
	TaskType m_taskType = TaskType::eNone;

	// The average time this task take
	uint32_t m_averageTime = 0;

	// Priority of the task ([0 - 255], 0 = higher priority)
	uint8_t m_priority = 0;

	// Task to be executed
	void (*m_fn)(const float&) = nullptr;

	// Next task in the list
	Task* m_pNext = nullptr;

	// Is free or allocated
	bool m_isFree = true;

private:
	// Task id
	uint32_t m_taskId = 0;

public:
	void setup(const TaskType& type, const uint8_t& priority, void (*function)(const float&));

	static Task* allocateTask();
};
