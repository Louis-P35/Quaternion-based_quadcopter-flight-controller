/*
 * scheduler.hpp
 *
 *  Created on: Jun 11, 2024
 *      Author: louis
 */

#pragma once

// Includes from project
#include "Scheduler/task.hpp"

// Includes from driver
#include "stm32h7xx_hal.h"

//Includes from STL
#include <stdint.h>

// DO not change this unless change the timer 2 settings accordingly
#define SYSTICKS_SAMPLE_FREQUENCY (4000)

enum class FREQUENCY_SLOT {
	e_32KHZ = 	0,
	e_16KHZ = 	1,
	e_8KHZ = 	2,
	e_4KHZ = 	3,
	e_2KHZ = 	4,
	e_1KHZ = 	5,
	e_500HZ = 	6,
	e_250HZ = 	7,
	e_100HZ = 	8,
	e_50HZ = 	9,
	e_10HZ = 	10,
	e_5HZ = 	11,
	e_1HZ = 	12,
	count
};

#define NUMBER_TASKS_FREQUENCY_SLOTS (static_cast<size_t>(FREQUENCY_SLOT::count))

#define LOOP_MASK_32KHZ (0b0000000000000001)
#define LOOP_MASK_16KHZ (0b0000000000000010)
#define LOOP_MASK_8KHZ  (0b0000000000000100)
#define LOOP_MASK_4KHZ  (0b0000000000001000)
#define LOOP_MASK_2KHZ  (0b0000000000010000)
#define LOOP_MASK_1KHZ  (0b0000000000100000)
#define LOOP_MASK_500HZ (0b0000000001000000)
#define LOOP_MASK_250HZ (0b0000000010000000)
#define LOOP_MASK_100HZ (0b0000000100000000)
#define LOOP_MASK_50HZ  (0b0000001000000000)
#define LOOP_MASK_10HZ  (0b0000010000000000)
#define LOOP_MASK_5HZ   (0b0000100000000000)
#define LOOP_MASK_1HZ   (0b0001000000000000)

#define DIVIDER_32KHZ 	(SYSTICKS_SAMPLE_FREQUENCY / 32000)
#define DIVIDER_16KHZ 	(SYSTICKS_SAMPLE_FREQUENCY / 16000)
#define DIVIDER_8KHZ 	(SYSTICKS_SAMPLE_FREQUENCY / 8000)
#define DIVIDER_4KHZ 	(SYSTICKS_SAMPLE_FREQUENCY / 4000)
#define DIVIDER_2KHZ 	(SYSTICKS_SAMPLE_FREQUENCY / 2000)
#define DIVIDER_1KHZ 	(SYSTICKS_SAMPLE_FREQUENCY / 1000)
#define DIVIDER_500HZ 	(SYSTICKS_SAMPLE_FREQUENCY / 500)
#define DIVIDER_250HZ 	(SYSTICKS_SAMPLE_FREQUENCY / 250)
#define DIVIDER_100HZ 	(SYSTICKS_SAMPLE_FREQUENCY / 100)
#define DIVIDER_50HZ 	(SYSTICKS_SAMPLE_FREQUENCY / 50)
#define DIVIDER_10HZ 	(SYSTICKS_SAMPLE_FREQUENCY / 10)
#define DIVIDER_5HZ 	(SYSTICKS_SAMPLE_FREQUENCY / 5)
#define DIVIDER_1HZ 	(SYSTICKS_SAMPLE_FREQUENCY)



/*
 * It is the scheduler of the tasks queue.
 */
class Scheduler
{
private:
	// Array of linked list of tasks. Each array's element correspond to a specific frequency
	static std::array<Task*, NUMBER_TASKS_FREQUENCY_SLOTS> m_ppTasksPoolArray;

public:
	volatile uint32_t m_ticksCounter = 0;					// Main tick counter
	volatile uint16_t m_loops_frequencies_bit_fields = 0;	// Each bit enable a specified frequency loop

public:
	uint32_t runTasks();

	bool addTask(
			const TaskType& type,
			const uint8_t& priority,
			void (*function)(),
			const FREQUENCY_SLOT& frequencySlot
			);
	Task* removeAndFreeTask(Task* const pTask);
	Task* removeAndFreeTask(const TaskType& type);
	Task* removeAndFreeTask(const uint32_t& taskId);
};




