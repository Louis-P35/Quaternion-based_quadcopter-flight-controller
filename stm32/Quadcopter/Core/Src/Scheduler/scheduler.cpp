/*
 * scheduler.cpp
 *
 *  Created on: Jun 11, 2024
 *      Author: louis
 */

// Includes from STL
#include <orchestrator.h>
#include <string.h>  // Include for memcpy
#include <algorithm>
#include <limits>

// Includes from Project
#include "Scheduler/scheduler.hpp"


Scheduler g_scheduler;

std::array<Task*, NUMBER_TASKS_FREQUENCY_SLOTS> Scheduler::m_ppTasksPoolArray = {nullptr};


/*
 * Called at 4khz by timer 2 overflow interrupt
 * This schedule the different frequencies loops
 */
void systemTicksScheduler()
{
	//static uint32_t ticks = 0;

	g_scheduler.m_ticksCounter++;

	if ((g_scheduler.m_ticksCounter % DIVIDER_4KHZ) == 0)
	{
		g_scheduler.m_loops_frequencies_bit_fields |= LOOP_MASK_4KHZ;
	}
	if ((g_scheduler.m_ticksCounter % DIVIDER_2KHZ) == 0)
	{
		g_scheduler.m_loops_frequencies_bit_fields |= LOOP_MASK_2KHZ;
	}
	if ((g_scheduler.m_ticksCounter % DIVIDER_1KHZ) == 0)
	{
		g_scheduler.m_loops_frequencies_bit_fields |= LOOP_MASK_1KHZ;
	}
	if ((g_scheduler.m_ticksCounter % DIVIDER_500HZ) == 0)
	{
		g_scheduler.m_loops_frequencies_bit_fields |= LOOP_MASK_500HZ;
	}
	if ((g_scheduler.m_ticksCounter % DIVIDER_250HZ) == 0)
	{
		g_scheduler.m_loops_frequencies_bit_fields |= LOOP_MASK_250HZ;
	}
	if ((g_scheduler.m_ticksCounter % DIVIDER_100HZ) == 0)
	{
		g_scheduler.m_loops_frequencies_bit_fields |= LOOP_MASK_100HZ;
	}
	if ((g_scheduler.m_ticksCounter % DIVIDER_50HZ) == 0)
	{
		g_scheduler.m_loops_frequencies_bit_fields |= LOOP_MASK_50HZ;
	}
	if ((g_scheduler.m_ticksCounter % DIVIDER_10HZ) == 0)
	{
		g_scheduler.m_loops_frequencies_bit_fields |= LOOP_MASK_10HZ;
	}
	if ((g_scheduler.m_ticksCounter % DIVIDER_5HZ) == 0)
	{
		g_scheduler.m_loops_frequencies_bit_fields |= LOOP_MASK_5HZ;
	}
	if ((g_scheduler.m_ticksCounter % DIVIDER_1HZ) == 0)
	{
		g_scheduler.m_loops_frequencies_bit_fields |= LOOP_MASK_1HZ;
	}


	//if (g_start /*&& !g_startPrint*/)
	/*{
		ticks++;

		// 4 khz loop
		g_scheduler.m_imu.readAndFilterIMU_gdps();

		// 1 khz loop
		// AHRS
		if ((ticks % AHRS_DIVIDER) == 0) // 1khz (4khz / 4)
		{
			g_scheduler.ahrsLoop();
			//ahrsTicks++;
		}

		// 500 hz loop
		// ESCs
		if ((ticks % ESC_DIVIDER) == 0) // 500hz (6khz / 12)
		{
			g_scheduler.escLoop();
			//escTicks++;
		}

		// 100 hz loop
		// PID position hold
		if ((ticks % POS_HOLD_DIVIDER) == 0) // 100hz (6khz / 60)
		{
			// Enable PID position hold loop (that will run in the state machine)
			// Enable it only in certain flight mode
			if (g_scheduler.m_ctrlStrat.m_flightMode == StabilizationMode::POSHOLD)
			{
				g_scheduler.m_posLoop = true;
			}

			// Compute voltage compensation
			g_scheduler.m_motorMixer.computeVoltageCompensation(g_scheduler.m_batteryVoltage);
		}

		// 50 hz loop
		// Radio
		if ((ticks % RADIO_DIVIDER) == 0) // 50hz (6khz / 120)
		{
			g_scheduler.radioLoop();
			//radioTicks++;
		}

		// 2 khz loop
		// PID rate loop
		// The last called because it run the state machine
		if ((ticks % RATE_DIVIDER) == 0) // 2khz (4khz / 2)
		{
			g_scheduler.pidRateLoop();
			//pidRateTicks++;
		}
	}*/
}



/*
 * Run all the tasks in the task queue array according to their priority.
 * TODO: If no tasks has been ran, return the wait time before the next task.
 */
uint32_t Scheduler::runTasks()
{
	// Loop through all the slots of fixed frequency tasks linked list
	for (size_t i = 0; i < NUMBER_TASKS_FREQUENCY_SLOTS; ++i)
	{
		// Evaluate if this tasks' frequency slot need to be run
		if (!((m_loops_frequencies_bit_fields >> uint16_t(i)) & 1u))
		{
			continue;
		}

		Task* pCurrentTask = m_ppTasksPoolArray[i];

		// Loop through the list of tasks
		while (pCurrentTask != nullptr)
		{
			if (pCurrentTask->m_fn != nullptr)
			{
				// Run the task
				pCurrentTask->m_fn();
			}

			pCurrentTask = pCurrentTask->m_pNext;
		}

		// Clear bit field
		m_loops_frequencies_bit_fields &= ~(1u << uint16_t(i));
	}

	return 0;
}


/*
 * Add a task to the task queue of a frequency slot.
 * The queue will remain sorted by tasks's priority.
 * Return false if insertion failed, true otherwise.
 */
bool Scheduler::addTask(Task* const pTask, const FREQUENCY_SLOT& frequencySlot)
{
	if (!pTask || frequencySlot >= FREQUENCY_SLOT::count)
	{
		return false;
	}

	// Get the address of the root of the linked list of the right frequency slot
	Task** ppCurrentTask = &m_ppTasksPoolArray[static_cast<size_t>(frequencySlot)];

	// Empty queue, insert as first element
	if (*ppCurrentTask == nullptr)
	{
		pTask->m_pNext = nullptr;
		*ppCurrentTask = pTask;

		return true;
	}

	// Sorted insertion by priority
	while (*ppCurrentTask != nullptr && (*ppCurrentTask)->m_priority <= pTask->m_priority)
	{
		ppCurrentTask = &((*ppCurrentTask)->m_pNext);
	}

	pTask->m_pNext = *ppCurrentTask;
	*ppCurrentTask = pTask;

	return true;
}


/*
 * Remove a task form the task queue.
 * Return the address of the removed task.
 * Also mark the task as free in the static pre-allocated buffer.
 */
Task* Scheduler::removeAndFreeTask(Task* const pTask)
{
	if (!pTask)
	{
		return nullptr;
	}

	// Loop through all the slots of fixed frequency tasks linked list
	for (size_t i = 0; i < NUMBER_TASKS_FREQUENCY_SLOTS; ++i)
	{
		Task** ppCurrentTask = &m_ppTasksPoolArray[i];

		while (*ppCurrentTask != nullptr)
		{
			if (*ppCurrentTask == pTask)
			{
				*ppCurrentTask = (*ppCurrentTask)->m_pNext;

				*pTask = {}; // Write all fields at 0, thus it set it's m_pNext to nullptr
				pTask->m_isFree = true; // Mark the task as free in the static pre-allocated buffer

				return pTask;
			}

			ppCurrentTask = &((*ppCurrentTask)->m_pNext);
		}
	}

	return nullptr;
}



