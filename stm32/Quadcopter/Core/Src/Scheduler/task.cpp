/*
 * task.cpp
 *
 *  Created on: Jul 11, 2025
 *      Author: louis
 */


// Includes from project
#include "Scheduler/task.hpp"



/*
 * Task initialization.
 */
void Task::setup(const TaskType& type, const uint8_t& priority, void (*function)(const float&))
{
	m_taskType = type;
	m_priority = priority;
	m_fn = function;
}


/*
 * Allocate memory for a Task instance from a pre-allocated array.
 * Return the address of the first available slot.
 */
Task* Task::allocateTask()
{
	static std::array<Task, MAX_TASKS_NUMBER> tasksPreAllocatedMemory; // Pre-allocated array
	static uint32_t tasksId = 0;

	for (std::size_t i = 0; i < MAX_TASKS_NUMBER; ++i)
	{
		if (tasksPreAllocatedMemory[i].m_isFree)
		{
			tasksPreAllocatedMemory[i].m_isFree = false; // Mark slot as allocated

			// Basic initialization
			tasksPreAllocatedMemory[i].m_pNext = nullptr;
			tasksPreAllocatedMemory[i].m_taskId = tasksId++;
			tasksPreAllocatedMemory[i].m_taskType = TaskType::eNone;
			tasksPreAllocatedMemory[i].m_fn = nullptr;
			tasksPreAllocatedMemory[i].m_priority = 0;

			return &tasksPreAllocatedMemory[i];
		}
	}

	// Allocation failed
	return nullptr;
}

