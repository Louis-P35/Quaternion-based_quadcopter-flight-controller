/*
 * schedulerUnitTests.cpp
 *
 *  Created on: Jul 12, 2025
 *      Author: louis
 */


// Includes from project
#include "UnitTests/schedulerUnitTests.hpp"

// Includes from STL
#include <cstdint>

// 4 kHz tasks
uint32_t g_task_4khz_1Cntr = 0;
void task_4khz_1(const float& dt)
{
    g_task_4khz_1Cntr++;
}

uint32_t g_task_4khz_2Cntr = 0;
void task_4khz_2(const float& dt)
{
    g_task_4khz_2Cntr++;
}

uint32_t g_task_4khz_3Cntr = 0;
void task_4khz_3(const float& dt)
{
    g_task_4khz_3Cntr++;
}

uint32_t g_task_4khz_4Cntr = 0;
void task_4khz_4(const float& dt)
{
    g_task_4khz_4Cntr++;
}

uint32_t g_task_4khz_5Cntr = 0;
void task_4khz_5(const float& dt)
{
    g_task_4khz_5Cntr++;
}

// 2 kHz tasks
uint32_t g_task_2khz_1Cntr = 0;
void task_2khz_1(const float& dt)
{
    g_task_2khz_1Cntr++;
}

uint32_t g_task_2khz_2Cntr = 0;
void task_2khz_2(const float& dt)
{
    g_task_2khz_2Cntr++;
}

uint32_t g_task_2khz_3Cntr = 0;
void task_2khz_3(const float& dt)
{
    g_task_2khz_3Cntr++;
}

uint32_t g_task_2khz_4Cntr = 0;
void task_2khz_4(const float& dt)
{
    g_task_2khz_4Cntr++;
}

uint32_t g_task_2khz_5Cntr = 0;
void task_2khz_5(const float& dt)
{
    g_task_2khz_5Cntr++;
}

// 1 kHz tasks
uint32_t g_task_1khz_1Cntr = 0;
void task_1khz_1(const float& dt)
{
    g_task_1khz_1Cntr++;
}

uint32_t g_task_1khz_2Cntr = 0;
void task_1khz_2(const float& dt)
{
    g_task_1khz_2Cntr++;
}

uint32_t g_task_1khz_3Cntr = 0;
void task_1khz_3(const float& dt)
{
    g_task_1khz_3Cntr++;
}

uint32_t g_task_1khz_4Cntr = 0;
void task_1khz_4(const float& dt)
{
    g_task_1khz_4Cntr++;
}

uint32_t g_task_1khz_5Cntr = 0;
void task_1khz_5(const float& dt)
{
    g_task_1khz_5Cntr++;
}

// 500 Hz tasks
uint32_t g_task_500hz_1Cntr = 0;
void task_500hz_1(const float& dt)
{
    g_task_500hz_1Cntr++;
}

uint32_t g_task_500hz_2Cntr = 0;
void task_500hz_2(const float& dt)
{
    g_task_500hz_2Cntr++;
}

uint32_t g_task_500hz_3Cntr = 0;
void task_500hz_3(const float& dt)
{
    g_task_500hz_3Cntr++;
}

uint32_t g_task_500hz_4Cntr = 0;
void task_500hz_4(const float& dt)
{
    g_task_500hz_4Cntr++;
}

uint32_t g_task_500hz_5Cntr = 0;
void task_500hz_5(const float& dt)
{
    g_task_500hz_5Cntr++;
}

// 250 Hz tasks
uint32_t g_task_250hz_1Cntr = 0;
void task_250hz_1(const float& dt)
{
    g_task_250hz_1Cntr++;
}

uint32_t g_task_250hz_2Cntr = 0;
void task_250hz_2(const float& dt)
{
    g_task_250hz_2Cntr++;
}

uint32_t g_task_250hz_3Cntr = 0;
void task_250hz_3(const float& dt)
{
    g_task_250hz_3Cntr++;
}

uint32_t g_task_250hz_4Cntr = 0;
void task_250hz_4(const float& dt)
{
    g_task_250hz_4Cntr++;
}

uint32_t g_task_250hz_5Cntr = 0;
void task_250hz_5(const float& dt)
{
    g_task_250hz_5Cntr++;
}

// 100 Hz tasks
uint32_t g_task_100hz_1Cntr = 0;
void task_100hz_1(const float& dt)
{
    g_task_100hz_1Cntr++;
}

uint32_t g_task_100hz_2Cntr = 0;
void task_100hz_2(const float& dt)
{
    g_task_100hz_2Cntr++;
}

uint32_t g_task_100hz_3Cntr = 0;
void task_100hz_3(const float& dt)
{
    g_task_100hz_3Cntr++;
}

uint32_t g_task_100hz_4Cntr = 0;
void task_100hz_4(const float& dt)
{
    g_task_100hz_4Cntr++;
}

uint32_t g_task_100hz_5Cntr = 0;
void task_100hz_5(const float& dt)
{
    g_task_100hz_5Cntr++;
}

// 50 Hz tasks
uint32_t g_task_50hz_1Cntr = 0;
void task_50hz_1(const float& dt)
{
    g_task_50hz_1Cntr++;
}

uint32_t g_task_50hz_2Cntr = 0;
void task_50hz_2(const float& dt)
{
    g_task_50hz_2Cntr++;
}

uint32_t g_task_50hz_3Cntr = 0;
void task_50hz_3(const float& dt)
{
    g_task_50hz_3Cntr++;
}

uint32_t g_task_50hz_4Cntr = 0;
void task_50hz_4(const float& dt)
{
    g_task_50hz_4Cntr++;
}

uint32_t g_task_50hz_5Cntr = 0;
void task_50hz_5(const float& dt)
{
    g_task_50hz_5Cntr++;
}

// 10 Hz tasks
uint32_t g_task_10hz_1Cntr = 0;
void task_10hz_1(const float& dt)
{
    g_task_10hz_1Cntr++;
}

uint32_t g_task_10hz_2Cntr = 0;
void task_10hz_2(const float& dt)
{
    g_task_10hz_2Cntr++;
}

uint32_t g_task_10hz_3Cntr = 0;
void task_10hz_3(const float& dt)
{
    g_task_10hz_3Cntr++;
}

uint32_t g_task_10hz_4Cntr = 0;
void task_10hz_4(const float& dt)
{
    g_task_10hz_4Cntr++;
}

uint32_t g_task_10hz_5Cntr = 0;
void task_10hz_5(const float& dt)
{
    g_task_10hz_5Cntr++;
}

// 5 Hz tasks
uint32_t g_task_5hz_1Cntr = 0;
void task_5hz_1(const float& dt)
{
    g_task_5hz_1Cntr++;
}

uint32_t g_task_5hz_2Cntr = 0;
void task_5hz_2(const float& dt)
{
    g_task_5hz_2Cntr++;
}

uint32_t g_task_5hz_3Cntr = 0;
void task_5hz_3(const float& dt)
{
    g_task_5hz_3Cntr++;
}

uint32_t g_task_5hz_4Cntr = 0;
void task_5hz_4(const float& dt)
{
    g_task_5hz_4Cntr++;
}

uint32_t g_task_5hz_5Cntr = 0;
void task_5hz_5(const float& dt)
{
    g_task_5hz_5Cntr++;
}

// 1 Hz tasks
uint32_t g_task_1hz_1Cntr = 0;
void task_1hz_1(const float& dt)
{
    g_task_1hz_1Cntr++;
}

uint32_t g_task_1hz_2Cntr = 0;
void task_1hz_2(const float& dt)
{
    g_task_1hz_2Cntr++;
}

uint32_t g_task_1hz_3Cntr = 0;
void task_1hz_3(const float& dt)
{
    g_task_1hz_3Cntr++;
}

uint32_t g_task_1hz_4Cntr = 0;
void task_1hz_4(const float& dt)
{
    g_task_1hz_4Cntr++;
}

uint32_t g_task_1hz_5Cntr = 0;
void task_1hz_5(const float& dt)
{
	g_task_1hz_5Cntr++;
}


/*
 * Add 50 tasks with different priority in a random order.
 */
void addAllTasks(Scheduler& scheduler)
{
    // 4 kHz
    scheduler.addTask(TaskType::eMain_fsm, 2, task_4khz_1, FREQUENCY_SLOT::e_4KHZ);
    scheduler.addTask(TaskType::eMain_fsm, 3, task_4khz_2, FREQUENCY_SLOT::e_4KHZ);
    scheduler.addTask(TaskType::eMain_fsm, 0, task_4khz_3, FREQUENCY_SLOT::e_4KHZ);
    scheduler.addTask(TaskType::eMain_fsm, 4, task_4khz_4, FREQUENCY_SLOT::e_4KHZ);
    scheduler.addTask(TaskType::eMain_fsm, 1, task_4khz_5, FREQUENCY_SLOT::e_4KHZ);

    // 2 kHz
    scheduler.addTask(TaskType::eMain_fsm, 2, task_2khz_1, FREQUENCY_SLOT::e_2KHZ);
    scheduler.addTask(TaskType::eMain_fsm, 3, task_2khz_2, FREQUENCY_SLOT::e_2KHZ);
    scheduler.addTask(TaskType::eMain_fsm, 0, task_2khz_3, FREQUENCY_SLOT::e_2KHZ);
    scheduler.addTask(TaskType::eMain_fsm, 4, task_2khz_4, FREQUENCY_SLOT::e_2KHZ);
    scheduler.addTask(TaskType::eMain_fsm, 1, task_2khz_5, FREQUENCY_SLOT::e_2KHZ);

    // 1 kHz
    scheduler.addTask(TaskType::eMain_fsm, 2, task_1khz_1, FREQUENCY_SLOT::e_1KHZ);
    scheduler.addTask(TaskType::eMain_fsm, 3, task_1khz_2, FREQUENCY_SLOT::e_1KHZ);
    scheduler.addTask(TaskType::eMain_fsm, 0, task_1khz_3, FREQUENCY_SLOT::e_1KHZ);
    scheduler.addTask(TaskType::eMain_fsm, 4, task_1khz_4, FREQUENCY_SLOT::e_1KHZ);
    scheduler.addTask(TaskType::eMain_fsm, 1, task_1khz_5, FREQUENCY_SLOT::e_1KHZ);

    // 500 Hz
    scheduler.addTask(TaskType::eMain_fsm, 2, task_500hz_1, FREQUENCY_SLOT::e_500HZ);
    scheduler.addTask(TaskType::eMain_fsm, 3, task_500hz_2, FREQUENCY_SLOT::e_500HZ);
    scheduler.addTask(TaskType::eMain_fsm, 0, task_500hz_3, FREQUENCY_SLOT::e_500HZ);
    scheduler.addTask(TaskType::eMain_fsm, 4, task_500hz_4, FREQUENCY_SLOT::e_500HZ);
    scheduler.addTask(TaskType::eMain_fsm, 1, task_500hz_5, FREQUENCY_SLOT::e_500HZ);

    // 250 Hz
    scheduler.addTask(TaskType::eMain_fsm, 2, task_250hz_1, FREQUENCY_SLOT::e_250HZ);
    scheduler.addTask(TaskType::eMain_fsm, 3, task_250hz_2, FREQUENCY_SLOT::e_250HZ);
    scheduler.addTask(TaskType::eMain_fsm, 0, task_250hz_3, FREQUENCY_SLOT::e_250HZ);
    scheduler.addTask(TaskType::eMain_fsm, 4, task_250hz_4, FREQUENCY_SLOT::e_250HZ);
    scheduler.addTask(TaskType::eMain_fsm, 1, task_250hz_5, FREQUENCY_SLOT::e_250HZ);

    // 100 Hz
    scheduler.addTask(TaskType::eMain_fsm, 2, task_100hz_1, FREQUENCY_SLOT::e_100HZ);
    scheduler.addTask(TaskType::eMain_fsm, 3, task_100hz_2, FREQUENCY_SLOT::e_100HZ);
    scheduler.addTask(TaskType::eMain_fsm, 0, task_100hz_3, FREQUENCY_SLOT::e_100HZ);
    scheduler.addTask(TaskType::eMain_fsm, 4, task_100hz_4, FREQUENCY_SLOT::e_100HZ);
    scheduler.addTask(TaskType::eMain_fsm, 1, task_100hz_5, FREQUENCY_SLOT::e_100HZ);

    // 50 Hz
    scheduler.addTask(TaskType::eMain_fsm, 2, task_50hz_1, FREQUENCY_SLOT::e_50HZ);
    scheduler.addTask(TaskType::eMain_fsm, 3, task_50hz_2, FREQUENCY_SLOT::e_50HZ);
    scheduler.addTask(TaskType::eMain_fsm, 0, task_50hz_3, FREQUENCY_SLOT::e_50HZ);
    scheduler.addTask(TaskType::eMain_fsm, 4, task_50hz_4, FREQUENCY_SLOT::e_50HZ);
    scheduler.addTask(TaskType::eMain_fsm, 1, task_50hz_5, FREQUENCY_SLOT::e_50HZ);

    // 10 Hz
    scheduler.addTask(TaskType::eMain_fsm, 2, task_10hz_1, FREQUENCY_SLOT::e_10HZ);
    scheduler.addTask(TaskType::eMain_fsm, 3, task_10hz_2, FREQUENCY_SLOT::e_10HZ);
    scheduler.addTask(TaskType::eMain_fsm, 0, task_10hz_3, FREQUENCY_SLOT::e_10HZ);
    scheduler.addTask(TaskType::eMain_fsm, 4, task_10hz_4, FREQUENCY_SLOT::e_10HZ);
    scheduler.addTask(TaskType::eMain_fsm, 1, task_10hz_5, FREQUENCY_SLOT::e_10HZ);

    // 5 Hz
    scheduler.addTask(TaskType::eMain_fsm, 2, task_5hz_1, FREQUENCY_SLOT::e_5HZ);
    scheduler.addTask(TaskType::eMain_fsm, 3, task_5hz_2, FREQUENCY_SLOT::e_5HZ);
    scheduler.addTask(TaskType::eMain_fsm, 0, task_5hz_3, FREQUENCY_SLOT::e_5HZ);
    scheduler.addTask(TaskType::eMain_fsm, 4, task_5hz_4, FREQUENCY_SLOT::e_5HZ);
    scheduler.addTask(TaskType::eMain_fsm, 1, task_5hz_5, FREQUENCY_SLOT::e_5HZ);

    // 1 Hz
    scheduler.addTask(TaskType::eMain_fsm, 2, task_1hz_1, FREQUENCY_SLOT::e_1HZ);
    scheduler.addTask(TaskType::eMain_fsm, 3, task_1hz_2, FREQUENCY_SLOT::e_1HZ);
    scheduler.addTask(TaskType::eMain_fsm, 0, task_1hz_3, FREQUENCY_SLOT::e_1HZ);
    scheduler.addTask(TaskType::eMain_fsm, 4, task_1hz_4, FREQUENCY_SLOT::e_1HZ);
    scheduler.addTask(TaskType::eMain_fsm, 1, task_1hz_5, FREQUENCY_SLOT::e_1HZ);
}
