/*
 * schedulerUnitTests.hpp
 *
 *  Created on: Jul 12, 2025
 *      Author: louis
 */

#pragma once

// Includes from project
#include "Scheduler/scheduler.hpp"

void task_4khz_1(const float& dt);
void task_4khz_2(const float& dt);
void task_4khz_3(const float& dt);
void task_4khz_4(const float& dt);
void task_4khz_5(const float& dt);

void task_2khz_1(const float& dt);
void task_2khz_2(const float& dt);
void task_2khz_3(const float& dt);
void task_2khz_4(const float& dt);
void task_2khz_5(const float& dt);

void task_1khz_1(const float& dt);
void task_1khz_2(const float& dt);
void task_1khz_3(const float& dt);
void task_1khz_4(const float& dt);
void task_1khz_5(const float& dt);

void task_500hz_1(const float& dt);
void task_500hz_2(const float& dt);
void task_500hz_3(const float& dt);
void task_500hz_4(const float& dt);
void task_500hz_5(const float& dt);

void task_250hz_1(const float& dt);
void task_250hz_2(const float& dt);
void task_250hz_3(const float& dt);
void task_250hz_4(const float& dt);
void task_250hz_5(const float& dt);

void task_100hz_1(const float& dt);
void task_100hz_2(const float& dt);
void task_100hz_3(const float& dt);
void task_100hz_4(const float& dt);
void task_100hz_5(const float& dt);

void task_50hz_1(const float& dt);
void task_50hz_2(const float& dt);
void task_50hz_3(const float& dt);
void task_50hz_4(const float& dt);
void task_50hz_5(const float& dt);

void task_10hz_1(const float& dt);
void task_10hz_2(const float& dt);
void task_10hz_3(const float& dt);
void task_10hz_4(const float& dt);
void task_10hz_5(const float& dt);

void task_5hz_1(const float& dt);
void task_5hz_2(const float& dt);
void task_5hz_3(const float& dt);
void task_5hz_4(const float& dt);
void task_5hz_5(const float& dt);

void task_1hz_1(const float& dt);
void task_1hz_2(const float& dt);
void task_1hz_3(const float& dt);
void task_1hz_4(const float& dt);
void task_1hz_5(const float& dt);

void addAllTasks(Scheduler& scheduler);
