/*
 * utilsTimer.hpp
 *
 *  Created on: Jul 20, 2024
 *      Author: Louis
 */

#pragma once


/*
 * Init the timer counter
 */
void timerCounterInit();

/*
 * Reset the timer counter
 */
void timerCounterReset();

/*
 * Get the current timer counter value
 */
uint32_t timerCounterGetCycles();

/*
 * Get the ellapsed time is us between
 * start and now.
 */
uint32_t getEllapsedTime_us(const uint32_t start);

/*
 * Get the ellapsed time is second between
 * start and now.
 */
double getEllapsedTime_s(const uint32_t start);
