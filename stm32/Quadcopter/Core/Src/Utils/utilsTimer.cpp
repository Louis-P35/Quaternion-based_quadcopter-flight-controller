/*
 * utilsTimer.cpp
 *
 *  Created on: Jul 20, 2024
 *      Author: Louis
 */

// STL
#include "stdint.h"

// Project
#include "stm32h7xx_hal.h"
#include "Utils/utilsTimer.hpp"



/*
 * Init the timer counter
 */
void timerCounterInit()
{
	// Enable TRC (Trace Control Register)
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	// Enable the counter
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/*
 * Reset the timer counter
 */
void timerCounterReset()
{
    // Reset the cycle counter
    DWT->CYCCNT = 0;
}

/*
 * Get the current timer counter value
 */
uint32_t timerCounterGetCycles()
{
    // Return the current cycle count
    return DWT->CYCCNT;
}

/*
 * Get the ellapsed time is us between
 * start and now.
 */
uint32_t getEllapsedTime_us(const uint32_t start)
{
    // Get the end cycle count
    uint32_t end = timerCounterGetCycles();

    // Calculate the number of cycles elapsed (unsigned handle one overflow natively)
    uint32_t cycles_elapsed = end - start;

    // Calculate the time in microseconds
     return (cycles_elapsed / (SystemCoreClock / 1000000));
}

/*
 * Get the ellapsed time in second between
 * start and now.
 */
double getEllapsedTime_s(const uint32_t start)
{
    return static_cast<double>(getEllapsedTime_us(start)) / 1000000.0;
}


