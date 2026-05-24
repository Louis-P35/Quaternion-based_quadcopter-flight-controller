/*
 * interfaceMain.cpp
 *
 *  Created on: May 24, 2025
 *      Author: louis
 */


/*
 * This file is an interface between main.c which is generated .c and flightCore.cpp
 */

// Includes from HAL
#include "stm32h7xx_hal.h"

// Includes from project
#include "main.h"
#include "Scheduler/scheduler.hpp"
#include "flightCore.hpp"
#include "Utils/utilsTimer.hpp"
#include "FSM/stateMachine.hpp"

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_tx;

extern "C"
{
extern Scheduler g_scheduler;

// FlightCore instance is a pointer, because some constructor it call need
// some hardware to be initialized, so we must instantiate it later.
FlightCore* g_pFlightCore = nullptr;
}

extern "C" void interfaceMain()
{
	/*
	 * Init and reset the timer counter
	 */
	timerCounterInit();
	timerCounterReset();

	FlightCore flightCoreInstance(SPI_CS_Pin, SPI_CS_GPIO_Port, SPI_CS_ESP_Pin, SPI_CS_ESP_GPIO_Port);
	g_pFlightCore = &flightCoreInstance;

	g_pFlightCore->mainSetup();

    uint32_t start = timerCounterGetCycles();
    HAL_Delay(1);

    while (true)
    {
        const double dt = getEllapsedTime_s(start);
        start = timerCounterGetCycles();
        mainLoop(dt);
    }
}

