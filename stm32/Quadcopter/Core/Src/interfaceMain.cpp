/*
 * interfaceMain.cpp
 *
 *  Created on: May 24, 2025
 *      Author: louis
 */


#include "main.h"
#include "stm32h7xx_hal.h"

#include "scheduler.hpp"
#include "Utils/utilsTimer.hpp"
#include "stateMachine.hpp"

extern SD_HandleTypeDef hsd1;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_tx;

extern "C"
{
Scheduler g_scheduler(SPI_CS_Pin, SPI_CS_GPIO_Port);
}

extern "C" void interfaceMain()
{
	/*
	 * Init and reset the timer counter
	 */
	timerCounterInit();
	timerCounterReset();


	g_scheduler.mainSetup();

    uint32_t start = timerCounterGetCycles();
    HAL_Delay(1);

    while (true)
    {
        const double dt = getEllapsedTime_s(start);
        start = timerCounterGetCycles();
        g_scheduler.mainLoop(dt);
    }
}

