/*
 * pwmWrapperC.c
 *
 *  Created on: May 26, 2025
 *      Author: louis
 */


// Includes from project
#include "Radio/pwmWrapperC.h"

// Includes from HAL
#include "stm32h7xx_ll_exti.h"

#define NUM_CHANNELS 4

static GPIO_TypeDef* channelPorts[NUM_CHANNELS];
static uint16_t channelPins[NUM_CHANNELS];

volatile uint32_t pulseStart[NUM_CHANNELS] = {0};
volatile uint32_t pulseWidth[NUM_CHANNELS] = {0};


void setupRadio()
{
	// Setup PWM reading on 4 pins
	PWM_Init(GPIOB, GPIO_PIN_0,   // PB0
			 GPIOB, GPIO_PIN_1,   // PB1
			 GPIOB, GPIO_PIN_4,   // PB4
			 GPIOB, GPIO_PIN_5);  // PB5
}

void PWM_Init(GPIO_TypeDef* port0, uint16_t pin0,
              GPIO_TypeDef* port1, uint16_t pin1,
              GPIO_TypeDef* port2, uint16_t pin2,
              GPIO_TypeDef* port3, uint16_t pin3)
{
    channelPorts[0] = port0;
    channelPins[0] = pin0;

    channelPorts[1] = port1;
    channelPins[1] = pin1;

    channelPorts[2] = port2;
    channelPins[2] = pin2;

    channelPorts[3] = port3;
    channelPins[3] = pin3;
}

/*
 * Called by external interrupt on pin PB0, PB1, PB4 & PB5
 * Read the PWM high time
 */
void PWM_EXTI_Callback(uint16_t GPIO_Pin)
{
	__disable_irq();

    for (int i = 0; i < NUM_CHANNELS; ++i)
    {
        if (GPIO_Pin == channelPins[i])
        {
            if (HAL_GPIO_ReadPin(channelPorts[i], channelPins[i]) == GPIO_PIN_SET)
            {
            	pulseStart[i] = timerCounterGetCycles();
            }
            else
            {
				uint32_t pulse = getEllapsedTime_us(pulseStart[i]);
				if (pulse > 950 && pulse < 2200)
				{
					pulseWidth[i] = pulse;
				}
				else
				{
					pulseWidth[i] = 0; // Signal lost
				}
            }

            break;
        }
    }

    __enable_irq();
}

uint32_t PWM_GetPulse(int channel)
{
	uint32_t tmp;
    __disable_irq();
    tmp = pulseWidth[channel];
    __enable_irq();
    return tmp;
}
