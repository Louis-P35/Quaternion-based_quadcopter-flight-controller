/*
 * pwmWrapperC.h
 *
 *  Created on: May 26, 2025
 *      Author: louis
 */

#ifndef INC_PWM_PWMWRAPPERC_H_
#define INC_PWM_PWMWRAPPERC_H_

#include "stm32h7xx_hal.h"
#include "Utils/utilsTimer.hpp"

#ifdef __cplusplus
extern "C" {
#endif

void setupRadio();

void PWM_Init(GPIO_TypeDef* port0, uint16_t pin0,
              GPIO_TypeDef* port1, uint16_t pin1,
              GPIO_TypeDef* port2, uint16_t pin2,
              GPIO_TypeDef* port3, uint16_t pin3);

void PWM_EXTI_Callback(uint16_t GPIO_Pin);

uint32_t PWM_GetPulse(int channel);

#ifdef __cplusplus
}
#endif


#endif /* INC_PWM_PWMWRAPPERC_H_ */
