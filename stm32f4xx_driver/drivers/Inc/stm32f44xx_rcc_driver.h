/*
 * stm32f44xx_rcc_driver.h
 *
 *  Created on: May 26, 2020
 *      Author: Darp
 */

#ifndef INC_STM32F44XX_RCC_DRIVER_H_
#define INC_STM32F44XX_RCC_DRIVER_H_

#include "stm32f446xx.h"


// Returns PClock1 Value
uint32_t RCC_GetPCLK1Value(void);

// Returns PClock2 Value
uint32_t RCC_GetPCLK2Value(void);

// Returns PLL Clock Value
uint32_t RCC_GetPLLOutputClock(void);

#endif /* INC_STM32F44XX_RCC_DRIVER_H_ */
