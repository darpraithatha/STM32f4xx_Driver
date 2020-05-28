/*
 * stm32f4xx_gpoi_driver.h
 *
 *  Created on: Apr 11, 2020
 *      Author: Darp Raithatha
 */

#ifndef INC_STM32F44XX_GPIO_DRIVER_H_
#define INC_STM32F44XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"


/*
 *  The Configuration Structure for GPIO Pin
 */

typedef struct
{

	uint8_t GPIO_PinNumber;				/* Possible Values From @GPIO_PIN_NUMBER */
	uint8_t GPIO_PinMode;				/* Possible Values From @GPIO_POSSIBLE_MODES */
	uint8_t GPIO_PinSpeed;				/* Possible Values From @GPIO_POSSIBLE_OUTPUT_SPEEDS */
	uint8_t GPIO_PinPuPdControl;		/* Possible Values From @GPIO_POSSIBLE_PULL-UP/PULL-DOWN CONFIGUARTION */
	uint8_t GPIO_PinOPType;				/* Possible Values From @GPIO_POSSIBLE_OUTPUT_TYPES */
	uint8_t GPIO_PinAltFunMode;			/* Possible Values From */

}GPIO_PinConfig_t;


typedef struct
{

	GPIO_RegDef_t *pGPIOx;				/* Base Address of GPIO Port to which the Pin Belongs */
	GPIO_PinConfig_t GPIO_PinConfig;	/* This Holds the GPIO Pin Configuration Settings */

}GPIO_Handle_t;


/*
 * @GPIO_PIN_NUMBER
 * GPIO Pin Numbers
 */

#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15


/*
 * @GPIO_POSSIBLE_MODES
 * GPIO Pin Possible Modes
 */

typedef enum{

	GPIO_MODE_IN = 0,					// Input Mode
	GPIO_MODE_OUT = 1,					// Output Mode
	GPIO_MODE_ALTFN = 2,				// Alternate Function Mode
	GPIO_MODE_ANALOG = 3,				// Analog Mode
	GPIO_MODE_IT_FT = 4,				// Interrupt Mode (Falling Edge)
	GPIO_MODE_IT_RT = 5,				// Interrupt Mode (Rising Edge)
	GPIO_MODE_IT_RFT = 6,				// Interrupt Mode (Rising-Falling)

}GPIO_MODE_t;

/*
#define GPIO_MODE_IN		0			//Input Mode
#define GPIO_MODE_OUT		1			// Output Mode
#define GPIO_MODE_ALTFN		2			// Alternate Function Mode
#define GPIO_MODE_ANALOG	3			// Analog Mode
#define GPIO_MODE_IT_FT		4			// Interrupt Mode (Falling Edge)
#define GPIO_MODE_IT_RT		5			// Interrupt Mode (Rising Edge)
#define GPIO_MODE_IT_RFT	6			// Interrupt Mode (Rising-Falling)
*/

/*
 * @GPIO_POSSIBLE_OUTPUT_TYPES
 * GPIO Pin Possible Output Type
 */

typedef enum{
	GPIO_OP_TYPE_PP = 0,			/* Pull-up Pull-down Mode */
	GPIO_OP_TYPE_OD = 1,			/* Open Drain Mode */
}GPIO_OP_TYPE_t;

/*
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1
*/


/*
 * @GPIO_POSSIBLE_OUTPUT_SPEEDS
 * GPIO Pin Possible Output Speeds
 */

typedef enum{
	GPIO_SPEED_LOW = 0,				/* Low Speed */
	GPIO_SPEED_MEDIUM = 1,			/* Medium Speed */
	GPIO_SPEED_FAST = 2,			/* Fast Speed */
	GPIO_SPEED_HIGH = 3,			/* Very Fast Speed */

}GPIO_SPEED_t;

/*
#define GPIO_SPEED_LOW		0		// Low Speed
#define GPIO_SPEED_MEDIUM 	1		// Medium Speed
#define GPIO_SPEED_FAST 	2		// Fast Speed
#define GPIO_SPEED_HIGH		3		// Very Fast Speed
*/


/*
 * @GPIO_POSSIBLE_PULL-UP/PULL-DOWN CONFIGUARTION
 * GPIO Pin Pull-up and Pull-down Configuration
 */

typedef enum{
	GPIO_PP_NO_PUPD = 0,					/* No Pull-up/Pull-down */
	GPIO_PP_PU = 1,					/* No Pull-up/Pull-down */
	GPIO_PP_PD = 2,					/* No Pull-up/Pull-down */
} GPIO_PP_t;

/*
#define GPIO_NO_PUPD		0			// No Pull-up/Pull-down
#define GPIO_PIN_PU			1			// Pull-Up
#define GPIO_PIN_PD		 	2			// Pull-Down
*/

/*
 * Macros to Reset GPIOx Peripheral
 */

#define GPIOA_REG_RESET()			do{ (RCC->AHB1RSTR |= ( 1 << 0 )); (RCC->AHB1RSTR &= ~( 1 << 0 )); }while(0)
#define GPIOB_REG_RESET()			do{ (RCC->AHB1RSTR |= ( 1 << 1 )); (RCC->AHB1RSTR &= ~( 1 << 1 )); }while(0)
#define GPIOC_REG_RESET()			do{ (RCC->AHB1RSTR |= ( 1 << 2 )); (RCC->AHB1RSTR &= ~( 1 << 2 )); }while(0)
#define GPIOD_REG_RESET()			do{ (RCC->AHB1RSTR |= ( 1 << 3 )); (RCC->AHB1RSTR &= ~( 1 << 3 )); }while(0)
#define GPIOE_REG_RESET()			do{ (RCC->AHB1RSTR |= ( 1 << 4 )); (RCC->AHB1RSTR &= ~( 1 << 4 )); }while(0)
#define GPIOF_REG_RESET()			do{ (RCC->AHB1RSTR |= ( 1 << 5 )); (RCC->AHB1RSTR &= ~( 1 << 5 )); }while(0)
#define GPIOG_REG_RESET()			do{ (RCC->AHB1RSTR |= ( 1 << 6 )); (RCC->AHB1RSTR &= ~( 1 << 6 )); }while(0)
#define GPIOH_REG_RESET()			do{ (RCC->AHB1RSTR |= ( 1 << 7 )); (RCC->AHB1RSTR &= ~( 1 << 7 )); }while(0)

/*********************************************************************************************
 * 						API's Supported by this Driver
 * 		For More Information about the API's check the Function Definitions
 ********************************************************************************************/

/*
 * Peripheral Clock Setup
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);


/*
 * GPIO Init and De-Init
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * GPIO Read and Write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint8_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIO);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


/*
 * IRQ Configuration and ISR Handling
 */

void GPIO_IRQConfig(uint32_t IRQNumber, uint32_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* INC_STM32F44XX_GPIO_DRIVER_H_ */
