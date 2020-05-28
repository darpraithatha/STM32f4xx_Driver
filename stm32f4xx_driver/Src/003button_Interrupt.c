/*
 * 002led_button.c
 *
 *  Created on: Apr 15, 2020
 *      Author: Darp Raithatha
 */

#include <string.h>
#include "stm32f446xx.h"

void delay (void){

	for (uint32_t i = 0; i < 500000/2 ; i++);
}

int main(void){

	GPIO_Handle_t GpioLed, GPIOBtn;
	memset(&GpioLed, 0, sizeof(GpioLed));
	memset(&GPIOBtn, 0, sizeof(GPIOBtn));

	//GpioLed Configuration
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; //GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PP_PU;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GpioLed);

	//GPIOBtn Configuration
	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//GPIOBtn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PP_PU;

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&GPIOBtn);

	//IRQ Configuration
	GPIO_IRQConfig(IRQ_NO_EXTI15_10, 15, ENABLE);

	while(1);
}

void EXTI15_10_IRQHandler(void){

	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_13);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
}



