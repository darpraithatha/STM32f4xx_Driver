/*
 * 011usart_tx.c
 *
 *  Created on: May 27, 2020
 *      Author: Darp Raithatha
 */

#include "stm32f446xx.h"
#include <string.h>

USART_Handle_t usart2_handle;

char msg[1024] = "USART Tx Testing.....\n\r";

void delay (void){

	for (uint32_t i = 0; i < 500000/2 ; i++);
}


void USART2_Init(void){

	usart2_handle.pUSARTx = USART2;
	usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2_handle.USART_Config.USART_Mode = USART_MODE_TX;
	usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	usart2_handle.USART_Config.USART_wordLength = USART_WORDLEN_8BITS;
	USART_Config(&usart2_handle);

}

void GPIO_ButtonInit(void){

	GPIO_Handle_t GPIOBtn;

	//GPIOBtn Configuration
	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PP_NO_PUPD;

	GPIO_Init(&GPIOBtn);
}



void USART2_GPIOInit(void){

	GPIO_Handle_t usart_gpios;

	usart_gpios.pGPIOx = GPIOA;
	usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_gpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PP_PU;
	usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart_gpios.GPIO_PinConfig.GPIO_PinAltFunMode = 7;

	// USART2 TX
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&usart_gpios);

	// USART RX
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&usart_gpios);

}



int main(void){

	// Initializing the GPIO Pins in AltFun Mode
	USART2_GPIOInit();

	// Initializing the GPIO Pins
	GPIO_ButtonInit();

	// Initializing the USART2
	USART2_Init();

	// Enable USART Peripheral
	USART_PeripheralControl(USART2, ENABLE);

	while (1){
		while( GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) );
		// Debouncing
		delay();
		USART_SendData(&usart2_handle, (uint8_t*)msg, strlen(msg));
	}
	return 0;
}
