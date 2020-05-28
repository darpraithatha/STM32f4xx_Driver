/*
 * 005SPI_button_tx_test.c
 *
 *  Created on: Apr 22, 2020
 *      Author: Darp Raithatha
 */

#include <string.h>
#include "stm32f446xx.h"

/*
 * PB14 -> SPI2_MISO
 * PB15 -> SPI2_MOSI
 * PB13 -> SPI2_SCLK
 * PB12 -> SPI2_NSS
 * ALT_Functionaly_Mode: 5
 *
 */

void SPI2_GPIOInits(void){

	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PP_NO_PUPD;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

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


void SPI2_Inits(void){

	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;

	SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	SPI2Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_DI;

	SPI_Config(&SPI2Handle);

}

void delay (void){

	for (uint32_t i = 0; i < 500000/2 ; i++);
}


int main(){

	char user_data[] = "Hello World";

	//This Function is used to initialize the GPIO Pins to behave as SPI2 Pins
	SPI2_GPIOInits();

	// Initialize the GPIO Button
	GPIO_ButtonInit();

	//This Function is to initialize the SPI2 Peripheral
	SPI2_Inits();

	//SSOI Enable
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1){
		while( GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		delay();

		//Enable the SPI2 Peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		//Send the Data
		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

		while( SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) );

		//Disable the SPI2 Peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}
	return 0;
}




