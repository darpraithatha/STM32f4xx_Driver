/*
 * 006spi_command_handling.c
 *
 *  Created on: Apr 23, 2020
 *      Author: Darp Raithatha
 */

#include <stdio.h>
#include <string.h>
#include "stm32f446xx.h"


//Command Codes

#define COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x54

#define LED_ON			1
#define LED_OFF 		0

//Arduino Analog Pins
#define ANALOG_PIN_0	0
#define ANALOG_PIN_1	1
#define ANALOG_PIN_2	2
#define ANALOG_PIN_3	3
#define ANALOG_PIN_4	4
#define ANALOG_PIN_5	5

//Arduino LED

#define LED_PIN			9

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

uint8_t SPI_VerifyResponse(uint8_t ackbyte){
	if (ackbyte == 0xF5){
		//ack
		return 1;
	}
	else {
		//nack
		return 0;
	}
}

void delay (void){

	for (uint32_t i = 0; i < 500000/2 ; i++);
}


int main(){

	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

	//This Function is used to initialize the GPIO Pins to behave as SPI2 Pins
	SPI2_GPIOInits();

	//This Function is to initialize the SPI2 Peripheral
	SPI2_Inits();

	//SSOI Enable
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1){
		while( GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		delay();

		//Enable the SPI2 Peripheral
		SPI_PeripheralControl(SPI2, ENABLE);


		//1. CMD_LED_CTRL			<pin_no(1)>			<value(1)>
		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		//Send the Command
		SPI_SendData(SPI2, &commandcode, 1);

		//do dummy Read to clear RxNe
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//Send some Dummy Bytes to to Fetch the Response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if (SPI_VerifyResponse(ackbyte)){
			//Send other Arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI2, args, 2);

		}
		//End of First Command

		//2. CMD_SENSOR_READ		<analog pin number>
		while( GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) );

		delay();

		commandcode = COMMAND_SENSOR_READ;

		//Send the Command
		SPI_SendData(SPI2, &commandcode, 1);

		//do dummy Read to clear RxNe
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//Send some Dummy Bytes to to Fetch the Response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if (SPI_VerifyResponse(ackbyte)){
			//Send other Arguments
			args[0] = ANALOG_PIN_0;

			SPI_SendData(SPI2, args, 1);

			//do dummy Read to clear RxNe
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			//Delay for analogRead
			delay();

			//Send some Dummy Bytes to to Fetch the Response from the slave
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t analog_read;
			SPI_ReceiveData(SPI2, &analog_read, 1);
		}
		//End of Second Command


		//3. CMD_LED_READ 		<pin number>
		while( GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		delay();

		commandcode = COMMAND_LED_READ;

		//Send the Command
		SPI_SendData(SPI2, &commandcode, 1);

		//do dummy Read to clear RxNe
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//Send some Dummy Bytes to to Fetch the Response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if (SPI_VerifyResponse(ackbyte)){
			//Send other Arguments
			args[0] = LED_PIN;

			SPI_SendData(SPI2, args, 1);

			//do dummy Read to clear RxNe
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			//Delay for analogRead
			delay();

			//Send some Dummy Bytes to to Fetch the Response from the slave
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t led_read;
			SPI_ReceiveData(SPI2, &led_read, 1);
		}
		//End of Third Command

		//4. CMD_PRINT			<len(2)>	<message(m)>
		while( GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		delay();

		commandcode = COMMAND_PRINT;

		//Send the Command
		SPI_SendData(SPI2, &commandcode, 1);

		//do dummy Read to clear RxNe
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//Send some Dummy Bytes to to Fetch the Response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		char message[] = "Hello Arduino :)";

		if (SPI_VerifyResponse(ackbyte)){
			//Send other Arguments
			args[0] = strlen((char*)message);

			SPI_SendData(SPI2, args, 1);

			//Send the Message
			SPI_SendData(SPI2, (uint8_t*)message, args[0]);

		}
		//End of Fourth Command


		//5. CMD_ID_READ
		while( GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		delay();

		commandcode = COMMAND_ID_READ;

		//Send the Command
		SPI_SendData(SPI2, &commandcode, 1);

		//do dummy Read to clear RxNe
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//Send some Dummy Bytes to to Fetch the Response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		SPI_ReceiveData(SPI2, &ackbyte, 1);

		uint8_t id[11];
		uint32_t i=0;

		if (SPI_VerifyResponse(ackbyte)){
			for (i=0; i<10; i++){
				SPI_SendData(SPI2, &dummy_write, 1);
				SPI_ReceiveData(SPI2, &id[i], 1);
			}

			id[11] = '\0';

			printf("COMMAND_ID: %s \n", id);
		}
		//End of Fifth Command

		while( SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) );

		//Disable the SPI2 Peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}
	return 0;
}




