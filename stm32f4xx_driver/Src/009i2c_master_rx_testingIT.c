/*
 * 008i2c_master_rx_testing.c
 *
 *  Created on: May 5, 2020
 *      Author: Darp Raithatha
 */


#include <stdio.h>
#include <string.h>
#include "stm32f446xx.h"

/*
 * PB6 -> SCL
 * PB9 -> SDA
 *
 */

#define MY_ADDR 0x61
#define SLAVE_ADDR 0x68

I2C_Handle_t I2C1Handle;

//Rx_Buffer
uint8_t RxBuffer[32];

//Flag Variable
uint8_t rxComplt = RESET;

// Delay
void delay (void){

	for (uint32_t i = 0; i < 500000/2 ; i++);
}

// GPIO Init
void I2C1_GPIOInits(void){

	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PP_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;


	//SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	//SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&I2CPins);

}

void I2C1_Inits(void){

	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Config(&I2C1Handle);

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



int main(void){

	uint8_t command_code;
	uint8_t len;

	//I2C Pins Inits
	I2C1_GPIOInits();
	GPIO_ButtonInit();

	//I2C 1 Inits
	I2C1_Inits();

	//IRQ Config
	I2C_IRQConfig(IRQ_NO_I2C1_ER, 2, ENABLE);
	I2C_IRQConfig(IRQ_NO_I2C1_EV, 1, ENABLE);

	//Enable I2C Peripheral
	I2C_PeriClockControl(I2C1, ENABLE);

	//Enable Acking
	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	while(1){

		//Wait till the Button is Pressed
		while( GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) );

		delay();

		//Send the Command Code to Read the Length of the Data
		command_code = 0x51;
		while ( I2C_MasterSendDataIT(&I2C1Handle, &command_code, 1, SLAVE_ADDR, I2C_SR_ENABLE) != I2C_ST_READY );

		//Read the Response ( i.e Length of the Data )
		while ( I2C_MasterReceiveDataIT(&I2C1Handle, &len, 1, SLAVE_ADDR, I2C_SR_ENABLE) != I2C_ST_READY );

		//Send the Command Code to Read the Data of the Provided Length
		command_code = 0x52;
		while ( I2C_MasterSendDataIT(&I2C1Handle, &command_code, 1, SLAVE_ADDR, I2C_SR_ENABLE) != I2C_ST_READY );

		//Read the Data From the Slave
		while ( I2C_MasterReceiveDataIT(&I2C1Handle, RxBuffer, len, SLAVE_ADDR, I2C_SR_DISABLE) != I2C_ST_READY );

		rxComplt = RESET;

		// Wait till RX Completes
		while (rxComplt != SET);
		RxBuffer[len+1] = '\0';

		printf("Data: %s", RxBuffer);

		rxComplt = RESET;
	}

}


void I2C1_EV_IRQHandler(void){

	I2C_EV_IRQHandling(&I2C1Handle);
}


void I2C1_ER_IRQHandler(void){

	I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C_AppEventCallBack(I2C_Handle_t *pI2CHandle, uint8_t Event){
	if (Event == I2C_EVENT_TX_CMPLT){
		printf("Tx is Complete\n");
	}
	else if (Event == I2C_EVENT_RX_CMPT){
		printf("Rx is Complete\n");
		rxComplt = SET;
	}
	else if (Event == I2C_EVENT_AF_ERR){
		printf("Ack Failure \n");

		//Close the Communication
		I2C_CloseSendData(pI2CHandle);

		//Generate the Stop Condition
		I2C_GenerateStopCondition(I2C1);

		//Don't Proceed Further
		while (1);

	}
}
