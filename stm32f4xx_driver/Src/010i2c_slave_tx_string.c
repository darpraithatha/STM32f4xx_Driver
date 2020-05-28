/*
 * 010i2c_slave_tx_string.c
 *
 *  Created on: May 21, 2020
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

#define MY_ADDR 0x68

I2C_Handle_t I2C1Handle;

//Rx_Buffer
uint8_t TxBuffer[32] = "STM32 Slave Mode Testing!";

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

	//I2C Pins Inits
	I2C1_GPIOInits();
	GPIO_ButtonInit();

	//I2C 1 Inits
	I2C1_Inits();

	//IRQ Config
	I2C_IRQConfig(IRQ_NO_I2C1_ER, 2, ENABLE);
	I2C_IRQConfig(IRQ_NO_I2C1_EV, 1, ENABLE);

	//Enable Interrupts
	I2C_SlaveCallBackEvents(I2C1, ENABLE);

	//Enable I2C Peripheral
	I2C_PeriClockControl(I2C1, ENABLE);

	//Enable Acking
	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	while(1);

}


void I2C1_EV_IRQHandler(void){

	I2C_EV_IRQHandling(&I2C1Handle);
}


void I2C1_ER_IRQHandler(void){

	I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C_AppEventCallBack(I2C_Handle_t *pI2CHandle, uint8_t Event){

	static uint8_t commandCode = 0;
	static uint8_t Cnt = 0;
	// Data Request From Master
	if ( Event == I2C_EVENT_DATA_REQ ){
		// Master wants to send some Data
		if (commandCode == 0x51){
			// Master is Requesting the Length Information
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char*)TxBuffer));
		}
		else if (commandCode == 0x52 ){
			// Master is Requesting the Data
			I2C_SlaveSendData(pI2CHandle->pI2Cx, TxBuffer[Cnt++]);
		}
	}

	// Command is Received from Master
	else if ( Event == I2C_EVENT_DATA_RCV ) {
		// Save the Command in Command Code
		commandCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
	}

	else if ( Event == I2C_EVENT_AF_ERR ){

		// This Happens during Slave Transmission
		// Master has Send NACK and don't want any more data from Slave
		commandCode = 0xff;
		Cnt = 0;
	}

	else if ( Event == I2C_EVENT_STOP_ERR ){

		// This Happens only During Slave Reception
		// Master has Ended I2C Communication with the Slave ( Stop Condition is Detected )
		Cnt = 0;
	}

}


