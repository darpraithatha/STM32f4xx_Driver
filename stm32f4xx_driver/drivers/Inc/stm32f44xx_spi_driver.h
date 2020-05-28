/*
 * stm32f44xx_spi_driver.h
 *
 *  Created on: Apr 18, 2020
 *      Author: Darp Raithatha
 */

#ifndef INC_STM32F44XX_SPI_DRIVER_H_
#define INC_STM32F44XX_SPI_DRIVER_H_

#include "stm32f446xx.h"


typedef struct
{

	uint8_t SPI_DeviceMode;				/* Possible Values From @SPI_DeviceMode */
	uint8_t SPI_BusConfig;				/* Possible Values From @SPI_BusConfig */
	uint8_t SPI_SclkSpeed;				/* Possible Values From @SPI_SclkSpeed */
	uint8_t SPI_DFF;					/* Possible Values From @SPI_DFF */
	uint8_t SPI_CPOL;					/* Possible Values From @SPI_CPOL */
	uint8_t SPI_CPHA;					/* Possible Values From @SPI_CPHA */
	uint8_t SPI_SSM;					/* Possible Values From @SPI_SSM */

}SPI_Config_t;


typedef struct
{

	SPI_RegDef_t 	*pSPIx;				/* Base Address of SPI Peripheral (e.g SPI 1,2,3) */
	SPI_Config_t 	SPI_Config;			/* This Holds the SPI Configuration Settings */
	/* The Below  Parameters are Majorly used for Interrupt Mode */
	uint8_t 		*pTxBuffer;			/* To Store the Tx. Buffer Address */
	uint8_t 		*pRxBuffer;			/* To Store the Rx. Buffer Address */
	uint32_t 		TxLen;				/* To Store the Tx. Message */
	uint32_t 		RxLen;				/* To Store the Rx. Message */
	uint8_t 		RxState;			/* To Store the Rx. State */
	uint8_t 		TxState;			/* To Store the Tx. State */

}SPI_Handle_t;


/*
 * @SPI_DeviceMode
 * Possible Value of The Device Mode (Master and Slave)
 */

typedef enum{
	SPI_DEVICE_MODE_SLAVE = 0,
	SPI_DEVICE_MODE_MASTER = 1,

} SPI_DeviceMode_t;

/*
 * @SPI_BusConfig
 * Possible Value of The Bus Configuration (Simplex_Tx, Simplex_Rx, Half_Duplex, Full_Duplex)
 */
typedef enum{

	SPI_BUS_CONFIG_FD = 1,
	SPI_BUS_CONFIG_HD = 2,
	SPI_BUS_CONFIG_SIMPLEX_TX = 1, //Its same as Full Duplex (Just Don't connect the MISO Line)
	SPI_BUS_CONFIG_SIMPLEX_RX = 4, // Don't Connect the MOSI and set the RX_Only Register

} SPI_BusConfig_t;


/*
 * @SPI_SclkSpeed
 * Possible Value of SPI Clock Speed
 */
typedef enum{

	SPI_SCLK_SPEED_DIV2 = 0,
	SPI_SCLK_SPEED_DIV4 = 1,
	SPI_SCLK_SPEED_DIV8 = 2,
	SPI_SCLK_SPEED_DIV16 = 3,
	SPI_SCLK_SPEED_DIV32 = 4,
	SPI_SCLK_SPEED_DIV64 = 5,
	SPI_SCLK_SPEED_DIV128 = 6,
	SPI_SCLK_SPEED_DIV256 = 7,

} SPI_SClkSpeed_t;



/*
 * @SPI_DFF
 * Possible Value of SPI Frame Format
 */
typedef enum{

	SPI_DFF_8BITS = 0,
	SPI_DFF_16BITS = 1,

} SPI_DFF_t;



/*
 * @SPI_CPOL
 * Possible Value of Clock Polarity (0/1 while being ideal)
 */
typedef enum{

	SPI_CPOL_LOW = 0,
	SPI_CPOL_HIGH = 1,

} SPI_CPOL_t;



/*
 * @SPI_CPHA
 * Possible Value of Clock Phase ( First/Second Clock Transition = Capture Edge )
 */
typedef enum{

	SPI_CPHA_LOW = 0,
	SPI_CPHA_HIGH = 1,

} SPI_CPHA_t;

/*
 * @SPI_SSM
 * Possible Value of Software Slave Management (Enable/Disable) (Setting the Slave Select Pin to 0)
 */
typedef enum{

	SPI_SSM_DI = 0,
	SPI_SSM_EN = 1,

} SPI_SSM_t;


/*
 * @SPI_STATES
 * Possible Value SPI Peripheral State (This is a Kind of Semaphore Mechanism for SPI)
 *
 */
typedef enum{

	SPI_ST_READY = 0,
	SPI_ST_BUSY_RX = 1,
	SPI_ST_BUSY_TX = 2,

} SPI_ST_t;


/*
 * @SPI_EVENTS
 * Possible Value SPI Application Events
 *
 */
typedef enum{

	SPI_EVENT_TX_CMPLT = 1,
	SPI_EVENT_RX_CMPT = 2,
	SPI_EVENT_OVR_ERR = 3,
	SPI_EVENT_CRC_ERR = 4,

} SPI_EVENT_t;



/*
 * SPI Related Flag Definition
 */

#define SPI_TXE_FLAG			( 1 << SPI_SR_TXE )
#define SPI_RXNE_FLAG			( 1 << SPI_SR_RXNE )
#define SPI_BUSY_FLAG			( 1 << SPI_SR_BSY )


/*
 * Macros to Reset SPIx Peripheral
 */

#define SPI1_REG_RESET()			do{ (RCC->APB2RSTR |= ( 1 << 12 )); (RCC->APB2RSTR &= ~( 1 << 12 )); }while(0)
#define SPI2_REG_RESET()			do{ (RCC->APB1RSTR |= ( 1 << 14 )); (RCC->APB1RSTR &= ~( 1 << 14 )); }while(0)
#define SPI3_REG_RESET()			do{ (RCC->APB1RSTR |= ( 1 << 15 )); (RCC->APB1RSTR &= ~( 1 << 15 )); }while(0)
#define SPI4_REG_RESET()			do{ (RCC->APB2RSTR |= ( 1 << 13 )); (RCC->APB2RSTR &= ~( 1 << 13 )); }while(0)





/*********************************************************************************************
 * 						API's Supported by this Driver
 * 		For More Information about the API's check the Function Definitions
 ********************************************************************************************/

/*
 * Peripheral Clock Setup
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);


/*
 * SPI Config and De-Config
 */

void SPI_Config(SPI_Handle_t *pSPIHandle);

void SPI_DeConfig(SPI_RegDef_t *pSPIx);

/*
 * SPI Peripheral Control (Enable or Disable the SPI Peripheral)
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);



/*
 * Flag Status Read
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);


/*
 * Data Send and Receive
 */

// Blocking Based
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

//Non-Blocking/Interrupt Based

uint8_t SPI_SendData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);

uint8_t SPI_ReceiveData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);


//DMA Based


/*
 * IRQ Configuration and Handling
 */
void SPI_IRQConfig(uint32_t IRQNumber, uint32_t IRQPriority, uint8_t EnorDi);

void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * Other Peripheral Control API's
 */

// SPI Configuration API
void SPI_SSIConfig (SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig (SPI_RegDef_t *pSPIx, uint8_t EnorDi);

void SPI_ClearOVRFlag (SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission (SPI_Handle_t *pSPIHandle);
void SPI_CloseReception (SPI_Handle_t *pSPIHandle);

/*
 * Application CallBack
 */

void SPI_AppEventCallBack(SPI_Handle_t *pSPIHandle, uint8_t Event);


#endif /* INC_STM32F44XX_SPI_DRIVER_H_ */
