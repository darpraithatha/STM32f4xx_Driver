/*
 * stm32f44xx_usart_driver.h
 *
 *  Created on: May 23, 2020
 *      Author: Darp Raithatha
 */

#ifndef INC_STM32F44XX_USART_DRIVER_H_
#define INC_STM32F44XX_USART_DRIVER_H_

#include "stm32f446xx.h"


typedef struct{

	uint8_t USART_Mode;
	uint32_t USART_Baud;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_wordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;

} USART_Config_t;


typedef struct{

	USART_RegDef_t *pUSARTx;
	USART_Config_t USART_Config;
	uint8_t TxState;
	uint8_t RxState;
	uint8_t* pTxBuffer;
	uint8_t* pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;

}USART_Handle_t;


/*
 * @USART_MODE
 * Possible Values of USART Mode
 */
typedef enum{

	USART_MODE_TX = 0,
	USART_MODE_RX = 1,
	USART_MODE_TXRX = 2,

}USART_MODE_t;


/*
 * @USART_Baud
 * Possible Values of USART Baud
 */
typedef enum{

	USART_STD_BAUD_1200 = 1200,
	USART_STD_BAUD_2400 = 2400,
	USART_STD_BAUD_9600 = 9600,
	USART_STD_BAUD_19200 = 19200,
	USART_STD_BAUD_38400 = 38400,
	USART_STD_BAUD_57600 = 57600,
	USART_STD_BAUD_115200 = 115200,
	USART_STD_BAUD_230400 = 230400,
	USART_STD_BAUD_460800 = 460800,
	USART_STD_BAUD_921600 = 921600,
	USART_STD_BAUD_2M = 2000000,
	USART_STD_BAUD_3M = 3000000,

} USART_BAUD_t;

/*
 * @USART_NoOfStopBits
 * Possible Values of Parity Control
 */
typedef enum{

	USART_STOPBITS_1 = 0,
	USART_STOPBITS_0_5 = 1,
	USART_STOPBITS_2 = 2,
	USART_STOPBITS_1_5 = 3,

} USART_STOPBITS_t;


/*
 * @USART_wordLength
 * Possible Values of Word Length
 */
typedef enum{

	USART_WORDLEN_8BITS = 0,
	USART_WORDLEN_9BITS = 1,

} USART_WORDLEN_t;



/*
 * @USART_ParityControl
 * Possible Values of No of Stop Bits
 */
typedef enum{

	USART_PARITY_DISABLE = 0,
	USART_PARITY_EN_EVEN = 1,
	USART_PARITY_EN_ODD = 2,

} USART_PARITY_t;



/*
 * @USART_HWFlowControl
 * Possible Values of Word Length
 */
typedef enum{

	USART_HW_FLOW_CTRL_NONE = 0,
	USART_HW_FLOW_CTRL_CTS = 1,
	USART_HW_FLOW_CTRL_RTS = 2,
	USART_HW_FLOW_CTRL_CTS_RTS = 3,

} USART_HW_FLOW_CTRL_t;



/*
 * @USART_STATE
 * This is to Indicate the USART Peripheral State (Ready, Busy_Rx, Busy_Tx)
 */

typedef enum{

	USART_ST_READY = 0,
	USART_ST_BUSY_RX = 1,
	USART_ST_BUSY_TX = 2,

} USART_ST_t;


/*
 * @USART_EVENTS
 * Possible Value USART Application Events
 *
 */
typedef enum{

	USART_EVENT_TX_CMPLT = 0,			// Transmission is Complete
	USART_EVENT_RX_CMPLT = 1,			// Reception is Complete
	USART_EVENT_CTS = 2, 				// CTS is Detected
	USART_EVENT_IDLE = 3,				// IDLE
	USART_EVENT_ORE = 4,				// Overrun
	USART_ERREVENT_FE = 5,				// Failure Error
	USART_ERREVENT_NF =	6,				// Noise Flag
	USART_ERREVENT_ORE = 7,				// Overrun Error

} USART_EVENT_t;




/*
 * USART Related Flag Definition
 */

#define USART_FLAG_CTS					( 1 << USART_SR_CTS )
#define USART_FLAG_FE					( 1 << USART_SR_FE )
#define USART_FLAG_IDLE					( 1 << USART_SR_IDLE )
#define USART_FLAG_LBD					( 1 << USART_SR_LBD )
#define USART_FLAG_NF					( 1 << USART_SR_NF )
#define USART_FLAG_ORE					( 1 << USART_SR_ORE )
#define USART_FLAG_PE					( 1 << USART_SR_PE )
#define USART_FLAG_RXNE					( 1 << USART_SR_RXNE )
#define USART_FLAG_TC					( 1 << USART_SR_TC )
#define USART_FLAG_TXE					( 1 << USART_SR_TXE )



/*
 * Macros to Reset USARTx Peripheral
 */

#define USART1_REG_RESET()			do{ (RCC->APB2RSTR |= ( 1 << 4 )); (RCC->APB1RSTR &= ~( 1 << 4 )); }while(0)
#define USART2_REG_RESET()			do{ (RCC->APB1RSTR |= ( 1 << 17 )); (RCC->APB1RSTR &= ~( 1 << 17 )); }while(0)
#define USART3_REG_RESET()			do{ (RCC->APB1RSTR |= ( 1 << 18 )); (RCC->APB1RSTR &= ~( 1 << 18 )); }while(0)
#define UART4_REG_RESET()			do{ (RCC->APB1RSTR |= ( 1 << 19 )); (RCC->APB1RSTR &= ~( 1 << 19 )); }while(0)
#define UART5_REG_RESET()			do{ (RCC->APB1RSTR |= ( 1 << 20 )); (RCC->APB1RSTR &= ~( 1 << 20 )); }while(0)
#define USART6_REG_RESET()			do{ (RCC->APB2RSTR |= ( 1 << 5 )); (RCC->APB1RSTR &= ~( 1 << 5 )); }while(0)





/*********************************************************************************************
 * 						API's Supported by this Driver
 * 		For More Information about the API's check the Function Definitions
 ********************************************************************************************/

/*
 * Peripheral Clock Setup
 */

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);


/*
 * USART Config and De-Config
 */

void USART_Config(USART_Handle_t *pUSARTHandle);
void USART_DeConfig(USART_Handle_t *pUSARTHandle);


/*
 * Data Send and Receive
 */

// Blocking Call
void USART_SendData(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

// Interrupt Based
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

//DMA Based


/*
 * IRQ Configuration and Handling
 */
void USART_IRQConfig(uint32_t IRQNumber, uint32_t IRQPriority, uint8_t EnorDi);
void USART_IRQHandling(USART_Handle_t *pHandle);


//Interrupt Handler



/*
 * Other Peripheral Control API's
 */

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName);

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint32_t FlagName);

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

/*
 * Application CallBack
 */

void USART_AppEventCallBack(USART_Handle_t *pUSARTHandle, uint8_t Event);


#endif /* INC_STM32F44XX_USART_DRIVER_H_ */
