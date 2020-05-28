/*
 * stm32f44xx_i2c_driver.h
 *
 *  Created on: Apr 29, 2020
 *      Author: Darp Raithatha
 */

#ifndef INC_STM32F44XX_I2C_DRIVER_H_
#define INC_STM32F44XX_I2C_DRIVER_H_

#include "stm32f446xx.h"


typedef struct{

	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t	 I2C_AckControl;
	uint16_t I2C_FMDutyCycle;

}I2C_Config_t;


typedef struct{

	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t *pTxBuffer; 		// To store Application Tx Buffer Address
	uint8_t *pRxBuffer;			// To store Application Rx Buffer Address
	uint32_t TxLen;				// To store Tx Message Length
	uint32_t RxLen;				// To store Rx Message Length
	uint8_t TxRxState;			// To Store Communication State Possible Value @I2C_STATE
	uint8_t DevAddr; 			// To store Slave Device Address
	uint32_t RxSize;			// To store Rx size
	uint8_t Sr;					// To store Repeated Start

}I2C_Handle_t;



/*
 * @I2C_SCLSpeed
 * Possible Value of The Serial Clock Speed (Slow Mode and Fast Mode)
 */

typedef enum{

	I2C_SCL_SPEED_SM = 100000, 				//100 KHz (Slow Mode)
	I2C_SCL_SPEED_FM2x = 200000,			//200 KHz (Fast Mode - All Values Above Slow Mode is Fast Mode)
	I2C_SCL_SPEED_FM4x = 400000,			//400 KHz (Fast Mode - Max Frequency supported by Device)

} I2C_SCL_SPEED_t;



/*
 * @I2C_ACK_Control
 * Possible Value of The Serial Clock Speed (Slow Mode and Fast Mode)
 */

typedef enum{
	I2C_ACK_DISABLE = 0, 				// Disable Acknowledgment
	I2C_ACK_ENABLE = 1,					// Enable Acknowledgment

} I2C_ACK_CONTROL_t;


/*
 * @I2C_FMDutyCycle
 * Possible Value of The Clock Duty Cycle ( Look at the Reference Manual for the Possible Values )
 */

typedef enum{

	I2C_FM_DUTY_2 = 0,
	I2C_FM_Duty_16_9 = 1,

} I2C_FM_DUTY_t;


/*
 * @I2C_SR
 * This is to Indicate Whether to Stop or to Repeated Start
 * (If you Enable Repeated Start The Stop Condition won't be Generated)
 */

typedef enum{

	I2C_SR_DISABLE = 0,
	I2C_SR_ENABLE = 1,

} I2C_SR_t;

/*
 * @I2C_STATE
 * This is to Indicate the I2C Peripheral State (Ready, Busy_Rx, Busy_Tx)
 */

typedef enum{

	I2C_ST_READY = 0,
	I2C_ST_BUSY_RX = 1,
	I2C_ST_BUSY_TX = 2,

} I2C_STATE_t;

/*
 * @I2C_EVENTS
 * Possible Value I2C Application Events
 *
 */
typedef enum{

	I2C_EVENT_TX_CMPLT = 0,			// Transmission is Complete
	I2C_EVENT_RX_CMPLT = 1,			// Reception is Complete
	I2C_EVENT_STOP_ERR = 2, 			// STOP Condition is Detected
	I2C_EVENT_BUS_ERR = 3,			// Bus Error
	I2C_EVENT_ARLO_ERR = 4,			// Arbitration Lost
	I2C_EVENT_AF_ERR = 5,			// Acknowledge Failure
	I2C_EVENT_OVR_ERR =	6,			// Overrun/Underrun Error
	I2C_EVENT_TIMEOUT =	7,			// Timeout
	I2C_EVENT_DATA_REQ = 8,			// TXE in Case of Slave Mode
	I2C_EVENT_DATA_RCV = 9,			// RXE in Case of Slave Mode

} I2C_EVENT_t;



/*
 * I2C Related Flag Definition
 */

#define I2C_FLAG_SB					( 1 << I2C_SR1_SB )
#define I2C_FLAG_ADDR				( 1 << I2C_SR1_ADDR )
#define I2C_FLAG_BTF				( 1 << I2C_SR1_BTF )
#define I2C_FLAG_ADD10				( 1 << I2C_SR1_ADD10 )
#define I2C_FLAG_STOPF				( 1 << I2C_SR1_STOPF )
#define I2C_FLAG_TXE				( 1 << I2C_SR1_TXE )
#define I2C_FLAG_RXNE				( 1 << I2C_SR1_RXNE )
#define I2C_FLAG_BERR				( 1 << I2C_SR1_BERR )
#define I2C_FLAG_ARLO				( 1 << I2C_SR1_ARLO )
#define I2C_FLAG_AF					( 1 << I2C_SR1_AF )
#define I2C_FLAG_OVR				( 1 << I2C_SR1_OVR )
#define I2C_FLAG_PECERR				( 1 << I2C_SR1_PECERR )
#define I2C_FLAG_TIMEOUT			( 1 << I2C_SR1_TIMEOUT )
#define I2C_FLAG_SMBALERT			( 1 << I2C_SR1_SMBALERT )



/*
 * Macros to Reset I2Cx Peripheral
 */

#define I2C1_REG_RESET()			do{ (RCC->APB1RSTR |= ( 1 << 21 )); (RCC->APB1RSTR &= ~( 1 << 21 )); }while(0)
#define I2C2_REG_RESET()			do{ (RCC->APB1RSTR |= ( 1 << 22 )); (RCC->APB1RSTR &= ~( 1 << 22 )); }while(0)
#define I2C3_REG_RESET()			do{ (RCC->APB1RSTR |= ( 1 << 23 )); (RCC->APB1RSTR &= ~( 1 << 23 )); }while(0)



/*********************************************************************************************
 * 						API's Supported by this Driver
 * 		For More Information about the API's check the Function Definitions
 ********************************************************************************************/

/*
 * Peripheral Clock Setup
 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);


/*
 * I2C Config and De-Config
 */

void I2C_Config(I2C_Handle_t *pI2CHandle);

void I2C_DeConfig(I2C_RegDef_t *pI2Cx);


/*
 * I2C Peripheral Control (Enable or Disable the I2C Peripheral)
 */

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);



/*
 * Flag Status Read
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);


/*
 * Data Send and Receive
 */

// Master Mode
// Blocking Call

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t SR);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t SR);

// Interrupt Based

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t SR);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t SR);

// Slave Mode
// Blocking Call
void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data); // Data is sent Byte by Byte So no need of Length
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C);

// Enables the Interrupt Registers ( For call Backs in case of Slave Events )
void I2C_SlaveCallBackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

//DMA Based


/*
 * IRQ Configuration and Handling
 */
void I2C_IRQConfig(uint32_t IRQNumber, uint32_t IRQPriority, uint8_t EnorDi);

//Interrupt Handler
void I2C_EV_IRQHandling (I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling (I2C_Handle_t *pI2CHandle);


/*
 * Other Peripheral Control API's
 */
void I2C_ManageAcking ( I2C_RegDef_t *pI2Cx, uint8_t EnorDi );

// Start and Stop Condition
void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

//In case of an Interrupt
void I2C_CloseReceiveData ( I2C_Handle_t *pI2CHandle );
void I2C_CloseSendData ( I2C_Handle_t *pI2CHandle );


/*
 * Application CallBack
 */

void I2C_AppEventCallBack(I2C_Handle_t *pI2CHandle, uint8_t Event);


#endif /* INC_STM32F44XX_I2C_DRIVER_H_ */
