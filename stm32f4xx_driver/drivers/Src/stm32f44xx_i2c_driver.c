/*
 * stm32f44xx_i2c_driver.c
 *
 *  Created on: Apr 29, 2020
 *      Author: Darp Raithatha
 */

#include "stm32f44xx_i2c_driver.h"
#include "stm32f44xx_rcc_driver.h"


static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);



/*
 * This is a Function to Generate a Start Condition
 * (Keeping the Clock High and Pulling the Data line Low)
 */

void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){

	pI2Cx->CR1 |= ( 1 << I2C_CR1_START );

}


/*
 * This is a Function to Generate a Stop Condition
 * (Keeping the Clock High and Pulling the Data line High)
 */

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){

	pI2Cx->CR1 |= ( 1 << I2C_CR1_STOP );

}

/*
 * This is a Private Function to Clear the ADDR Flag
 * ( Once the Address Phase has been Completed and Address is Matched ADDR Flag will be set )
 */

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle){

	uint32_t dummy_read;

	// Check the Device Mode
	if ( pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL ) ){
		//The Device is in  Master Mode

		// In Busy in Rx
		if ( pI2CHandle->TxRxState == I2C_ST_BUSY_RX ){

			if ( pI2CHandle->RxSize == 1 ){

				// First Disable the ACK
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				// Clear the ADDR Flag (Read SR1 and SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}
		}
		// If Busy in Tx
		else{
			// Clear the ADDR Flag (Read SR1 and SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;

		}
	}

	// If Device is in Slave Mode
	else{
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}
}


/*
 * This is a Function to Close the Communication in case of Interrupt for Receiving the Data
 */
void I2C_CloseReceiveData ( I2C_Handle_t *pI2CHandle ){

	//Disable the ITBUFEN
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN );

	//Disable the ITEVFEN
	pI2CHandle->pI2Cx->CR2 &= ~( 1<< I2C_CR2_ITEVTEN );

	// Reset all the Parameters in the Handler
	pI2CHandle->TxRxState = I2C_ST_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;
	if ( pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE ){
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}

/*
 * This is a Private Function to Close the Communication in case of Interrupt for Sending the Data
 */

void I2C_CloseSendData ( I2C_Handle_t *pI2CHandle ){

	//Disable the ITBUFEN
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN );

	//Disable the ITEVFEN
	pI2CHandle->pI2Cx->CR2 &= ~( 1<< I2C_CR2_ITEVTEN );

	// Reset all the Parameters in the Handler
	pI2CHandle->TxRxState = I2C_ST_READY;
	pI2CHandle->pTxBuffer = NULL;
}


/*
 * This is a Function to Clear the ACK Bit
 * ( We need the Disable Automated Acknowledgment once the data Required is Received )
 * So Slave will Get NACK and Won't send additional Data
 */

/****************************************************************************************
 * @fn 				- I2C_ManageAcking
 *
 * @ brief 			- This Function Enables/Disables Automatic Acking
 *
 * @param[in]		- The Base Address of the SPI Peripheral
 * @param[in]		- Enable or Disable
 * @param[in] 		-
 *
 * @return 			- none
 *
 * @Note			-
 *
 */

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){

	if ( EnorDi == I2C_ACK_ENABLE ){
		//Enable the ACK
		pI2Cx->CR1 |= ( 1 << I2C_CR1_ACK );
	}
	else{
		//Disable the ACK
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK );
	}

}


/*
 * Peripheral Clock Setup
 */

/****************************************************************************************
 * @fn 				- I2C_PeriClockControl
 *
 * @ brief 			- This Function Enables or Disables Peripheral Clock for the Given I2C Peripheral
 *
 * @param[in]		- Base Address of I2C Peripheral
 * @param[in]		- ENABLE or DISABLE Macros
 * @param[in] 		-
 *
 * @return 			- none
 *
 * @Note			- none
 *
 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if (EnorDi == ENABLE){
		if (pI2Cx == I2C1){
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == I2C2){
			I2C2_PCLK_EN();
		}
		else if (pI2Cx == I2C3){
			I2C3_PCLK_EN();
		}

	}
	else{
		if (pI2Cx == I2C1){
				I2C1_PCLK_DI();
			}
			else if (pI2Cx == I2C2){
				I2C2_PCLK_DI();
			}
			else if (pI2Cx == I2C3){
				I2C3_PCLK_DI();
			}
	}
}


/****************************************************************************************
 * @fn 				- I2C_Config
 *
 * @ brief 			- This Function Initializes (Resets the Register) I2C (Set the Values of Registers to the Reset Value)
 *
 * @param[in]		- Base Address of I2C Peripheral
 * @param[in]		-
 * @param[in] 		-
 *
 * @return 			- none
 *
 * @Note			- none
 *
 */

void I2C_Config(I2C_Handle_t *pI2CHandle){

	uint32_t tempreg = 0;

	// Enable Clock for the I2C Peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	//1. Configure the Acking (CR1)
	tempreg |= (pI2CHandle->I2C_Config.I2C_AckControl << 10);
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//2. Configure the Freq Field in CR2 Register ( For Serial Clock - Equal to Peripheral Clock )
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value()/1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//3. Set the Device Own Address ( In Case of Slave Mode )
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= ( 1 << 14 ); // As per the Reference Manual the 14th Bit should always be kept 1
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//4. Set the CCR Register ( For Serial Clock Settings )
	uint16_t ccr_value = 0;
	tempreg = 0;
	if ( pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM ){
		//Mode is Standard
		//In Case of Standard Mode the 15th Bit (F/S) needs to be set to 0
		// CCR Value is Calculated as per the Data Sheet
		ccr_value = ( RCC_GetPCLK1Value() /( 2 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		tempreg |= ( ccr_value & 0xFFF );
	}
	else{
		//Fast Mode
		//In Case of Standard Mode the 15th Bit (F/S) needs to be set to 0
		tempreg |= ( 1<< 15 );
		tempreg |= ( pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14 );
		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2 ){
			ccr_value = ( RCC_GetPCLK1Value() /( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );

		}
		else{
			ccr_value = ( RCC_GetPCLK1Value() /( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		}

		tempreg |= ( ccr_value & 0xFFF );

	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//5. Setting up the TRISE Register
	tempreg = 0;
	if ( pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM ){
		//Mode is Standard
		//Note: Here the Value 1000000U is calculated from Max Rise Time in the Standard mode (From I2C Specs)
		// Here Tmax = 1000ns = 1MHz
		tempreg = ( ( RCC_GetPCLK1Value() / 1000000U ) + 1 );
	}
	else{
		//Mode if Fast Mode
		//Here the Tmax is 300ns
		tempreg = ( ( ( RCC_GetPCLK1Value() * 300 ) / 1000000000U ) + 1 );
	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}



/****************************************************************************************
 * @fn 				- I2C_DeConfig
 *
 * @ brief 			- This Function De-Initializes (Resets the Register) I2C (Set the Values of Registers to the Reset Value)
 *
 * @param[in]		- Base Address of I2C Peripheral
 * @param[in]		-
 * @param[in] 		-
 *
 * @return 			- none
 *
 * @Note			- none
 *
 */

void I2C_DeConfig(I2C_RegDef_t *pI2Cx){

	if (pI2Cx == I2C1){
		I2C1_REG_RESET();
	}
	else if (pI2Cx == I2C2){
		I2C2_REG_RESET();
	}
	else if (pI2Cx == I2C3){
		I2C3_REG_RESET();
	}
}


/*
 * I2C Peripheral Control (Enable or Disable the I2C Peripheral)
 */

/****************************************************************************************
 * @fn 				- I2C_PeripheralControl
 *
 * @ brief 			- This Function Enables and Disables the I2C Peripheral
 *					-
 *
 * @param[in]		- Base Address of I2C Peripheral
 * @param[in]		- Enable or Disable
 * @param[in] 		-
 *
 * @return 			- none
 *
 * @Note			- The I2C Needs to Enabled before Sending/Receiving Data and To be Disabled Before Configuring
 *
 */

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if (EnorDi == ENABLE){
		pI2Cx->CR1 |= ( 1 << I2C_CR1_PE );

	}else {
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_PE );
	}
}

/*
 * Flag Status Read
 */
/****************************************************************************************
 * @fn 				- I2C_GetFlagStatus
 *
 * @ brief 			- This Function returns the Status of the given Flag by Reading the Status Register
 *
 * @param[in]		- Base Address of I2C Peripheral
 * @param[in]		- Flag Name
 * @param[in] 		-
 *
 * @return 			- FLAG_RESET (0) or FLAG_SET (1)
 *
 * @Note			- none
 *
 */

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName){

	if (pI2Cx->SR1 & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}



/*
 * I2C Data Send and Receive API's
 */

/****************************************************************************************
 * @fn 				- I2C_MasterSendData
 *
 * @ brief 			- This Function Sends the Data using I2C Peripheral (Blocking Based)
 *
 * @param[in]		- Base Address of I2C Peripheral
 * @param[in]		- Data to Transfer (Buffer)
 * @param[in] 		- Length/Size of Data
 * @param[in]		- The Salve Address
 *
 * @return 			- none
 *
 * @Note			- This is a Blocking Call
 *
 */


void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t SR){
	//1. Generate a Start Condition
	I2C_GenerateStartCondition ( pI2CHandle->pI2Cx ) ;

	//2. Confirm that the Start Generation is Implemented by Checking the SB Flag in SR1
	//Note: Until SB is cleared SCL will be Stretched (Pulled to Low)
	while ( ! I2C_GetFlagStatus( pI2CHandle->pI2Cx, I2C_FLAG_SB ) );

	//3. Load the DR Register with the Slave Address with the Re(1)/Wr(0) Bit Set to 0
	SlaveAddr = SlaveAddr << 1; // Make space for Read/Write Bit
	SlaveAddr &= ~(1); // Clear the 0th Bit
	pI2CHandle->pI2Cx->DR |= (SlaveAddr); // Load the DR with Slave Address and Read/Write Bit

	//4. Confirm that the Address Phase is Complete (i.e By Reading the ADDR Flag in SR1)
	while( ! I2C_GetFlagStatus( pI2CHandle->pI2Cx, I2C_FLAG_ADDR ) );

	//5. Clear the ADDR Flag
	// Note: Until ADDR is Cleared the Clock will be Stretched (Pulled to Low)
	I2C_ClearADDRFlag(pI2CHandle);

	//6. Send the Data until the Len becomes 0
	while( Len>0 ){

		while( ! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE ) ) ); // Wait till TXE Flag is SET
		pI2CHandle->pI2Cx->DR |= *pTxbuffer;
		pTxbuffer++;
		Len--;

	}

	//7. When the Len becomes 0 Wait for TXE=1 and BTF=1 Before Generating Stop Condition
	//Note: TXE=1 and BTF=1, means that Both Shift Register and DR are Empty and Next Transmission should Begin
	//while BTF=1 the Clock will be Stretched
	while( ! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE ) ) ); // Wait till TXE is 1

	while( ! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF ) ) )	; // Wait till BTF is 1


	//8. Generate the Stop Condition (Master needs to Wait for the Stop Condition to Execute)
	//Note: Generating STOP Automatically clears the BTF
	if (SR == I2C_SR_DISABLE) // Only if Repeated Start is Disabled
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

}



/****************************************************************************************
 * @fn 				- I2C_MasterReceiveData
 *
 * @ brief 			- This Function Sends the Data using I2C Peripheral (Blocking Based)
 *
 * @param[in]		- Base Address of I2C Peripheral
 * @param[in]		- Data to Transfer (Buffer)
 * @param[in] 		- Length/Size of Data
 * @param[in]		- The Salve Address
 *
 * @return 			- none
 *
 * @Note			- This is a Blocking Call
 *
 */


void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t SR){

	//1. Generate a Start Condition
	I2C_GenerateStartCondition ( pI2CHandle->pI2Cx ) ;

	//2. Confirm that the Start Generation is Implemented by Checking the SB Flag in SR1
	//Note: Until SB is is cleared SCL will be Stretched (Pulled to Low)
	while ( ! I2C_GetFlagStatus( pI2CHandle->pI2Cx, I2C_FLAG_SB ) );

	//3. Load the DR Register with the Slave Address with the Re(1)/Wr(0) Bit Set to 0
	SlaveAddr = SlaveAddr << 1; // Make space for Read/Write Bit
	SlaveAddr |= 1; // Set the 0th Bit
	pI2CHandle->pI2Cx->DR |= (SlaveAddr); // Load the DR with Slave Address and Read/Write Bit

	//4. Confirm that the Address Phase is Complete (i.e By Reading the ADDR Flag in SR1)
	while( ! I2C_GetFlagStatus( pI2CHandle->pI2Cx, I2C_FLAG_ADDR ) );


	if (Len == 1){

		// Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		//Clear the ADDR Flag
		I2C_ClearADDRFlag(pI2CHandle);

		//Wait until RXNE becomes 1
		while( ! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE ) ) );

		//Generate STOP Condition
		if (SR == I2C_SR_DISABLE ) // Only if Repeated Start is Disabled
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//Read the Data From DR to Buffer
		*pRxbuffer = pI2CHandle->pI2Cx->DR;

	}

	if ( Len > 1 ){

		//Clear the ADDR Flag
		// Note: Until ADDR is Cleared the Clock will be Stretched (Pulled to Low)
		I2C_ClearADDRFlag(pI2CHandle);

		// Read the Data Until the Len becomes 0
		for (uint32_t i = Len; i>0; i++ ){

			//Wait until RXNE becomes 1
			while( ! ( I2C_GetFlagStatus(pI2CHandle->pI2Cx , I2C_FLAG_RXNE ) ) );

			if (i == 2){
				//Disable Acking
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				//Generate the Stop Condition
				if (SR == I2C_SR_DISABLE ) // Only if Repeated Start is Disabled
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			// Read the Received Data and Save it to RxBuffer
			*pRxbuffer = pI2CHandle->pI2Cx->DR;
			pRxbuffer++;
		}
	}
	//Enable Acking
	if ( pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE ){
		I2C_ManageAcking( pI2CHandle->pI2Cx, I2C_ACK_ENABLE );
	}
}


/*********************************************************************
 * @fn      		  - I2C_MasterSendDataIT
 *
 * @brief             -
 *
 * @param[in]         - I2C Handle Structure
 * @param[in]		  - Data to Transfer (Buffer)
 * @param[in] 		  - Length/Size of Data
 * @param[in]		  - The Salve Address
 *
 * @return            -
 *
 * @Note              -
 */

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t SR){

	uint8_t busystate = pI2CHandle->TxRxState;

		if( (busystate != I2C_ST_BUSY_TX) && (busystate != I2C_ST_BUSY_RX))
		{
			pI2CHandle->pTxBuffer = pTxbuffer;
			pI2CHandle->TxLen = Len;
			pI2CHandle->TxRxState = I2C_ST_BUSY_TX;
			pI2CHandle->DevAddr = SlaveAddr;
			pI2CHandle->Sr = SR;

			//Implement code to Generate START Condition
			I2C_GenerateStartCondition ( pI2CHandle->pI2Cx );

			//Implement the code to enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN );

			//Implement the code to enable ITEVFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN );

			//Implement the code to enable ITERREN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN );
		}
		return busystate;
}

/*********************************************************************
 * @fn      		  - I2C_MasterReceiveDataIT
 *
 * @brief             -
 *
 * @param[in]         - I2C Handle Structure
 * @param[in]		  - Address to Store the Received Data (Buffer)
 * @param[in] 		  - Length/Size of Data
 * @param[in]		  - The Salve Address
 *
 * @return            -
 *
 * @Note              - Complete the below code . Also include the fn prototype in header file

 */

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t SR){

	uint8_t busystate = pI2CHandle->TxRxState;

		if( (busystate != I2C_ST_BUSY_TX) && (busystate != I2C_ST_BUSY_RX))
		{
			pI2CHandle->pRxBuffer = pRxbuffer;
			pI2CHandle->RxLen = Len;
			pI2CHandle->TxRxState = I2C_ST_BUSY_RX;
			pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
			pI2CHandle->DevAddr = SlaveAddr;
			pI2CHandle->Sr = SR;

			//Implement code to Generate START Condition
			I2C_GenerateStartCondition ( pI2CHandle->pI2Cx );


			//Implement the code to enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN );


			//Implement the code to enable ITEVFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN );


			//Implement the code to enable ITERREN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN );
		}

		return busystate;
}


//Slave Mode API's

void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data){
	// Data is sent Byte by Byte So no need of Length
	pI2C->DR = data;

}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C){

	return (uint8_t) pI2C->DR;

}


void I2C_SlaveCallBackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if (EnorDi == ENABLE){
		// Enable ITBUFEN Control Bit
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN );
		// Enable ITEVFEN Control Bit
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN );
		// Enable ITERREN Control Bit
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN );
	}
	else{
		// Enable ITBUFEN Control Bit
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN );
		// Enable ITEVFEN Control Bit
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN );
		// Enable ITERREN Control Bit
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITERREN );
	}
}



/****************************************************************************************
 * @fn 				- I2C_IRQConfig
 *
 * @ brief 			- This Function Enables or Disables the Interrupt for the Given I2C Peripheral and
 * 					  Also sets the Priority of the the Interrupt
 *
 * @param[in]		- IRQ Number (Interrupt Request Number - From Vector Table )
 * @param[in]		- IRQ Priority (0 to 16)
 * @param[in] 		- ENABLE or DISABLE Interrupt
 *
 * @return 			- none
 *
 * @Note			- Priority will only be Configured in case of Enable
 *
 */

void I2C_IRQConfig(uint32_t IRQNumber, uint32_t IRQPriority, uint8_t EnorDi)
{
	if (EnorDi == ENABLE){

		if (IRQNumber <= 31){
			//Program ISER0 Register
			*NVIC_ISER0 |= ( 1 << IRQNumber);
		}

		else if (IRQNumber > 31 && IRQNumber < 64 ){
			//Program ISER1 Register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}

		else if (IRQNumber >=64 && IRQNumber < 96){
			//Program ISER2 Register
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}

		//Setting up the Priority

		uint8_t iprx = IRQNumber / 4;
		uint8_t iprx_section = IRQNumber % 4;
		uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED);

		*(NVIC_PR_BASE_ADDR + (iprx)) &= ~(0xF << shift_amount);
		*(NVIC_PR_BASE_ADDR + (iprx)) |= (IRQPriority << shift_amount);
	}

	else{
		if (IRQNumber <= 31){
			//Program ICER0 Register
			*NVIC_ICER0 |= ( 1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64 ){
			//Program ICER1 Register
			*NVIC_ICER1 |= ( 1<< (IRQNumber % 32) );
		}
		else if (IRQNumber >=64 && IRQNumber < 96){
			//Program ICER2 Register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}
	}
}



/****************************************************************************************
 * @fn 				- I2C_EV_IRQHandling
 *
 * @ brief 			- This Function Handles the I2C Events Whenever Triggered
 *
 * @param[in]		- I2C Handle Structure
 * @param[in]		-
 * @param[in] 		-
 *
 * @return 			- none
 *
 * @Note			-
 *
 */

void I2C_EV_IRQHandling (I2C_Handle_t *pI2CHandle){

	// Interrupt Handling for Both Master and Slave Mode of Device
	uint32_t temp1, temp2, temp3;
	uint8_t SlaveAddr = pI2CHandle->DevAddr;


	temp1 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITBUFEN);


	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_SB);
	// 1. Handle for Interrupt Generated by SB Event
	// Note: SB Flag is Only Applicable for Master Mode
	if ( temp1 && temp2 ){
		// SB Flag is Set ( Start Condition has Occurred )
		// This Block will only Execute in Master Mode because for Slave SB is Always 0
		// Once the Start Condition has occurred we need to Execute the Address Phase
		if ( pI2CHandle->TxRxState == I2C_ST_BUSY_TX ){

			SlaveAddr = SlaveAddr << 1; // Make space for Read/Write Bit
			SlaveAddr &= ~(1); // Clear the 0th Bit
			pI2CHandle->pI2Cx->DR |= SlaveAddr; // Load the DR with Slave Address and Read/Write Bit
		}
		else if ( pI2CHandle->TxRxState == I2C_ST_BUSY_RX ){
			SlaveAddr = SlaveAddr << 1; // Make space for Read/Write Bit
			SlaveAddr |= 1; // Set the 0th Bit
			pI2CHandle->pI2Cx->DR |= SlaveAddr; // Load the DR with Slave Address and Read/Write Bit
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_ADDR );
	// 2. Handle the Interrupt Triggered by ADDR Flag
	// Note: When in Master Mode: Address is Sent
	//		 When in Slave Mode : Address Matched with Own Address
	if (temp1 && temp3){
		// ADDR Flag is Set
		// Clear the ADDR Flag

		I2C_ClearADDRFlag(pI2CHandle);

	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_BTF );
	// 3. Handle the Interrupt Triggered by BTF ( Byte Transfer Finished ) Event
	if (temp1 && temp3){
		// BTF Flag is Set
		// If the Sate is Busy in Tx
		if (pI2CHandle->TxLen == 0){
			if (pI2CHandle->TxRxState == I2C_ST_BUSY_TX){

				// If TXE is Set
				if ( pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE ) ){

					//TXE and BTF is Set
					//1. Generate the STOP Condition (If Repeated Start is OFF)
					if ( pI2CHandle->Sr == I2C_SR_DISABLE ){
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

					// Reset all the Member Elements of the Handle Structure
					I2C_CloseSendData(pI2CHandle);

					// Notify the Application about the Transmission is Complete
					I2C_AppEventCallBack(pI2CHandle, I2C_EVENT_TX_CMPLT);

					}
				}
			}
		}
	}


	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_STOPF );
	// 4. Handle the Interrupt Triggered by STOPF Event
	// Note: Stop Detection is Only applicable for Slave Mode. For Master this Flag will Never be Set
	// STOP Flag is Set by the Hardware when the Stop Condition is Detected
	if (temp1 && temp3){

		// STOPF Flag is Set
		//Clear the STOP Flag (i.e Read SR1 (Done Above) and Write to CR1)
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		// Notify the Application
		I2C_AppEventCallBack(pI2CHandle, I2C_EVENT_STOP_ERR);

	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE );
	// 5. Handle the Interrupt Triggered by TXE Event
	if (temp1 && temp2 && temp3){
		// IF Device is in Master Mode
		if (pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL )){
			// TXE Flag is Set

			// If Device is Busy in Tx
			if ( pI2CHandle->TxRxState == I2C_ST_BUSY_TX ){
				if ( pI2CHandle->TxLen > 0 ){
					//1. Load the Data to DR
					pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
					//2. Decrement the TxLen
					pI2CHandle->TxLen--;
					//Increment the Buffer Address
					pI2CHandle->pTxBuffer++;
				}
			}
		}
		// Device is in Slave Mode
		else{
			// If the Device is on Transmission Mode
			// Note: TRA is Being set according to the Read/Write Bit Received from the Master
			if (pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA ) ){
				I2C_AppEventCallBack(pI2CHandle, I2C_EVENT_DATA_REQ);
			}
		}
	}



	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_RXNE );
	// 6. Handle the Interrupt Triggered by RXNE Event
	if ( temp1 && temp2 && temp3 ){

		// If Device is in Master Mode
		if (pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL )){
			// RXNE Flag is Set
			if ( pI2CHandle->TxRxState == I2C_ST_BUSY_RX ){

				if (pI2CHandle->RxSize == 1){

					// Save the Data to RxBuffer
					*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
					// Decrement the RxLen
					pI2CHandle->RxLen--;

				}

				else if (pI2CHandle->RxSize > 1){

					if (pI2CHandle->RxLen == 2){

						//Disable Acking
						I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
					}

					//Read the DR
					// Save the Data to RxBuffer
					*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
					// Increment the RX Buffer
					pI2CHandle->pRxBuffer++;
					// Decrement the RxLen
					pI2CHandle->RxLen--;
				}

				if ( pI2CHandle->RxLen == 0 ){
					//Close the Communication
					//1. Generate the Stop Condition
					if (pI2CHandle->Sr == I2C_SR_DISABLE){
						//Generate the STOP Condition
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					//2. Close the I2C RX
					I2C_CloseReceiveData(pI2CHandle);

					//3. Notify the Application
					I2C_AppEventCallBack(pI2CHandle, I2C_EVENT_RX_CMPLT);
				}
			}
		}

		// If Device is in Slave Mode
		else{
			// If the Device is on Reception Mode
			// Note: TRA is Being set according to the Read/Write Bit Received from the Master
			if (!(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA ) ) ){
				I2C_AppEventCallBack(pI2CHandle, I2C_EVENT_DATA_RCV);
			}

		}
	}
}




/****************************************************************************************
 * @fn 				- I2C_EV_IRQHandling
 *
 * @ brief 			- This Function Handles the I2C Error Whenever Triggered
 *
 * @param[in]		- I2C Handle Structure
 * @param[in]		-
 * @param[in] 		-
 *
 * @return 			- none
 *
 * @Note			-
 *
 */

void I2C_ER_IRQHandling (I2C_Handle_t *pI2CHandle){

	uint32_t temp1,temp2;

	//Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		// Bus error
		// Clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		// Implement the code to notify the application about the error
	   I2C_AppEventCallBack(pI2CHandle, I2C_EVENT_BUS_ERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		// Arbitration lost error
		// Clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);
		// Notify the application about the error
		I2C_AppEventCallBack(pI2CHandle, I2C_EVENT_ARLO_ERR);

	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		// ACK failure error
		// Clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);
		// Notify the application about the error
		I2C_AppEventCallBack(pI2CHandle, I2C_EVENT_AF_ERR);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		// This is Overrun/underrun
		// Clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);
		// Notify the application about the error
		I2C_AppEventCallBack(pI2CHandle, I2C_EVENT_OVR_ERR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		// Time out error
		// Clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);
		// Notify the application about the error
		I2C_AppEventCallBack(pI2CHandle, I2C_EVENT_TIMEOUT);
	}


}


/****************************************************************************************
 * @fn 				- I2C_AppEventCallBack
 *
 * @ brief 			- This Function Call back Function in case of any Event During the Communication
 * 					  (Transmission/Reception is Complete or OverRun Error any other Event)
 *
 * @param[in]		- I2C Handle of Configured I2C Peripheral
 * @param[in]		-
 * @param[in] 		-
 *
 * @return 			- none
 *
 * @Note			- This Function is the week implementation of I2C_AppEventCallBack
 * 					  Actually this Function needs to be implemented in the Application
 * 					  To Handle the Events
 *
 */


__weak void I2C_AppEventCallBack(I2C_Handle_t *pI2CHandle, uint8_t Event){

}



