/*
 * stm32f4xx_spi_driver.c
 *
 *  Created on: Apr 18, 2020
 *      Author: Darp Raithatha
 */

#include "stm32f44xx_spi_driver.h"

static void spi_txe_interrupt_handle( SPI_Handle_t *pSPIHandle );
static void spi_rxne_interrupt_handle( SPI_Handle_t *pSPIHandle );
static void spi_ovr_err_interrupt_handle( SPI_Handle_t *pSPIHandle );



/*
 * Peripheral Clock Setup
 */

/****************************************************************************************
 * @fn 				- SPI_PeriClockControl
 *
 * @ brief 			- This Function Enables or Disables Peripheral Clock for the Given SPI Peripheral
 *
 * @param[in]		- Base Address of SPI Peripheral
 * @param[in]		- ENABLE or DISABLE Macros
 * @param[in] 		-
 *
 * @return 			- none
 *
 * @Note			- none
 *
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if (EnorDi == ENABLE){
		if (pSPIx == SPI1){
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2){
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3){
			SPI3_PCLK_EN();
		}
		else if (pSPIx == SPI4){
			SPI4_PCLK_EN();
		}
	}
	else{
		if (pSPIx == SPI1){
				SPI1_PCLK_DI();
			}
			else if (pSPIx == SPI2){
				SPI2_PCLK_DI();
			}
			else if (pSPIx == SPI3){
				SPI3_PCLK_DI();
			}
			else if (pSPIx == SPI4){
				SPI4_PCLK_DI();
			}
	}
}


/*
 * SPI Init and De-Init
 */

/****************************************************************************************
 * @fn 				- SPI_Init
 *
 * @ brief 			- This Function Configures the SPI By setting up the SPI (Mode of Operation, Type of Bus, SPI_ClockSpeed, , DFF (Frame Format), CPOL, CPHA, SSM)
 *
 * @param[in]		- SPI_Handle_structure
 * @param[in]		-
 * @param[in] 		-
 *
 * @return 			- none
 *
 * @Note			- none
 *
 *
 */

void SPI_Config(SPI_Handle_t *pSPIHandle){

	uint32_t tempreg = 0;

	// Enable Clock for the SPI Peripheral
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//1. Configure the Device Mode
	tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. Configure the Bus Config
	if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD){

		// BIDI Mode Should be cleared
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE );

	}else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD){

		// BIDI Mode Should be set
		tempreg |= ( 1 << SPI_CR1_BIDIMODE );

	}else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX){

		// BIDI Mode Should be Cleared
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE );
		// RXOnly Bit Must Be Set
		tempreg |= ( 1 << SPI_CR1_RXONLY );
	}

	//3. Configure the SclkSpeed
	tempreg |= ( pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR );

	//4. Configure the DFF
	tempreg |= ( pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF );

	//5. Configure the CPOL
	tempreg |= ( pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL );

	//6. Configure the CPHA
	tempreg |= ( pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA );

	//7. Configure the SSM
	tempreg |= ( pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM );

	pSPIHandle->pSPIx->CR1 = tempreg;

}


/****************************************************************************************
 * @fn 				- SPI_DeConfig
 *
 * @ brief 			- This Function De-Initializes (Resets the Register) SPI (Set the Values of Registers to the Reset Value)
 *
 * @param[in]		- Base Address of SPI Peripheral
 * @param[in]		-
 * @param[in] 		-
 *
 * @return 			- none
 *
 * @Note			- none
 *
 */

void SPI_DeConfig(SPI_RegDef_t *pSPIx){

	if (pSPIx == SPI1){
		SPI1_REG_RESET();
	}
	else if (pSPIx == SPI2){
		SPI2_REG_RESET();
	}
	else if (pSPIx == SPI3){
		SPI3_REG_RESET();
	}
	else if (pSPIx == SPI4){
		SPI4_REG_RESET();
	}
}


/*
 * SPI Peripheral Control (Enable or Disable the SPI Peripheral)
 */

/****************************************************************************************
 * @fn 				- SPI_PeripheralControl
 *
 * @ brief 			- This Function Enables and Disables the SPI Peripheral
 *					-
 *
 * @param[in]		- Base Address of SPI Peripheral
 * @param[in]		- Enable or Disable
 * @param[in] 		-
 *
 * @return 			- none
 *
 * @Note			- The SPI Needs to Enabled before Sending/Receiving Data and To be Disabled Before Configuring
 *
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if (EnorDi == ENABLE){
		pSPIx->CR1 |= ( 1 << SPI_CR1_SPE );

	}else {
		pSPIx->CR1 &= ~( 1 << SPI_CR1_SPE );
	}
}


/*
 * SPI Peripheral Control (Enable or Disable the SPI Peripheral)
 */

/****************************************************************************************
 * @fn 				- SPI_SSIConfig
 *
 * @ brief 			- This Function Configures the SSI Register of the SPI Peripheral
 *					-
 *
 * @param[in]		- Base Address of SPI Peripheral
 * @param[in]		- Enable or Disable
 * @param[in] 		-
 *
 * @return 			- none
 *
 * @Note			- SSI needs to be Enabled when using SSM (This makes NSS Pin Internally High and prevents MODEF Error)
 *
 */

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if (EnorDi == ENABLE){
		pSPIx->CR1 |= ( 1 << SPI_CR1_SSI );

	}else {
		pSPIx->CR1 &= ~( 1 << SPI_CR1_SSI );
	}
}


/****************************************************************************************
 * @fn 				- SPI_SSOEConfig
 *
 * @ brief 			- This Function Configures the SSOE Register of the SPI Peripheral
 *					-
 *
 * @param[in]		- Base Address of SPI Peripheral
 * @param[in]		- Enable or Disable
 * @param[in] 		-
 *
 * @return 			- none
 *
 * @Note			- SSOE needs to be Enabled So NSS will be toggled along with the SPI_Enable
 *					  Therefore Slave will be selected
 */

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if (EnorDi == ENABLE){
		pSPIx->CR2 |= ( 1 << SPI_CR2_SSOE );

	}else {
		pSPIx->CR2 &= ~( 1 << SPI_CR2_SSOE );
	}
}

/*
 * Flag Status Read
 */
/****************************************************************************************
 * @fn 				- SPI_GetFlagStatus
 *
 * @ brief 			- This Function returns the Status of the given Flag by Reading the Status Register
 *
 * @param[in]		- Base Address of SPI Peripheral
 * @param[in]		- Flag Name
 * @param[in] 		-
 *
 * @return 			- FLAG_RESET (0) or FLAG_SET (1)
 *
 * @Note			- none
 *
 */

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){

	if (pSPIx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}




/*
 * Data Send and Receive
 */

/****************************************************************************************
 * @fn 				- SPI_SendData
 *
 * @ brief 			- This Function Sends the Data using SPI Peripheral (Blocking Based)
 *
 * @param[in]		- Base Address of SPI Peripheral
 * @param[in]		- Data to Transfer (Buffer)
 * @param[in] 		- Length/Size of Data
 *
 * @return 			- none
 *
 * @Note			- This is a Blocking Call
 *
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){

	while(Len > 0){
		// 1. Wait while TXE is SET
		while ( SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		// 2. Check the DFF Bit
		if ( pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ){
			// 16 Bit DFF
			// 1. Load the Data to DR
			pSPIx->DR = *( (uint16_t*)pTxBuffer );

			// 2. Decrement the Length
			Len--;
			Len--;

			// 3. Increment the Pointer
			(uint16_t*)pTxBuffer++;

		}
		else {
			// 8 Bit DFF
			// 1. Load the Data to DR
			pSPIx->DR = *pTxBuffer;

			// 2. Decrement the Length
			Len--;

			//3. Increment the Pointer
			(*pTxBuffer)++;
		}
	}
}


/****************************************************************************************
 * @fn 				- SPI_ReceiveData
 *
 * @ brief 			- This Function Receives the Data using SPI Peripheral (Blocking Based)
 *
 * @param[in]		- Base Address of SPI Peripheral
 * @param[in]		- Data to Transfer (Buffer)
 * @param[in] 		- Length/Size of Data
 *
 * @return 			- Received Data
 *
 * @Note			- none
 *
 */

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len){

	while(Len > 0){

		// 1. Wait while TXE is SET
		while ( SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		// 2. Load the Data from DR to RxBuffer
		if (pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ){
			*((uint16_t*) pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;

			(uint16_t*)pRxBuffer++;
		}
		else{
			*(pRxBuffer) = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

//Non-Blocking/Interrupt Based

/****************************************************************************************
 * @fn 				- SPI_SendData_IT
 *
 * @ brief 			- This Function Sends the Data using SPI Peripheral (Interrupt Based)
 *
 * @param[in]		- Base Address of SPI Peripheral
 * @param[in]		- Data to Transfer (Buffer)
 * @param[in] 		- Length/Size of Data
 *
 * @return 			- none
 *
 * @Note			- This is Interrupt Base So you have to Configure Interrupt (@IRQ_Config),
 * 					  also Handle the Interrupt (@IRQ_Handling) and
 * 					  also implement SPI_AppEventCallBack in the Application (Call back in case of Events such as overrun/CRC Error etc)
 *
 */


uint8_t SPI_SendData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len){

	uint8_t state = pSPIHandle->TxState;
	if ( state != SPI_ST_BUSY_TX){

		//1. Save the TxBuffer address and Len to the Handle Structure
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//2. Mark SPI in Transmission State (So that No other process can Take the Control of SPI Peripheral )
		pSPIHandle->TxState = SPI_ST_BUSY_TX;

		//3. Enable the TXEIE Control Bit to trigger the Interrupt whenever TXE Flag is set
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE );
	}

	return state;
}


/****************************************************************************************
 * @fn 				- SPI_ReceiveData_IT
 *
 * @ brief 			- This Function Receives the Data using SPI Peripheral (Interrupt Based)
 *
 * @param[in]		- Base Address of SPI Peripheral
 * @param[in]		- Data to Transfer (Buffer)
 * @param[in] 		- Length/Size of Data
 *
 * @return 			- none
 *
 * @Note			- This is Interrupt Base So you have to Configure Interrupt (@IRQ_Config),
 * 					  also Handle the Interrupt (@IRQ_Handling) and
 * 					  also implement SPI_AppEventCallBack in the Application (Call back in case of Events such as Tx-Complete/overrun/CRC Error etc)
 *
 */

uint8_t SPI_ReceiveData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len){

	uint8_t state = pSPIHandle->RxState;
	if ( state != SPI_ST_BUSY_RX){

		//1. Save the TxBuffer address and Len to the Handle Structure
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		//2. Mark SPI in Transmission State (So that No other process can Take the Control of SPI Peripheral )
		pSPIHandle->RxState = SPI_ST_BUSY_RX;

		//3. Enable the TXEIE Control Bit to trigger the Interrupt whenever TXE Flag is set
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE );
	}

	return state;

}


//DMA Based


/*
 * IRQ Configuration and Handling
 */

/****************************************************************************************
 * @fn 				- SPI_IRQConfig
 *
 * @ brief 			- This Function Enables or Disables the Interrupt for the Given SPI Peripheral and
 * 					  Also sets the Priority of the the Interrupt
 *
 * @param[in]		- IRQ Number (Interrupt Request Number - From Vector Table )
 * @param[in]		- IRQ Priority (0 to 16)
 * @param[in] 		- ENABLE or DISABLE Interrupt
 *
 * @return 			- none
 *
 * @Note			- none
 *
 */

void SPI_IRQConfig(uint32_t IRQNumber, uint32_t IRQPriority, uint8_t EnorDi)
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
 * @fn 				- SPI_IRQHandle
 *
 * @ brief 			- This Function Handles the Interrupt for the Given SPI Peripheral
 * 					  TxInterrupt, RxInterrupt and OverRun Interrupt
 * 					  (Basically Checks the Register for an Event - Which Event has occurred and calls the Function)
 *
 * @param[in]		- SPI_Handle of the Configured SPI Peripheral
 * @param[in]		-
 * @param[in] 		-
 *
 * @return 			- none
 *
 * @Note			- This Function Calls the Internal Functions such as:
 * 					  ( spi_txe_interrupt_handle, spi_rxne_interrupt_handle, spi_ovr_err_interrupt_handle )
 *
 */


void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){

	// Check the Reason for Interrupt
	uint8_t temp1, temp2;

	//1. Check for Tx
	temp1 = pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_TXE );
	temp2 = pSPIHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE );

	if (temp1 && temp2){
		//Handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}

	//2. Check for Rx
	temp1 = pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE );
	temp2 = pSPIHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE );

	if (temp1 && temp2){
		//Handle RXE
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	//3. Check for the Over-Run Error
	temp1 = pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_OVR );
	temp2 = pSPIHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

	if (temp1 && temp2){

		//Handle the Over Run Error
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}

}



/****************************************************************************************
 * @fn 				- SPI_ClearOVRFlag
 *
 * @ brief 			- This Function Clears the OverRun Flag for the Given SPI Peripheral (Interrupt Based)
 *
 * @param[in]		- Base Address of SPI Peripheral
 * @param[in]		-
 * @param[in] 		-
 *
 * @return 			- none
 *
 * @Note			- This API Should be called by the Application in the SPI_AppEventCallBack
 * 					  In case of OverFlag Event (To clear the OverRun Flag in the SR Register)
 *
 */

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){

	uint8_t temp;

	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}



/****************************************************************************************
 * @fn 				- SPI_CloseTransmission
 *
 * @ brief 			- This Function closes the Communication in case of Transmission Event
 * 					  (Transmission is Complete or any other Event)
 *
 * @param[in]		- SPI Handle of Configured  SPI Peripheral
 * @param[in]		-
 * @param[in] 		-
 *
 * @return 			- none
 *
 * @Note			- This API Should be called by the Application in the SPI_AppEventCallBack
 * 					  In case any Event occurs which needs to stop the Transmission
 *
 */

//Abruptly stopping the Transmission SPI_Handle of the Configured SPI Peripheral

void SPI_CloseTransmission (SPI_Handle_t *pSPIHandle){

	//1. Disable the TX Interrupt (Clear the TXEIE Bit- Disable the Interrupt)
	pSPIHandle->pSPIx->CR2 &= ~( 1<< SPI_CR2_TXEIE);

	//2. Reset all the Transmission Parameters in the Handle
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_ST_READY;

}


/****************************************************************************************
 * @fn 				- SPI_CloseReception
 *
 * @ brief 			- This Function closes the Communication in case of Reception Event
 * 					  (Transmission is Complete or any other Event)
 *
 * @param[in]		- SPI Handle of Configured  SPI Peripheral
 * @param[in]		-
 * @param[in] 		-
 *
 * @return 			- none
 *
 * @Note			- This API needs to be called by the Application in the SPI_AppEventCallBack
 * 					  In case any Event occurs which needs to stop the Reception
 *
 */

void SPI_CloseReception (SPI_Handle_t *pSPIHandle){

	//Disable the RX Interrupt (Clear the RXNE
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);

	//Clear the SPIHandle TxParameters
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_ST_READY;

}

/****************************************************************************************
 * @fn 				- SPI_AppEventCallBack
 *
 * @ brief 			- This Function Call back Function in case of any Event During the Communication
 * 					  (Transmission/Reception is Complete or OverRun Error any other Event)
 *
 * @param[in]		- SPI Handle of Configured  SPI Peripheral
 * @param[in]		-
 * @param[in] 		-
 *
 * @return 			- none
 *
 * @Note			- This Function is the week implementation of SPI_AppEventCallBack
 * 					  Actually this Function needs to be implemented in the Application
 * 					  To Handle the Events
 *
 */


__weak void SPI_AppEventCallBack(SPI_Handle_t *pSPIHandle, uint8_t Event){

}





//Some Static Function Implementation
/* Function to Handle the TXE Interrupt (DR is Empty)
 * Loads the DR with the next value to Transmit from TxBuffer
 *
 * Note- In case the the Event occurs and the TXLen is Zero it Closes the Transmission
 * 		 And Calls the Application with TX_COMPLT Event (Transmission Complete)
 *
 */


static void spi_txe_interrupt_handle( SPI_Handle_t *pSPIHandle ){

	if ( pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ){
		// 16 Bit DFF
		// 1. Load the Data to DR
		pSPIHandle->pSPIx->DR = *( (uint16_t*)pSPIHandle->pTxBuffer );

		// 2. Decrement the Length
		pSPIHandle->TxLen-= 2;

		// 3. Increment the Pointer
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}
	else {
		// 8 Bit DFF
		// 1. Load the Data to DR
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);

		// 2. Decrement the Length
		pSPIHandle->TxLen--;

		//3. Increment the Pointer
		 pSPIHandle->pTxBuffer++;
	}

	if (!pSPIHandle->TxLen){
		//Close the SPI Transmission
		SPI_CloseTransmission (pSPIHandle);
		//3. CallBack the Application ( This Function need to be implemented in Application )
		SPI_AppEventCallBack(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}

}


/* Function to Handle the RXNE Interrupt (DR is Full)
 * Reads the DR Register Saves it to the RxBuffer
 *
 * Note- In case the the Event occurs and the RXLen is Zero it Closes the Reception
 * 		 And Calls the Application with RX_COMPLT Event (Reception Complete)
 */

static void spi_rxne_interrupt_handle( SPI_Handle_t *pSPIHandle ){

	if (pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ){
		*((uint16_t*) pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;

		(uint16_t*) pSPIHandle->pRxBuffer++;
	}
	else{
		*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if (!pSPIHandle->RxLen){
		// Close the Reception
		SPI_CloseReception (pSPIHandle);
		//Call Back the Application
		SPI_AppEventCallBack(pSPIHandle, SPI_EVENT_RX_CMPT);
	}
}


// Function to Handle the Over Error Interrupt
/*
 * IT clears the OverRun Flag and calls the Application with an OverRun Event
 *
 */
static void spi_ovr_err_interrupt_handle( SPI_Handle_t *pSPIHandle ){

	uint8_t temp;
	//1. Clear the OverRun Flag
	if (pSPIHandle->TxState != SPI_ST_BUSY_TX){

		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void) temp;

	//2. Callback the Application
	SPI_AppEventCallBack(pSPIHandle, SPI_EVENT_OVR_ERR);
}

