/*
 * stm32f44xx_usart_driver.c
 *
 *  Created on: May 23, 2020
 *      Author: Darp Raithatha
 */

#include "stm32f44xx_usart_driver.h"
#include "stm32f44xx_rcc_driver.h"


/*
 * Peripheral Clock Setup
 */

/****************************************************************************************
 * @fn 				- USART_PeriClockControl
 *
 * @ brief 			- This Function Enables or Disables Peripheral Clock for the Given USART Peripheral
 *
 * @param[in]		- Base Address of USART Peripheral
 * @param[in]		- ENABLE or DISABLE Macros
 * @param[in] 		-
 *
 * @return 			- none
 *
 * @Note			- none
 *
 */


void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi){
	if (EnorDi == ENABLE){
		if (pUSARTx == USART1){
			USART1_PCLK_EN();
		}
		else if (pUSARTx == USART2){
			USART2_PCLK_EN();
		}
		else if (pUSARTx == USART3){
			USART3_PCLK_EN();
		}
		else if (pUSARTx == UART4){
			UART4_PCLK_EN();
		}
		else if (pUSARTx == UART5){
			UART5_PCLK_EN();
		}
		else if (pUSARTx == USART6){
			USART6_PCLK_EN();
		}
	}
	else{
		if (pUSARTx == USART1){
			USART1_PCLK_DI();
		}
		else if (pUSARTx == USART2){
			USART2_PCLK_DI();
		}
		else if (pUSARTx == USART3){
			USART3_PCLK_DI();
		}
		else if (pUSARTx == UART4){
			UART4_PCLK_DI();
		}
		else if (pUSARTx == UART5){
			UART5_PCLK_DI();
		}
		else if (pUSARTx == USART6){
			USART6_PCLK_DI();
		}
	}
}


/*********************************************************************
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  Learning outcome (Rounding and dealing with Fractional Calculation without using Floating numbers)

 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part, F_part;

	uint32_t tempreg=0;
	//Get the value of APB bus clock in to the variable PCLKx
	if(pUSARTx == USART1 || pUSARTx == USART6){
		//USART1 and USART6 are hanging on APB2 bus
		PCLKx = RCC_GetPCLK2Value();
	}
	else{
		PCLKx = RCC_GetPCLK1Value();
	}

	//Check for OVER8 configuration bit
	if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8 ) ){
		//OVER8 = 1 , over sampling by 8 (USARTDIV = (100*FCLK)/(8*BaudRate)
		usartdiv = ((25 * PCLKx) / (2 * BaudRate));
	}
	else{
		//over sampling by 16 (USARTDIV = (100*FCLK)/(16*BaudRate)
		usartdiv = ((25 * PCLKx) / (4 * BaudRate));
	}

	//Calculate the Mantissa part
	M_part = usartdiv/100;

	//Place the Mantissa part in appropriate bit position . refer USART_BRR
	tempreg |= M_part << 4;

	//Extract the fraction part
	F_part = (usartdiv - (M_part * 100));

	//Calculate the final fractional
	if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8)){
		//OVER8 = 1 , over sampling by 8 (Rounding Concept)
		F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);
	}
	else{
		//over sampling by 16
		F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);
	}

	//Place the fractional part in appropriate bit position . refer USART_BRR
	tempreg |= F_part;

	//copy the value of tempreg in to BRR register
	pUSARTx->BRR = tempreg;
}


/****************************************************************************************
 * @fn 				- USART_Config
 *
 * @ brief 			- This Function Initializes (Resets the Register) USART (Set the Values of Registers to the Reset Value)
 *
 * @param[in]		- Base Address of USART Peripheral
 * @param[in]		-
 * @param[in] 		-
 *
 * @return 			- none
 *
 * @Note			- none
 *
 */

void USART_Config(USART_Handle_t *pUSARTHandle){

	//Temporary variable
	uint32_t tempreg=0;

/******************************** Configuration of CR1******************************************/

	// Enable the Clock for given USART peripheral
	 USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	// Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_RX)
	{
		// Enable the Receiver bit field
		tempreg|= ( 1 << USART_CR1_RE );
	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TX)
	{
		// Enable the Transmitter bit field
		tempreg |= ( 1 << USART_CR1_TE );

	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		// Enable the both Transmitter and Receiver bit fields
		tempreg |= ( ( 1 << USART_CR1_RE ) | ( 1 << USART_CR1_TE ) );
	}

    // Configure the Word length configuration item
	tempreg |= pUSARTHandle->USART_Config.USART_wordLength << USART_CR1_M ;


    //Configuration of parity control bit fields
	if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		//Enable the parity control
		tempreg |= ( 1 << USART_CR1_PCE );

		// Enable EVEN parity
		//tempreg &= ( 1 << USART_CR1_PS )
		// Not required because by default EVEN parity will be selected once you enable the parity control

	}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
	{
		// Enable the parity control
	    tempreg |= ( 1 << USART_CR1_PCE);

	    // Enable ODD parity
	    tempreg |= ( 1 << USART_CR1_PS );

	}

   //Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

/******************************** Configuration of CR2******************************************/

	tempreg=0;

	// Configure the number of stop bits inserted during USART frame transmission
	tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;

/******************************** Configuration of CR3******************************************/

	tempreg=0;

	//Configuration of USART hardware flow control
	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		// Enable CTS flow control
		tempreg |= ( 1 << USART_CR3_CTSE );


	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		// Enable RTS flow control
		tempreg |= (1 << USART_CR3_RTSE );

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		// Enable both CTS and RTS Flow control
		tempreg |= ( ( 1 << USART_CR3_CTSE ) | ( 1<< USART_CR3_RTSE ) );
	}


	pUSARTHandle->pUSARTx->CR3 = tempreg;


/******************************** Configuration of BRR(Baudrate register)******************************************/

	// Configure the baud rate
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);

}



/****************************************************************************************
 * @fn 				- USART_DeConfig
 *
 * @ brief 			- This Function De-Initializes (Resets the Register) USART (Set the Values of Registers to the Reset Value)
 *
 * @param[in]		- Base Address of USART Peripheral
 * @param[in]		-
 * @param[in] 		-
 *
 * @return 			- none
 *
 * @Note			- none
 *
 */

void USART_DeConfig(USART_Handle_t *pUSARTHandle){

	if (pUSARTHandle->pUSARTx == USART1){
		USART1_REG_RESET();
	}
	else if (pUSARTHandle->pUSARTx == USART2){
		USART2_REG_RESET();
	}
	else if (pUSARTHandle->pUSARTx == USART3){
		USART3_REG_RESET();
	}
	else if (pUSARTHandle->pUSARTx == UART4){
		UART4_REG_RESET();
	}
	else if (pUSARTHandle->pUSARTx == UART4){
		UART5_REG_RESET();
	}
	else if (pUSARTHandle->pUSARTx == USART6){
		USART6_REG_RESET();
	}
}





/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{

	uint16_t *pdata;
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while(! USART_GetFlagStatus( pUSARTHandle->pUSARTx, USART_FLAG_TXE ) );

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_wordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);

			//Implement the code to increment the buffer address
			pTxBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while( ! USART_GetFlagStatus( pUSARTHandle->pUSARTx, USART_FLAG_TC ) );
}




/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */




void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while ( ! ( USART_GetFlagStatus( pUSARTHandle->pUSARTx, USART_FLAG_RXNE ) ) );

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.USART_wordLength == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE )
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

				//Now increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = ( pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

				 //Increment the pRxBuffer
				pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				 *pRxBuffer = (uint8_t)( pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF );
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = (uint8_t)( pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F );
			}

			//increment the pRxBuffer
			pRxBuffer++;
		}
	}

}


/*********************************************************************
 * @fn      		  - USART_SendDataWithIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->TxState;

	if(txstate != USART_ST_BUSY_TX)
	{
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxState = USART_ST_BUSY_TX;

		// Enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_TXEIE );

		// Enable interrupt for TC
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_TCIE );

	}
	return txstate;
}

/*********************************************************************
 * @fn      		  - USART_ReceiveDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->RxState;

	if( rxstate != USART_ST_BUSY_RX )
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxState = USART_ST_BUSY_RX;

		// Enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_RXNEIE );
	}
	return rxstate;
}


/*
 * USART Peripheral Control (Enable or Disable the USART Peripheral)
 */

/****************************************************************************************
 * @fn 				- USART_PeripheralControl
 *
 * @ brief 			- This Function Enables and Disables the USART Peripheral
 *					-
 *
 * @param[in]		- Base Address of USART Peripheral
 * @param[in]		- Enable or Disable
 * @param[in] 		-
 *
 * @return 			- none
 *
 * @Note			- The USART Needs to Enabled before Sending/Receiving Data and To be Disabled Before Configuring
 *
 */

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi){
	if (EnorDi == ENABLE){
		pUSARTx->CR1 |= ( 1 << USART_CR1_UE );

	}else {
		pUSARTx->CR1 &= ~( 1 << USART_CR1_UE );
	}
}



/*
 * Flag Status Read
 */
/****************************************************************************************
 * @fn 				- USART_GetFlagStatus
 *
 * @ brief 			- This Function returns the Status of the given Flag by Reading the Status Register
 *
 * @param[in]		- Base Address of USART Peripheral
 * @param[in]		- Flag Name
 * @param[in] 		-
 *
 * @return 			- FLAG_RESET (0) or FLAG_SET (1)
 *
 * @Note			- none
 *
 */

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName){

	if (pUSARTx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}




/*
 * Flag Clear
 */
/****************************************************************************************
 * @fn 				- USART_ClearFlag
 *
 * @ brief 			- This Function Clears the given Flag by writing 0 to the Status Register
 *
 * @param[in]		- Base Address of USART Peripheral
 * @param[in]		- Flag Name
 * @param[in] 		-
 *
 * @return 			- FLAG_RESET (0) or FLAG_SET (1)
 *
 * @Note			- none
 *
 */

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint32_t FlagName){

	pUSARTx->SR &= ~(FlagName);

}




/****************************************************************************************
 * @fn 				- USART_IRQConfig
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

void USART_IRQConfig(uint32_t IRQNumber, uint32_t IRQPriority, uint8_t EnorDi)
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


/*********************************************************************
 * @fn      		  - USART_IRQHandler
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
	uint32_t temp1 , temp2, temp3;

	uint16_t *pdata;

/*************************Check for TC flag ********************************************/

    // Check the state of TC bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TC );

	 // Check the state of TCIE bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TCIE );

	if(temp1 && temp2 )
	{
		//this interrupt is because of TC

		//close transmission and call application callback if TxLen is zero
		if ( pUSARTHandle->TxState == USART_ST_BUSY_TX )
		{
			//Check the TxLen . If it is zero then close the data transmission
			if(! pUSARTHandle->TxLen )
			{
				// Clear the TC flag
				pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_TC );

				// Clear the TCIE control bit
				pUSARTHandle->pUSARTx->CR1 &= ( 1<< USART_CR1_TCIE );

				// Reset the application state
				pUSARTHandle->TxState = USART_ST_READY;

				// Reset Buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;

				//Reset the length to zero
				pUSARTHandle->TxLen = 0;

				//Call the application call back with event USART_EVENT_TX_CMPLT
				USART_AppEventCallBack( pUSARTHandle, USART_EVENT_TX_CMPLT );
			}
		}
	}

/*************************Check for TXE flag ********************************************/

	// Check the state of TXE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TXE );

	//Implement the code to check the state of TXEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TXEIE );

	if(temp1 && temp2 )
	{
		//this interrupt is because of TXE
		if(pUSARTHandle->TxState == USART_ST_BUSY_TX)
		{
			//Keep sending data until Txlen reaches to zero
			if(pUSARTHandle->TxLen > 0)
			{
				//Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if(pUSARTHandle->USART_Config.USART_wordLength == USART_WORDLEN_9BITS)
				{
					//if 9BIT , load the DR with 2bytes masking the bits other than first 9 bits
					pdata = (uint16_t*)(pUSARTHandle->pTxBuffer);

					//loading only first 9 bits , so we have to mask with the value 0x01FF
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					//check for USART_ParityControl
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						// No parity is used in this transfer , so, 9bits of user data will be sent
						// Implement the code to increment pTxBuffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;

						// Implement the code to decrement the length
						pUSARTHandle->TxLen--;
						pUSARTHandle->TxLen--;

					}
					else
					{
						// Parity bit is used in this transfer . so , 8bits of user data will be sent
						// The 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;

						// Decrement the length
						pUSARTHandle->TxLen--;
					}
				}
				else
				{
					// This is 8bit data transfer
					pUSARTHandle->pUSARTx->DR = (*(pUSARTHandle->pTxBuffer) & (uint8_t)0xFF);

					// Increment the buffer address
					pUSARTHandle->pTxBuffer++;

					// Decrement the length
					pUSARTHandle->TxLen--;
				}
			}
			if (pUSARTHandle->TxLen == 0 )
			{
				//TxLen is zero
				//Implement the code to clear the TXEIE bit ( disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TXEIE );
			}
		}
	}

/*************************Check for RXNE flag ********************************************/

	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_RXNE );
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE );


	if(temp1 && temp2 )
	{
		//this interrupt is because of rxne
		//this interrupt is because of txe
		if(pUSARTHandle->RxState == USART_ST_BUSY_RX)
		{
			//TXE is set so send data
			if(pUSARTHandle->RxLen > 0)
			{
				//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if(pUSARTHandle->USART_Config.USART_wordLength == USART_WORDLEN_9BITS)
				{
					//We are going to receive 9bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used. so, all 9bits will be of user data

						//read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*)(pUSARTHandle->pRxBuffer)) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

						//Now increment the pRxBuffer two times
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;

						//Implement the code to decrement the length
						pUSARTHandle->RxLen--;
						pUSARTHandle->RxLen--;
					}
					else
					{
						//Parity is used. so, 8bits will be of user data and 1 bit is parity
						 *(pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

						 //Now increment the pRxBuffer
						 pUSARTHandle->pRxBuffer++;

						 //Implement the code to decrement the length
						 pUSARTHandle->RxLen--;
					}
				}
				else
				{
					//We are going to receive 8bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used , so all 8bits will be of user data

						//read 8 bits from DR
						 *(pUSARTHandle->pRxBuffer) = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
					}

					else
					{
						//Parity is used, so , 7 bits will be of user data and 1 bit is parity

						//read only 7 bits , hence mask the DR with 0X7F
						 *(pUSARTHandle->pRxBuffer) = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

					}

					//Now , increment the pRxBuffer
					pUSARTHandle->pRxBuffer++;

					//Implement the code to decrement the length
					pUSARTHandle->RxLen--;
				}


			}//if of >0

			if(! pUSARTHandle->RxLen)
			{
				//Disable the rxne
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE );
				pUSARTHandle->RxState = USART_ST_READY;
				USART_AppEventCallBack(pUSARTHandle, USART_EVENT_RX_CMPLT);
			}
		}
	}


/*************************Check for CTS flag ********************************************/
//Note : CTS feature is not applicable for UART4 and UART5

	//Implement the code to check the status of CTS bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_CTS );

	//Implement the code to check the state of CTSE bit in CR3
	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE );

	//Implement the code to check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)

	temp3 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSIE );

	if( temp1  && temp2 && temp3 )
	{
		//Implement the code to clear the CTS flag in SR
		pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_CTS );

		//this interrupt is because of cts
		USART_AppEventCallBack(pUSARTHandle, USART_EVENT_CTS );
	}

/*************************Check for IDLE detection flag ********************************************/

	//Implement the code to check the status of IDLE flag bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_IDLE );

	//Implement the code to check the state of IDLEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE );


	if(temp1 && temp2)
	{
		// Clear the IDLE flag.
		// To Clear the IDLE Flag Read the SR Register ( Done Above ) Followed by Read to DR
		// Doubt ???????????????????????????????
		temp3 = pUSARTHandle->pUSARTx->DR;
		//this interrupt is because of idle
		USART_AppEventCallBack(pUSARTHandle, USART_EVENT_IDLE);
	}

/*************************Check for Overrun detection flag ********************************************/

	//Implement the code to check the status of ORE flag  in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_ORE;

	//Implement the code to check the status of RXNEIE  bit in the CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;


	if(temp1  && temp2 )
	{
		//Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .
		USART_ClearFlag(pUSARTHandle->pUSARTx, USART_FLAG_ORE);

		//this interrupt is because of Overrun error
		USART_AppEventCallBack(pUSARTHandle, USART_EVENT_ORE);
	}



/*************************Check for Error Flag ********************************************/

//Noise Flag, Overrun error and Framing Error in multibuffer communication
//We dont discuss multibuffer communication in this course. please refer to the RM
//The blow code will get executed in only if multibuffer mode is used.

	temp2 =  pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE ) ;

	if(temp2)
	{
		temp1 = pUSARTHandle->pUSARTx->SR;
		if(temp1 & ( 1 << USART_SR_FE ))
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			USART_AppEventCallBack(pUSARTHandle,USART_ERREVENT_FE);
		}

		if(temp1 & ( 1 << USART_SR_NF ) )
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			USART_AppEventCallBack(pUSARTHandle, USART_ERREVENT_NF);
		}

		if(temp1 & ( 1 << USART_SR_ORE) )
		{
			USART_AppEventCallBack(pUSARTHandle,USART_ERREVENT_ORE);
		}
	}

}


