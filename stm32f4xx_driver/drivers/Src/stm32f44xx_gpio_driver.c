/*
 * stm32f4xx_gpio_driver.c
 *
 *  Created on: Apr 11, 2020
 *      Author: Darp Raithatha
 */


#include <stm32f44xx_gpio_driver.h>



/*
 * Peripheral Clock Setup
 */

/****************************************************************************************
 * @fn 				- GPIO_PeriClockControl
 *
 * @ brief 			- This Function Enables or Disables Peripheral Clock for the Given GPIO
 *
 * @param[in]		- Base Address of GPIO Peripheral
 * @param[in]		- ENABLE or DISABLE Macros
 * @param[in] 		-
 *
 * @return 			- none
 *
 * @Note			- none
 *
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}
		else if (pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}
	}

	else{
		if (pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}
		else if (pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}
		else if (pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}
	}
}


/*
 * GPIO Init and De-Init
 */

/****************************************************************************************
 * @fn 				- GPIO_Init
 *
 * @ brief 			- This Function Initializes the GPIO By setting up the GPIO (Mode, Pull-up/PullDown, Speed, Output Type, Alternate Func)
 *
 * @param[in]		- GPIO_Handle_structure
 * @param[in]		-
 * @param[in] 		-
 *
 * @return 			- none
 *
 * @Note			- none
 *
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	// Enable Clock for the GPIO Peripheral
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);


	//1.Configure the Mode of GPIO Pin

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){

		//Non-Interrupt Mode
		temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clear
		pGPIOHandle->pGPIOx->MODER |= temp; // Set
	}

	else{
		//Interrupt Mode
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			//1. Configure the FTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			//Clear the corresponding RTSR Bit
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			//2. Configure the RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			//Clear the corresponding FTSR Bit
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			//3. Configure FTSR and RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}

		//2. Configure the GPIO PORT Selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4 ;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4 ;
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= ( ( GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx) ) << (temp2 * 4));

		//3. Enable the EXTI Interrupt Delivery using IMR
		EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

	}

	temp = 0;

	//2. Configure the Speed
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clear
	pGPIOHandle->pGPIOx->OSPEEDER |= temp; // Set

	temp = 0;

	//3. Configure the Pull-up/Pull-down Settings

	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clear
	pGPIOHandle->pGPIOx->PUPDR |= temp; // Set

	temp = 0;

	//4. Configure the Output Type

	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType) );
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ); // Clear
	pGPIOHandle->pGPIOx->OTYPER |= temp; // Set

	temp = 0;

	//5. Configure the Alternate Functionality

	//If the Pin Mode is Alternate Functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){

		//Configure Alternate Functionality Register
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~( 0xF << (4 * temp2) ); //Clear
		pGPIOHandle->pGPIOx->AFR[temp1] |= ( pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2) ); //Set

	}

}


/****************************************************************************************
 * @fn 				- GPIO_DeInit
 *
 * @ brief 			- This Function De-Initializes GPIO (Set the Values of Registers to the Reset Value)
 *
 * @param[in]		- Base Address of GPIO Peripheral
 * @param[in]		-
 * @param[in] 		-
 *
 * @return 			- none
 *
 * @Note			- none
 *
 */

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

	if (pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}
	else if (pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}
	else if (pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}
	else if (pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}
	else if (pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	}
	else if (pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}
}


/*
 * GPIO Read and Write
 */


/****************************************************************************************
 * @fn 				- GPIO_ReadFromInputPin
 *
 * @ brief 			- This Function Reads GPIO Input Registers for the Given GPIO PinNumber
 *
 * @param[in]		- Base Address of GPIO Peripheral
 * @param[in]		- Pin Number
 * @param[in] 		-
 *
 * @return 			- Value at the Input Pin (1 or 0)
 *
 * @Note			- none
 *
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)( (pGPIOx->IDR >> PinNumber) & 0x00000001 );
	return value;
}


/****************************************************************************************
 * @fn 				- GPIO_ReadFromInputPort
 *
 * @ brief 			- This Function Reads the GPIO Port Value
 *
 * @param[in]		- Base Address of GPIO Peripheral
 * @param[in]		-
 * @param[in] 		-
 *
 * @return 			- Value at the input Port (0x0000 to 0xFFFF)
 *
 * @Note			- none
 *
 */

uint8_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}



/****************************************************************************************
 * @fn 				- GPIO_WriteToOutputPin
 *
 * @ brief 			- This Enables or Disables the GPIO Pin given the Port Address and PinNumber
 *
 * @param[in]		- Base Address of GPIO Peripheral
 * @param[in]		- PinNumber (0 to 15)
 * @param[in] 		- Value that needs to be set at the Pin (ENABLE / DISABLE Macros or 1/0)
 *
 * @return 			- none
 *
 * @Note			- none
 *
 */

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET){
		//Write 1 to the register at Bit Field corresponding to the Pin Number
		pGPIOx->ODR |= ( 1 << PinNumber );
	}
	else{
		//Write 0 to the Register at the Bit Filed corresponding to the Pin Number
		pGPIOx->ODR &= ~( 1 << PinNumber );
	}

}


/****************************************************************************************
 * @fn 				- GPIO_WriteToOutputPort
 *
 * @ brief 			- This Function Sets the Port to the Given Value
 *
 * @param[in]		- Base Address of GPIO Peripheral
 * @param[in]		- Value that needs to be set (0x0000 to 0xFFFF)
 * @param[in] 		-
 *
 * @return 			- none
 *
 * @Note			- none
 *
 */

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;

}


/****************************************************************************************
 * @fn 				- GPIO_ToggleOutputPin
 *
 * @ brief 			- This Function Toggles the Pin Given the Port address and PinNumber
 *
 * @param[in]		- Base Address of GPIO Peripheral
 * @param[in]		- ENABLE or DISABLE Macros
 * @param[in] 		-
 *
 * @return 			- none
 *
 * @Note			- none
 *
 */

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}


/*
 * IRQ Configuration and ISR Handling
 */

/****************************************************************************************
 * @fn 				- GPIO_IRQConfig
 *
 * @ brief 			- This Function Enables or Disables the Interrupt for the Given GPIO and also sets the Priority of the the Interrupt
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

void GPIO_IRQConfig(uint32_t IRQNumber, uint32_t IRQPriority, uint8_t EnorDi)
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
 * @fn 				- GPIO_IRQHandling
 *
 * @ brief 			- This Function Clears the Pending Register for the Given Pin Number
 *
 * @param[in]		- PinNumber (On which the Interrupt occurred)
 * @param[in]		-
 * @param[in] 		-
 *
 * @return 			- none
 *
 * @Note			- none
 *
 */

void GPIO_IRQHandling(uint8_t PinNumber)
{
	//Clear the EXTI PR Register
	if ( EXTI->PR & (1 << PinNumber) ){
		EXTI->PR |= ( 1 << PinNumber );
	}
}
