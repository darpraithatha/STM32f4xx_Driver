/*
 * stm32f44xx_rcc_driver.c
 *
 *  Created on: May 26, 2020
 *      Author: Darp Raithatha
 */

#include "stm32f44xx_rcc_driver.h"


uint16_t AHB_PreScalar[8] = { 2, 4, 8, 16, 64, 128, 256, 512 };
uint16_t APB_PreScalar[4] = { 2, 4, 8, 16 };


/****************************************************************************************
 * @fn 				- RCC_GetPLLOutputClock
 *
 * @ brief 			- This Function is the Calculate the PLL Clock Value by Fetching various PreScalars
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in] 		-
 *
 * @return 			- The Value of PLL Clock
 *
 * @Note			-
 *
 */

uint32_t RCC_GetPLLOutputClock(void){

	//Implement Latter
	return 0;
}


/****************************************************************************************
 * @fn 				- RCC_GetPCLK1Value
 *
 * @ brief 			- This Function is the Calculate the Peripheral 1 Clock (APB1 Clock) Value by Fetching various PreScalars
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in] 		-
 *
 * @return 			- The Value of Peripheral Clock
 *
 * @Note			-
 *
 */

uint32_t RCC_GetPCLK1Value(void){
	uint32_t pclk1, SystemClk;

	uint8_t clkscr, temp, ahbp, apb1p;

	clkscr = ((RCC->CFGR >> 2) & 0x3);
	if (clkscr == 0){
		SystemClk = HSI_Freq;
	}
	else if (clkscr == 1){
		SystemClk = HSE_Freq;
	}
	else if (clkscr == 2){
		SystemClk = RCC_GetPLLOutputClock();
	}

	// AHP PreScalar
	temp = ((RCC->CFGR >> 4) & 0xF);
	if(temp < 8){
		ahbp = 1;
	}
	else{
		ahbp = AHB_PreScalar[temp-8];
	}

	//APB1 PreScalar
	temp = ((RCC->CFGR >> 10) & 0x7);
	if(temp < 4){
		apb1p = 1;
	}
	else{
		apb1p = APB_PreScalar[temp-4];
	}

	pclk1 = ( (SystemClk / ahbp) / apb1p );

	return pclk1;
}




/****************************************************************************************
 * @fn 				- RCC_GetPCLK2Value
 *
 * @ brief 			- This Function is the Calculate the Peripheral 1 Clock (APB2 Clock) Value by Fetching various PreScalars
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in] 		-
 *
 * @return 			- The Value of Peripheral Clock
 *
 * @Note			-
 *
 */

uint32_t RCC_GetPCLK2Value(void){

	uint32_t pclk2, SystemClk;

	uint8_t clkscr, temp, ahbp, apb2p;

	clkscr = ((RCC->CFGR >> 2) & 0x3);

	if (clkscr == 0){
		SystemClk = HSI_Freq;
	}
	else if (clkscr == 1){
		SystemClk = HSE_Freq;
	}
	else if (clkscr == 2){
		SystemClk = RCC_GetPLLOutputClock();
	}

	// AHP PreScalar
	temp = ((RCC->CFGR >> 4) & 0xF);
	if(temp < 8){
		ahbp = 1;
	}
	else{
		ahbp = AHB_PreScalar[temp-8];
	}

	//APB2 PreScalar
	temp = ((RCC->CFGR >> 13) & 0x7);
	if(temp < 4){
		apb2p = 1;
	}
	else{
		apb2p = APB_PreScalar[temp-4];
	}

	pclk2 = ( (SystemClk / ahbp) / apb2p );

	return pclk2;
}



