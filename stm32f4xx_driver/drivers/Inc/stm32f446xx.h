/*
 * stm326446xx.h
 *
 *  Created on: Apr 5, 2020
 *  Author: Darp Raithatha
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>
#include <stddef.h>

#define __vo volatile
#define __weak __attribute__((weak))


/*************************************** Processor Specific Details **********************************/
/*
 * ARM Cortex M4 Processor NVIC ISERx Register Addresses
 */

#define NVIC_ISER0 				( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1 				( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2 				( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3 				( (__vo uint32_t*)0xE000E10C )

/*
 * ARM Cortex M4 Processor NVIC ICERx Register Addresses
 */

#define NVIC_ICER0 				( (__vo uint32_t*)0xE000E180 )
#define NVIC_ICER1 				( (__vo uint32_t*)0xE000E184 )
#define NVIC_ICER2 				( (__vo uint32_t*)0xE000E188 )
#define NVIC_ICER3 				( (__vo uint32_t*)0xE000E18C )

/*
 * ARM Cortex M4 Processor Interrupt Priority Register Addresses
 */

#define NVIC_PR_BASE_ADDR		( (__vo uint32_t*)0xE000E400)



#define NO_PR_BITS_IMPLEMENTED				4





/*************************************** Controller Specific Details **********************************/
/* Clock Source Frequencies */

#define HSI_Freq 				16000000
#define HSE_Freq				8000000


/*Base addresses of Flash, SRAM and ROM*/

#define FLASH_BASEADDR						0x08000000U   				/*BASE ADDRESS OF FLASH MEMORY */
#define SRAM1_BASEADDR						0x20000000U					/*BASE ADDRESS OF SRAM1 MEMORY */
#define SRAM2_BASEADDR						0X20001C00U					/*BASE ADDRESS OF SRAM1+SRAM_SIZE */
#define ROM_BASEADDR						0x1FFF0000U					/*BASE ADDRESS OF ROM (SYSTEM MEMORY) */
#define SRAM_BASEADDR						SRAM1_BASEADDR				/*BASE ADDRESS OF SRAM MEMORY (SRAM1) */


/* APBx and AHPx Peripheral base addresses */

#define PERIPH_BASE							0x40000000U					/* BASE ADDRESS OF PERPHERAL BUS */
#define APB1PERIPH_BASE						0x40000000U					/* BASE ADDRESS OF APB1 */
#define APB2PERIPH_BASE						0x40010000U					/* BASE ADDRESS OF APB2 */
#define AHB1PERIPH_BASE						0x40020000U					/* BASE ADDRESS OF AHB1 */
#define AHB2PERIPH_BASE						0x50000000U					/* BASE ADDRESS OF AHB2 */


/* Base Address of peripherals which are hanging on AHB1 Bus */

#define GPIOA_BASEADDR						(AHB1PERIPH_BASE + 0x0000)	/*BASE ADDRESS OF GPIOA REGISTER*/
#define GPIOB_BASEADDR						(AHB1PERIPH_BASE + 0x0400)	/*BASE ADDRESS OF GPIOB REGISTER*/
#define GPIOC_BASEADDR						(AHB1PERIPH_BASE + 0x0800)	/*BASE ADDRESS OF GPIOC REGISTER*/
#define GPIOD_BASEADDR						(AHB1PERIPH_BASE + 0x0C00)	/*BASE ADDRESS OF GPIOD REGISTER*/
#define GPIOE_BASEADDR						(AHB1PERIPH_BASE + 0x1000)	/*BASE ADDRESS OF GPIOE REGISTER*/
#define GPIOF_BASEADDR						(AHB1PERIPH_BASE + 0x1400)	/*BASE ADDRESS OF GPIOF REGISTER*/
#define GPIOG_BASEADDR						(AHB1PERIPH_BASE + 0x1800)	/*BASE ADDRESS OF GPIOG REGISTER*/
#define GPIOH_BASEADDR						(AHB1PERIPH_BASE + 0x1C00)	/*BASE ADDRESS OF GPIOH REGISTER*/

#define RCC_BASEADDR						(AHB1PERIPH_BASE + 0x3800)  /*BASE ADDRESS OF RCC REGISTER*/

/* Base Address of Peripherals which are hanging on APB1 Bus */
#define I2C1_BASEADDR						(APB1PERIPH_BASE + 0x5400)	/*BASE ADDRESS OF I2C1 REGISTER*/
#define I2C2_BASEADDR						(APB1PERIPH_BASE + 0x5800)	/*BASE ADDRESS OF I2C2 REGISTER*/
#define I2C3_BASEADDR						(APB1PERIPH_BASE + 0x5C00)	/*BASE ADDRESS OF I2C3 REGISTER*/

#define SPI2_BASEADDR						(APB1PERIPH_BASE + 0x3800)	/*BASE ADDRESS OF SPI2 REGISTER*/
#define SPI3_BASEADDR						(APB1PERIPH_BASE + 0x3C00)	/*BASE ADDRESS OF SPI3 REGISTER*/

#define USART2_BASEADDR						(APB1PERIPH_BASE + 0x4400)	/*BASE ADDRESS OF USART2 REGISTER*/
#define USART3_BASEADDR						(APB1PERIPH_BASE + 0x4800)	/*BASE ADDRESS OF USART3 REGISTER*/
#define UART4_BASEADDR						(APB1PERIPH_BASE + 0x4C00)	/*BASE ADDRESS OF UART4 REGISTER*/
#define UART5_BASEADDR						(APB1PERIPH_BASE + 0x5000)	/*BASE ADDRESS OF UART5 REGISTER*/


/* Base Address of Peripherals which are hanging on APB2 Bus */
#define SPI1_BASEADDR						(APB2PERIPH_BASE + 0x3000)	/*BASE ADDRESS OF SPI2 REGISTER*/
#define SPI4_BASEADDR						(APB2PERIPH_BASE + 0x3400)	/*BASE ADDRESS OF SPI4 REGISTER*/

#define USART1_BASEADDR						(APB2PERIPH_BASE + 0x1000)	/*BASE ADDRESS OF USART1 REGISTER*/
#define USART6_BASEADDR						(APB2PERIPH_BASE + 0x1400)	/*BASE ADDRESS OF USART6 REGISTER*/

#define EXTI_BASEADDR						(APB2PERIPH_BASE + 0x3C00)	/*BASE ADDRESS OF EXTI REGISTER*/

#define SYSCFG_BASEADDR						(APB2PERIPH_BASE + 0x3800)	/*BASE ADDRESS OF SYSCFG REGISTER*/



/******************************** Peripheral Register Definition Structure *****************************/

/* Note: Registers of a Peripheral are specific to MCU
 * Please check your Device Reference Manual
 */

/*
 * Peripheral register definition structure for GPIO Peripheral
 */

typedef struct
{
	__vo uint32_t MODER;			/* GPIO PORT MODE REGISTER						ADDRESS_OFFSET: 0x00 */
	__vo uint32_t OTYPER;			/* GPIO PORT OUTPUT TYPE REISTER		  		ADDRESS_OFFSET: 0x04 */
	__vo uint32_t OSPEEDER;			/* GPIO PORT OUTPUT SPEED REGISTER				ADDRESS_OFFSET: 0x08 */
	__vo uint32_t PUPDR;			/* GPIO PORT PULL-UP/PULL-DOWN REGISTER			ADDRESS_OFFSET: 0x0C */
	__vo uint32_t IDR;				/* GPIO PORT INPUT DATA REGISTER  				ADDRESS_OFFSET: 0x0C */
	__vo uint32_t ODR;				/* GPIO PORT OUTPUT DATA REGISTER  				ADDRESS_OFFSET: 0x0C */
	__vo uint32_t BSRR;				/* GPIO PORT BIT SET/RESET REGISTER  			ADDRESS_OFFSET: 0x0C */
	__vo uint32_t LCKR;				/* GPIO PORT CONFIGURATION LOCK REGISTER  		ADDRESS_OFFSET: 0x1C */
	__vo uint32_t AFR[2];			/* AFR[0]: GPIO ATERNATE FUNCTION LOW REGISTER	ADDRESS_OFFSET: 0x20 ,  AFR[1]: GPIO ALTERNATE FUNCTION HIGH REGISTER ADDRESS_OFFSET: 0x24 */

}GPIO_RegDef_t;


/*
 * Peripheral register definition structure for SPI and I2S Peripheral
 */
typedef struct
{
	__vo uint32_t CR1;				/* SPI control register 1						ADDRESS_OFFSET: 0x00 */
	__vo uint32_t CR2;				/* SPI control register 2						ADDRESS_OFFSET: 0x04 */
	__vo uint32_t SR;				/* SPI status register		  				ADDRESS_OFFSET: 0x08 */
	__vo uint32_t DR;				/* SPI data register						ADDRESS_OFFSET: 0x0C */
	__vo uint32_t CRCPR;			/* SPI CRC polynomial register				ADDRESS_OFFSET: 0x10 */
	__vo uint32_t RXCRCR;			/* SPI RX CRC register  					ADDRESS_OFFSET: 0x14 */
	__vo uint32_t TXCRCR;			/* SPI TX CRC register  					ADDRESS_OFFSET: 0x18 */
	__vo uint32_t I2SCFGR;			/* SPI_I2S configuration register  			ADDRESS_OFFSET: 0x1C */
	__vo uint32_t I2SPR;			/* SPI_I2S Pre-scaler register  			ADDRESS_OFFSET: 0x20 */

}SPI_RegDef_t;


/*
 * Peripheral register definition structure for I2C Peripheral
 */

typedef struct
{
	__vo uint32_t CR1;				/* I2C control register 1				ADDRESS_OFFSET: 0x00 */
	__vo uint32_t CR2;				/* I2C control register 2				ADDRESS_OFFSET: 0x04 */
	__vo uint32_t OAR1;				/* I2C Own address register 1  			ADDRESS_OFFSET: 0x08 */
	__vo uint32_t OAR2;				/* I2C Own address register 2			ADDRESS_OFFSET: 0x0C */
	__vo uint32_t DR;				/* I2C Data register					ADDRESS_OFFSET: 0x10 */
	__vo uint32_t SR1;				/* I2C status register 1  				ADDRESS_OFFSET: 0x14 */
	__vo uint32_t SR2;				/* I2C status register 2  				ADDRESS_OFFSET: 0x18 */
	__vo uint32_t CCR;				/* I2C clock control register  			ADDRESS_OFFSET: 0x1C */
	__vo uint32_t TRISE;			/* I2C TRISE(Rise Time) register  		ADDRESS_OFFSET: 0x20 */
	__vo uint32_t FLTR;				/* I2C FLTR Register (Filter)			ADDRESS_OFFSET: 0x24 */

}I2C_RegDef_t;


/*
 * Peripheral register definition structure for UART/USART Peripheral
 */
typedef struct{

	__vo uint32_t SR;				/* USART Status Register ADDRESS_OFFSET: 0x00 */
	__vo uint32_t DR;				/* USART Data Register ADDRESS_OFFSET: 0x04 */
	__vo uint32_t BRR;				/* USART Baud rate Register ADDRESS_OFFSET: 0x08 */
	__vo uint32_t CR1;				/* USART Control Register 1 ADDRESS_OFFSET: 0x0C */
	__vo uint32_t CR2;				/* USART Control Register 2 ADDRESS_OFFSET: 0x10 */
	__vo uint32_t CR3;				/* USART Control Register 3 ADDRESS_OFFSET: 0x14 */
	__vo uint32_t GTPR;				/* USART Guard time and prescaler Register ADDRESS_OFFSET: 0x18 */

}USART_RegDef_t;



/*
 * Peripheral register definition structure for EXTI Peripheral
 */
typedef struct
{
	__vo uint32_t IMR; 					/* Interrupt mask register 				ADDRESS_OFFSET: 0x00 */
	__vo uint32_t EMR;					/* Event mask register					ADDRESS_OFFSET: 0x04 */
	__vo uint32_t RTSR;					/* Rising trigger selection register	ADDRESS_OFFSET: 0x08 */
	__vo uint32_t FTSR;					/* Falling trigger selection register	ADDRESS_OFFSET: 0x0C */
	__vo uint32_t SWIER;				/* Software interrupt event register	ADDRESS_OFFSET: 0x10 */
	__vo uint32_t PR;					/* Pending register						ADDRESS_OFFSET: 0x14 */

}EXTI_RegDef_t;


/*
 * Peripheral register definition structure for SYSCFG
 */
typedef struct{

	__vo uint32_t MEMRMP;				/* SYSCFG memory remap register							ADDRESS_OFFSET: 0x00 */
	__vo uint32_t PMC;					/* SYSCFG peripheral mode configuration register		ADDRESS_OFFSET: 0x04 */
	__vo uint32_t EXTICR[4];			/* SYSCFG external interrupt configuration register		ADDRESS_OFFSET: 0x08 to 0x14 */
	uint32_t RESERVED1[2];				/* RESERVED 											ADDRESS_OFFSET: 0x18 to 0x1C */
	__vo uint32_t CMPCR;				/* Compensation cell control register					ADDRESS_OFFSET: 0x20 */
	uint32_t RESERVED2[2];				/* RESERVED 											ADDRESS_OFFSET: 0x24 to 0x28 */
	__vo uint32_t CFGR;					/* SYSCFG configuration register						ADDRESS_OFFSET: 0x1C */

}SYSCFG_RegDef_t;



/*
 * Peripheral register definition structure for RCC Peripheral
 */
typedef struct
{
	__vo uint32_t 	CR;					/* RCC clock control register 									ADDRESS_OFFSET: 0x00 */
	__vo uint32_t 	PLLCFGR;			/* RCC PLL configuration register								ADDRESS_OFFSET: 0x04 */
	__vo uint32_t 	CFGR;				/* RCC clock configuration register								ADDRESS_OFFSET: 0x08 */
	__vo uint32_t 	CIR;				/* RCC clock interrupt register									ADDRESS_OFFSET: 0x0C */
	__vo uint32_t 	AHB1RSTR;			/* RCC AHB1 peripheral reset register							ADDRESS_OFFSET: 0x10 */
	__vo uint32_t 	AHB2RSTR;			/* RCC AHB2 peripheral reset register							ADDRESS_OFFSET: 0x14 */
	__vo uint32_t 	AHB3RSTR;			/* RCC AHB3 peripheral reset register							ADDRESS_OFFSET: 0x18 */
	uint32_t 		RESERVED0;			/* RESERVED 													ADDRESS_OFFSET: 0x1C */
	__vo uint32_t 	APB1RSTR;			/* RCC APB1 peripheral reset register							ADDRESS_OFFSET: 0x20 */
	__vo uint32_t 	APB2RSTR;			/* RCC APB2 peripheral reset register							ADDRESS_OFFSET: 0x24 */
	uint32_t 		RESERVED2[2];		/* RESERVED[0] ADDRESS_OFFSET: 0x28, RESERVED[1] 				ADDRESS_OFFSET: 0x2C*/
	__vo uint32_t 	AHB1ENR;			/* RCC AHB1 peripheral clock enable register					ADDRESS_OFFSET: 0x30 */
	__vo uint32_t 	AHB2ENR;			/* RCC AHB2 peripheral clock enable register					ADDRESS_OFFSET: 0x34 */
	__vo uint32_t 	AHB3ENR;			/* RCC AHB3 peripheral clock enable register					ADDRESS_OFFSET: 0x38 */
	uint32_t 		RESERVED3;			/* RESERVED 													ADDRESS_OFFSET: 0x3C */
	__vo uint32_t 	APB1ENR;			/* RCC APB1 peripheral clock enable register					ADDRESS_OFFSET: 0x40 */
	__vo uint32_t 	APB2ENR;			/* RCC APB2 peripheral clock enable register					ADDRESS_OFFSET: 0x44 */
	uint32_t		RESERVED4[2];		/* RESERVED[0] ADDRESS_OFFSET: 0x48, RESERVED[1] 				ADDRESS_OFFSET: 0x4C */
	__vo uint32_t 	AHB1LPENR;			/* RCC AHB1 peripheral clock enable in low power mode register	ADDRESS_OFFSET: 0x50 */
	__vo uint32_t 	AHB2LPENR;			/* RCC AHB2 peripheral clock enable in low power mode register	ADDRESS_OFFSET: 0x54 */
	__vo uint32_t 	AHB3LPENR;			/* RCC AHB3 peripheral clock enable in low power mode register	ADDRESS_OFFSET: 0x58 */
	uint32_t 		RESERVED5;			/* RESERVED														ADDRESS_OFFSET: 0x5C */
	__vo uint32_t 	APB1LPENR;			/* RCC APB1 peripheral clock enable in low power mode register	ADDRESS_OFFSET: 0x60 */
	__vo uint32_t 	APB2LPENR;			/* RCC APB2 peripheral clock enabled in low power mode register	ADDRESS_OFFSET: 0x64 */
	uint32_t 		RESERVED6[2];		/* RESERVED[0] ADDRESS_OFFSET: 0x68, RESERVED[1] ADDRESS_OFFSET: 0x6C */
	__vo uint32_t 	BDCR;				/* RCC Backup domain control register							ADDRESS_OFFSET: 0x70 */
	__vo uint32_t 	CSR;				/* RCC clock control & status register							ADDRESS_OFFSET: 0x74 */
	uint32_t 		RESERVED7[2];		/* RESERVED[0] ADDRESS_OFFSET: 0x78, RESERVED[1] ADDRESS_OFFSET: 0x7C */
	__vo uint32_t 	SSCGR;				/* RCC spread spectrum clock generation register				ADDRESS_OFFSET: 0x80 */
	__vo uint32_t 	PLLI2SCFGR;			/* RCC PLLI2S configuration register							ADDRESS_OFFSET: 0x84 */
	__vo uint32_t 	PLLSAICFGR;			/* RCC PLL configuration register								ADDRESS_OFFSET: 0x88 */
	__vo uint32_t 	DCKCFGR;			/* RCC Dedicated Clock Configuration Register					ADDRESS_OFFSET: 0x8C */
	__vo uint32_t 	CKGATENR;			/* RCC clocks gated enable register								ADDRESS_OFFSET: 0x90 */
	__vo uint32_t 	DCKCFGR2;			/* RCC dedicated clocks configuration register 2				ADDRESS_OFFSET: 0x94 */

}RCC_RegDef_t;






/*
 * PERIPHERAL DEFINATION ( PERIPHERAL BASE ADDRESSES TYPECASTED TO xxx_RegDef_t)
 */

//GPIO
#define GPIOA			((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB			((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC			((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD			((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE			((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF			((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG			((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH			((GPIO_RegDef_t*)GPIOH_BASEADDR)


//SPI
#define SPI1			((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2			((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3			((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4 			((SPI_RegDef_t*)SPI4_BASEADDR)

//I2C
#define I2C1			((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2			((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3			((I2C_RegDef_t*)I2C3_BASEADDR)

//UART
#define UART4			((USART_RegDef_t*)UART4_BASEADDR)
#define UART5			((USART_RegDef_t*)UART5_BASEADDR)

//USART
#define USART1			((USART_RegDef_t*)USART1_BASEADDR)
#define USART2			((USART_RegDef_t*)USART2_BASEADDR)
#define USART3			((USART_RegDef_t*)USART3_BASEADDR)
#define USART6			((USART_RegDef_t*)USART6_BASEADDR)

//RCC
#define RCC 			((RCC_RegDef_t*)RCC_BASEADDR)

//EXTI
#define EXTI			((EXTI_RegDef_t*)EXTI_BASEADDR)

//SYSCFG
#define SYSCFG			((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/****************************** Clock Enable Macros ********************************/

/*
 * Clock Enable Macros for GPIOx Peripherals
 */
#define GPIOA_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 0 ) )
#define GPIOB_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 1 ) )
#define GPIOC_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 2 ) )
#define GPIOD_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 3 ) )
#define GPIOE_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 4 ) )
#define GPIOF_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 5 ) )
#define GPIOG_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 6 ) )
#define GPIOH_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 7 ) )


/*
 * Clock Enable Macros for I2Cx Peripherals
 */
#define I2C1_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 21 ) )
#define I2C2_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 22 ) )
#define I2C3_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 23 ) )


/*
 * Clock Enable Macros for SPIx Peripherals
 */
#define SPI1_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 12 ) )
#define SPI2_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 14 ) )
#define SPI3_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 15 ) )
#define SPI4_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 13 ) )

/*
 * Clock Enable Macros for USARTx Peripherals
 */
#define USART1_PCLK_EN() 	( RCC->APB2ENR |= ( 1 << 4 ) )
#define USART2_PCLK_EN() 	( RCC->APB1ENR |= ( 1 << 17 ) )
#define USART3_PCLK_EN() 	( RCC->APB1ENR |= ( 1 << 18 ) )
#define USART6_PCLK_EN() 	( RCC->APB2ENR |= ( 1 << 5 ) )


/*
 * Clock Enable Macros for UARTx Peripherals
 */
#define UART4_PCLK_EN() 	( RCC->APB1ENR |= ( 1 << 19 ) )
#define UART5_PCLK_EN() 	( RCC->APB1ENR |= ( 1 << 20 ) )


/*
 * Clock Enable Macros for SYSCFG Peripheral
 */
#define SYSCFG_PCLK_EN() 	( RCC->APB2ENR |= ( 1 << 14 ) )


/****************************** Clock Disable Macros ********************************/

/*
 * Clock Disable Macros for GPIOx Peripherals
 */
#define GPIOA_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 0 ) )
#define GPIOB_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 1 ) )
#define GPIOC_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 2 ) )
#define GPIOD_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 3 ) )
#define GPIOE_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 4 ) )
#define GPIOF_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 5 ) )
#define GPIOG_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 6 ) )
#define GPIOH_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 7 ) )



/*
 * Clock Disable Macros for I2Cx Peripherals
 */
#define I2C1_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 21 ) )
#define I2C2_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 22 ) )
#define I2C3_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 23 ) )



/*
 * Clock Disable Macros for SPIx Peripherals
 */
#define SPI1_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 12 ) )
#define SPI2_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 14 ) )
#define SPI3_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 15 ) )
#define SPI4_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 13 ) )



/*
 * Clock Disable Macros for USARTx Peripherals
 */

#define USART1_PCLK_DI() 	( RCC->APB2ENR &= ~( 1 << 4 ) )
#define USART2_PCLK_DI() 	( RCC->APB1ENR &= ~( 1 << 17 ) )
#define USART3_PCLK_DI() 	( RCC->APB1ENR &= ~( 1 << 18 ) )
#define USART6_PCLK_DI() 	( RCC->APB2ENR &= ~( 1 << 5 ) )



/*
 * Clock Disable Macros for UARTx Peripherals
 */

#define UART4_PCLK_DI() 	( RCC->APB1ENR &= ~( 1 << 19 ) )
#define UART5_PCLK_DI() 	( RCC->APB1ENR &= ~( 1 << 20 ) )


/*
 * Clock Disable Macros for SYSCFG Peripheral
 */

#define SYSCFG_PCLK_DI() 	( RCC->APB2ENR &= ~( 1 << 14 ) )

/*
 * GPIO Port to Code Macro
 */

#define GPIO_BASEADDR_TO_CODE(x) 	  (	(x == GPIOA) ? 0 :\
										(x == GPIOB) ? 1 :\
										(x == GPIOC) ? 2 :\
										(x == GPIOD) ? 3 :\
										(x == GPIOE) ? 4 :\
										(x == GPIOF) ? 5 :\
										(x == GPIOG) ? 6 :\
										(x == GPIOH) ? 7 : 0 )

/****************************************************************************
 * SPI Bit definition Macros for stm326446xx
 * Note: Update These Macros with the Valid values according to MCU
 ****************************************************************************/

/*
 * Bit Position Definition for SPI_CR1
 */
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRCNEXT			12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15


/*
 * Bit Position Definition for SPI_CR2
 */
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7


/*
 * Bit Position Definition for SPI_SR
 */
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8





/****************************************************************************
 * SPI Bit definition Macros for stm326446xx
 * Note: Update These Macros with the Valid values according to MCU
 ****************************************************************************/

/*
 * Bit Position Definition for SPI_CR1
 */
#define I2C_CR1_PE				0 				//Peripheral Enable
#define I2C_CR1_SMBUS			1				//SMBus Mode
#define I2C_CR1_SMBTYPE			3				//SMBus Type
#define I2C_CR1_ENARP			4
#define I2C_CR1_ENPEC			5
#define I2C_CR1_ENGC			6
#define I2C_CR1_NOSTRETCH		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10
#define I2C_CR1_POS				11
#define I2C_CR1_PEC				12
#define I2C_CR1_ALERT			13
#define I2C_CR1_SWRST			15


/*
 * Bit Position Definition for SPI_CR2
 */
#define I2C_CR2_FREQ			0
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN			10
#define I2C_CR2_DMAEN			11
#define I2C_CR2_LAST			12


/*
 * Bit Position Definition for SPI_OAR1
 */
#define I2C_OAR1_ADD0			0
#define I2C_OAR1_ADD71			1
#define I2C_OAR1_ADD98			8
#define I2C_OAR1_ADDMODE		15


/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_PECERR					12
#define I2C_SR1_TIMEOUT 				14
#define I2C_SR1_SMBALERT				15


/*
 * Bit position definitions I2C_SR2
 */

#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7


/*
 * Bit position definitions I2C_CCR
 */

#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15


/*
 * I2C Other Specifications
 */

#define I2C_MIN_FREQ_SM					1000000U			// Calculated from Max Rise Time of I2C in Standard Mode
#define I2C_MIN_FREQ_FM



/****************************************************************************
 * USART Bit definition Macros for stm326446xx
 * Note: Update These Macros with the Valid values according to MCU
 ****************************************************************************/

/*
 * Bit Position Definition for SPI_SR
 */
#define USART_SR_PE			0		// Parity Error
#define USART_SR_FE 		1		// Framing Error
#define USART_SR_NF			2		// Noise Detection Flag
#define USART_SR_ORE		3 		// Overrun Error
#define USART_SR_IDLE		4		// IDLE Line Detection
#define USART_SR_RXNE		5		// Receive Data Register Full
#define USART_SR_TC			6 		// Transmission Complete
#define USART_SR_TXE		7		// Transmit Data Register Empty
#define USART_SR_LBD 		8		// LIN break detection flag
#define USART_SR_CTS 		9 		// CTS Flag

/*
 * Bit Position Definition for SPI_CR1
 */
#define USART_CR1_SBK			0		// Send Break
#define USART_CR1_RWU			1		// Receiver Wakeup
#define USART_CR1_RE			2		// Receiver Enable
#define USART_CR1_TE			3		// Transmitter Enable
#define USART_CR1_IDLEIE		4		// IDLE Interrupt Enable
#define USART_CR1_RXNEIE		5		// RXNE Interrupt Enable
#define USART_CR1_TCIE 			6		// Transmission Complete Interrupt Enable
#define USART_CR1_TXEIE			7 		// TXE Interrupt Enable
#define USART_CR1_PEIE			8		// PE Interrupt Enable
#define USART_CR1_PS			9		// PS (Setting the Parity ODD/EVEN)
#define USART_CR1_PCE			10		// Parity Control Enable
#define USART_CR1_WAKE			11 		// Wakeup Method
#define USART_CR1_M 			12 		// Word Length
#define USART_CR1_UE			13		// USART Enable
#define USART_CR1_OVER8			15		// Oversampling Mode


/*
 * Bit Position Definition for SPI_CR2
 */
#define USART_CR2_LBDL			5		// lin break detection length (11/10 Bit Break Detection)
#define USART_CR2_LBDIE 		6		// LIN break detection interrupt enable
#define USART_CR2_LBCL			8 		// Last bit clock pulse ( Not Available for UART )
#define USART_CR2_CPHA			9		// Clock phase ( Not Available for UART )
#define USART_CR2_CPOL 			10 		// Clock Priority ( Not Available for UART )
#define USART_CR2_CLKEN 		11 		// Clock Enable ( Not Available for UART )
#define USART_CR2_STOP	 		12 		// USED to Specify STOP Bits (0.5, 1, 1.5, 2) ( 0.5 and 1.5 not Available for UART )
#define USART_CR2_LINEN			14		// LIN Mode Enable


/*
 * Bit Position Definition for SPI_CR3
 */
#define USART_CR3_EIE			0 		// Error Enable Interrupt
#define USART_CR3_IREN			1		// IrDA Mode Enable/Disable
#define USART_CR3_IRLP			2 		// IrDA Low Power
#define USART_CR3_HDSEL			3 		// Half-Duplex Selection
#define USART_CR3_NACK			4 		// Smartcard NACK Enable ( Not Available for UART )
#define USART_CR3_SCEN			5		// Smartcard Mode Enable ( Not Available for UART )
#define USART_CR3_DMAR			6 		// DMA Enable Receiver
#define USART_CR3_DMAT			7 		// DMA Enable Transmitter
#define USART_CR3_RTSE			8 		// RTS Enable
#define USART_CR3_CTSE			9		// CTS Enable
#define USART_CR3_CTSIE			10 		// CTS Interrupt Enable
#define USART_CR3_ONEBIT		11 		// Sample Bit Method Enable ( Noise Detection - NF will be Disabled )


/*
 * Bit Position Definition for SPI_CR3
 */



/****************************************************************************
 * IRQ (Interrupt Request) Numbers for stm326446xx
 * Note: Update These Macros with the Valid values according to MCU
 ****************************************************************************/

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51
#define IRQ_NO_SPI4			84
#define IRQ_NO_I2C1_EV		31
#define IRQ_NO_I2C1_ER		32
#define IRQ_NO_I2C2_EV		33
#define IRQ_NO_I2C2_ER		34
#define IRQ_NO_I2C3_EV		72
#define IRQ_NO_I2C3_ER		73
#define IRQ_NO_USART1		37
#define IRQ_NO_USART2		38
#define IRQ_NO_USART3		39
#define IRQ_NO_UART4		52
#define IRQ_NO_UART5		53
#define IRQ_NO_USART6		71





// Some Generic Macros

#define ENABLE 				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define FLAG_RESET			RESET
#define FLAG_SET			SET


#include "stm32f44xx_gpio_driver.h"
#include "stm32f44xx_spi_driver.h"
#include "stm32f44xx_i2c_driver.h"
#include "stm32f44xx_usart_driver.h"
#include "stm32f44xx_rcc_driver.h"


#endif /* INC_STM32F446XX_H_ */
