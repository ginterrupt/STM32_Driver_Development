#ifndef INC_STM32F411CEUX_H_
#define INC_STM32F411CEUX_H_

#include <stdint.h>




#define __vo	volatile
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U
#define APB1PERIPH_BASEADDR		0x40000000U
#define APB2PERIPH_BASEADDR		0x40010000U


#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR+0x3800)

#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR+0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR+0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR+0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR+0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR+0x1000)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR+0x1C00)

#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400)

//... I2C2, SPI1,SPI2,UART1...


//GPIO Regster Structure
typedef struct
{
	__vo uint32_t	MODER;				//GPIO PORT MODE REG offset is 0x00
	__vo uint32_t	OTYPER;				//GPIO port output type register offset is 0x04
	__vo uint32_t	OSPEEDR;			//GPIO port output speed register offset is 0x08
	__vo uint32_t	PUPDR;				//GPIO port pull-up/pull-down register offset is 0x0C
	__vo uint32_t	IDR;				//GPIO port input data register offset is 0x10
	__vo uint32_t	ODR;				//GPIO port output data register offset is 0x14
	__vo uint32_t	BSRR;				//GPIO port bit set/reset register offset is 0x18
	__vo uint32_t	LCKR;				//GPIO port configuration lock register offset is 0x1C
	__vo uint32_t	AFRL;				//GPIO alternate function low register offset is 0x20
	__vo uint32_t	AFRH;				//GPIO alternate function high register offset is 0x24
}GPIO_RegDef_t;

// RCC register structure
typedef struct
{
	__vo uint32_t CR;				//RCC clock control register 			0x00
	__vo uint32_t PLLCFGR;			//RCC PLL configuration register		0x04
	__vo uint32_t CFGR;				//RCC clock configuration register		0x08
	__vo uint32_t CIR;				//RCC clock interrupt register			0x0C
	__vo uint32_t AHB1RSTR;			//RCC AHB1 peripheral reset register	0x10
	__vo uint32_t AHB2RSTR;			//RCC AHB1 peripheral reset register	0x14
	__vo uint32_t reserved1;		// reserved 0x18
	__vo uint32_t reserved2;		//reserved 0x1C
	__vo uint32_t APB1RSTR;			//RCC APB1 peripheral reset register	0x20
	__vo uint32_t APB2RSTR;			//RCC APB2 peripheral reset register	0x24
	__vo uint32_t reserved3;		//reserved 0x28
	__vo uint32_t reserved4;		//reserved 0x2C
	__vo uint32_t AHB1ENR;			//RCC AHB1 peripheral clock enable register 0x30
	__vo uint32_t AHB2ENR;			//RCC AHB2 peripheral clock enable register 0x34
	__vo uint32_t reserved5;		//reversed 0x38
	__vo uint32_t reserved6;		//reversed 0x3C
	__vo uint32_t APB1ENR;			//RCC APB1 peripheral clock enable register 0x40
	__vo uint32_t APB2ENR;			//RCC APB2 peripheral clock enable register 0x44
	__vo uint32_t reserved7;		//reversed 0x48
	__vo uint32_t reserved8;		//reversed 0x4C
	__vo uint32_t AHB1LPENR;		//RCC AHB1 peripheral clock enable in low power mode register 0x50
	__vo uint32_t AHB2LPENR;		//RCC AHB2 peripheral clock enable in low power mode register 0x54
	__vo uint32_t reserved9;		//reversed 0x58
	__vo uint32_t reserved10;		//reversed 0x5C
	__vo uint32_t APB1LPENR;		//RCC APB1 peripheral clock enable in low power mode register 0x60
	__vo uint32_t APB2LPENR;		//RCC APB2 peripheral clock enabled in low power mode register 0x64
	__vo uint32_t reserved11;		//reversed 0x68
	__vo uint32_t reserved12;		//reversed 0x6C
	__vo uint32_t BDCR;				//RCC Backup domain control register 0x70
	__vo uint32_t CSR;				//RCC clock control & status register 0x74	
	__vo uint32_t reserved13;		//reversed 0x78
	__vo uint32_t reserved14;		//reversed 0x7C	
	__vo uint32_t SSCGR;			//RCC spread spectrum clock generation register 0x80
	__vo uint32_t PLLI2SCFGR;		//RCC PLLI2S configuration register 0x84
	__vo uint32_t reserved15;		//reversed 0x88
	__vo uint32_t DCKCFGR;			//RCC Dedicated Clocks Configuration Register 0x8C
}RCC_RegDef_t;

#define GPIOA 	((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB 	((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC 	((GPIO_RegDef_t *)GPIOC_BASEADDR)

#define RCC 	((RCC_RegDef_t *)RCC_BASEADDR)


#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1<<2))

#define GPIOA_REG_RESET() 	(RCC->AHB1RSTR |= (1 << 0))
#define GPIOB_REG_RESET() 	(RCC->AHB1RSTR |= (1 << 1))
#define GPIOC_REG_RESET() 	(RCC->AHB1RSTR |= (1 << 2))

#define ENABLE			1
#define DISABLE			0

#define HIGH			1
#define LOW				0

#include "gpio_stm32f411ceux.h"

#endif
