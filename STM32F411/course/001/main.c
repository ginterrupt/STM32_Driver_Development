#include "stdint.h"

#define HIGH		1
#define LOW			0

#define RCC_BASE				0x40023800
#define GPIOC_BASE				0x40020800

#define	RCC_AHB1ENR				(*(volatile uint32_t *)(RCC_BASE + 0x30))	//0x40023830
#define GPIOC_MODER				(*(volatile uint32_t *)(GPIOC_BASE + 0x00))
#define GPIOC_ODR				(*(volatile uint32_t *)(GPIOC_BASE + 0x14))

void delay(uint32_t delaytime)
{
	while(delaytime--);
}
void ledon(uint8_t pin, uint8_t value)
{
	GPIOC_ODR	|= (value<< pin);

}
int main()
{	
	//enable GPIOC on RCC
	RCC_AHB1ENR	|= (1 << 2);
	//clear gpioc port13 moder bits
	GPIOC_MODER &= ~(0b11 << 26);	
	//set GPIOC 13 output
	GPIOC_MODER |= (0b01 << 26);
	
	while(1)
	{
		//set GPIOC 13 high
		
		//GPIOC_ODR	|= (1 << 13);
		ledon(13,HIGH);
		delay(1000000);
	
		//set GPIOC 13 high
		GPIOC_ODR	&= ~(0b1 << 13);
		delay(1000000);
		
	}
	
  //return 0;
}
