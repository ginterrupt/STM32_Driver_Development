#include "stm32f411ceux.h"

void delay(uint32_t t)
{
	while (t--);
}


int main ()
{
	GPIO_Handle_t led;
	led.pGPIOx = GPIOC;
	led.GPIO_PinConfig.GPIO_PinNumber = 13;
	led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	
	GPIO_Init(&led);


	while(1)
	{
		GPIO_WriteToOutputPin(GPIOC,13,1);
		delay(1000000);
		GPIO_WriteToOutputPin(GPIOC,13,0);
		delay(1000000);
	
	}


}