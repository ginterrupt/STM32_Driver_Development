#include "stm32f411ceux.h"

void delay (uint32_t delayperiod)
{
	while (delayperiod--);
}

int main()
{
	
	
	
	GPIO_Handle_t btn;
	

	btn.pGPIOx = GPIOC;
	btn.GPIO_Config.GPIO_PinNumber = GPIO_PIN_13;
	btn.GPIO_Config.PinMode = GPIO_MODE_OUTPUT;
	btn.GPIO_Config.PinOPType = GPIO_OP_TYPE_PP;
	
	//GPIO_PeripClockControl(GPIOC,ENABLE);
	
	GPIO_Init(&btn);
	
	
	//GPIO_DeInit(GPIOA);
	
	
	while(1)
	{
		//pinA0 = GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0);
		
		
		GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_13, HIGH);
		delay (2000000);
		GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_13, LOW);
		delay (2000000);
		
		//GPIO_WriteToOutputPort(GPIOC, 0b000000000011111);
	}
	
	//int a = 15;
  //return 0;
}


