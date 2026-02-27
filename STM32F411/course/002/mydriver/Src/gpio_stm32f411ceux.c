#include "stm32f411ceux.h"


/**********************************************
	* @fn 		- RCC Enable Func
	* @brief	- Enable or Disable selected peripheral on RCC
	*
	* @param 	- *pGPIOx
	* @param	- EnOrDi	//enable or disable selected peripheral
	*
	*
	* @retunr //void type no return
	*
	* @note	???

***********************************************/
void GPIO_PeripClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		
	}
	

}

/**********************************************
	* @fn 		- GPIO_INIT
	* @brief	- initialize configuration of GPIO
	*
	* @param 	- *pGPIOHandle
	* 
	*
	*
	* @retunr //void type no return
	*
	* @note	???

***********************************************/

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	
	
	GPIO_PeripClockControl(pGPIOHandle->pGPIOx,ENABLE);
	
	
	uint32_t temp = 0;
	// 1 select mode
	temp = (pGPIOHandle->GPIO_Config.PinMode << (2 * pGPIOHandle->GPIO_Config.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->MODER &= ~(0b11 << (2* pGPIOHandle->GPIO_Config.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->MODER |= temp;
	temp = 0;
	
	// 2 configure output tpye
	temp = (pGPIOHandle->GPIO_Config.PinOPType << ( pGPIOHandle->GPIO_Config.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0b11 << ( pGPIOHandle->GPIO_Config.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	
	// 3 config speed
	
	temp = (pGPIOHandle->GPIO_Config.PinSpeed << (2 * pGPIOHandle->GPIO_Config.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0b11 << (2* pGPIOHandle->GPIO_Config.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	
	// 4 config pull up / down
	
	temp = (pGPIOHandle->GPIO_Config.PinPUPDControl << (2 * pGPIOHandle->GPIO_Config.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0b11 << (2* pGPIOHandle->GPIO_Config.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;


}

/**********************************************
	* @fn 		- GPIO Deinit Func
	* @brief	- RESET GPIO Peripherals on RCC
	*
	* @param 	- *pGPIOx
	* 
	*
	*
	* @retunr //void type no return
	*
	* @note	???

***********************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}

}


/**********************************************
	* @fn 		- GPIO Input pin
	* @brief	- read the data from GPIO input pin
	*
	* @param 	- *pGPIOx
	* @param    - pinNumber
	*
	*
	* @retunr //void type no return
	*
	* @note	???

***********************************************/


uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber )
{
	uint8_t value;
	value = ((uint8_t)(pGPIOx->IDR >> pinNumber) & 1);

	return value;

}

/**********************************************
	* @fn 		- GPIO Input port
	* @brief	- read the data from GPIO input port
	*
	* @param 	- *pGPIOx
	* 
	*
	*
	* @retunr //void type no return
	*
	* @note	???

***********************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

/**********************************************
	* @fn 		- GPIO OUT pin
	* @brief	- Write the data to the GPIO  pin
	*
	* @param 	- *pGPIOx
	* @param    - pinNumber
	* @param 	- value
	*
	* @retunr //void type no return
	*
	* @note	???

***********************************************/



void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value)
{
	
	if(value == HIGH)
	{
		pGPIOx->ODR |= (1<<pinNumber);
	
	}
	else
	{
		pGPIOx->ODR &= ~(1<<pinNumber);
	}
}





/**********************************************
	* @fn 		- GPIO OUTPUT Port
	* @brief	- read the data from GPIO OUTPUT port
	*
	* @param 	- *pGPIOx
	* @param    - pinNumber
	*
	*
	* @retunr //void type no return
	*
	* @note	???

***********************************************/

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;

}




