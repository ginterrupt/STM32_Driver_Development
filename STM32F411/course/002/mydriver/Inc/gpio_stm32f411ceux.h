#ifndef INC_GPIO_STM32F411CEUX_H_
#define INC_GPIO_STM32F411CEUX_H_

#include "stm32f411ceux.h"


typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t	PinMode;
	uint8_t PinSpeed;
	uint8_t PinPUPDControl;
	uint8_t PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_Config_t;
 
typedef struct
{
	GPIO_RegDef_t *pGPIOx;
	GPIO_Config_t GPIO_Config;

}GPIO_Handle_t;



#define GPIO_PIN_0		0
#define GPIO_PIN_1		1
#define GPIO_PIN_2		2
#define GPIO_PIN_3		3
#define GPIO_PIN_4		4
#define GPIO_PIN_5		5
#define GPIO_PIN_6		6
#define GPIO_PIN_7		7
#define GPIO_PIN_8		8
#define GPIO_PIN_9		9
#define GPIO_PIN_10		10
#define GPIO_PIN_11		11
#define GPIO_PIN_12		12
#define GPIO_PIN_13		13
#define GPIO_PIN_14		14
#define GPIO_PIN_15		15

//GPIO PIN Possible modes
#define GPIO_MODE_INPUT			0		//0b00
#define GPIO_MODE_OUTPUT		1		//0b01
#define GPIO_MODE_ALTF			2		//0b10
#define GPIO_MODE_ANALOG		3		//0b11

//possible output type
#define GPIO_OP_TYPE_PP			0
#define GPIO_OP_TYPE_OD			1


//possible speeds
#define GPIO_SPEED_LOW				0
#define GPIO_SPEED_MEDIUM			1
#define GPIO_SPEED_FAST				2
#define GPIO_SPEED_HIGH				3

//pullup/down contorl

#define GPIO_NO_PUPD				0
#define GPIO_PU						1
#define GPIO_PD						2




void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber );
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);

void GPIO_PeripClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);




#endif