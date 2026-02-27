#include "spi_stm32f411ceux.h"

/********************************************************
  * @fn                 - GPIO RCC enable func
  *
  * @brief              - this func enable or disable peripheral on RCC
  *
  * @param [in]         - *pGPIOx structure(GPIO_RegDef_t) where is  GPIO Registers
  * @param [in]         - EnOrDi Enable or disable clock
  * @param [in]         - 
  *
  * @return             - none
  *
  * @note               - none
********************************************************/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
                     SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2)
		{
                    SPI2_PCLK_EN();
		}else if(pSPIx == SPI3)
                {
                    SPI2_PCLK_EN();
                }
        }
        
}


/********************************************************
  * @fn                 - SPI_Init
  *
  * @brief              - this func initialize user SPI config
  *
  * @param [in]         - *pSPIHandle structure where is 2 more structure inside 1 *pSPIx GPIO Registers
			- and *SPI_Config where would be user settings (configuration)
  * @param [in]         - 
  * @param [in]         - 
  *
  * @return             - none
  *
  * @note               - none
********************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
    // 1 enable clock
    SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
    
    uint32_t temp = 0;
    //device mode
    temp |= pSPIHandle->SPIConf.DevMode << SPI_CR1_MSTR;
    //configure device mode
    //temp |= pSPIHandle->SPIConf.DevMode << 2;
    
    //configure bus config
    if(pSPIHandle->SPIConf.BusConf == SPI_BUS_FD)
    {
        //BIDIMODE should be cleared
        temp &= ~(1 << 15);
    }
    else if(pSPIHandle->SPIConf.BusConf == SPI_BUS_HD)
    {
        //BIDIMODE should be set
        temp |= (1 << 15);
    }
    else if(pSPIHandle->SPIConf.BusConf == SPI_BUS_SIMPLEX_RX)
    {
        //BIDI MODE should be cleared
        temp &= ~(1 << 15);
        //RXONLY mode should be set
        temp |= (1 << 10);
    }
    else if(pSPIHandle->SPIConf.BusConf == SPI_BUS_SIMPLEX_TX)
    {
        //BIDI MODE should be cleared
        //tempreg &= ~(1 << 15);
        //TXONLY mode should be set
        //tempreg |= (1 << 15);
    }
    
    //configure the SPI Serial clock speed (baud rate)
    temp |= pSPIHandle->SPIConf.SclkSpeed << 3;
    
    //configure the DFF
    temp |= pSPIHandle->SPIConf.DFF << 11;
    
    //configure the cpol
    temp |= pSPIHandle->SPIConf.CPOL << 1;
    
    
    //configure the chpa
    temp |= pSPIHandle->SPIConf.CPHA << 0;
    
    //configure the SSM
    temp |= pSPIHandle->SPIConf.SSM << 9;
    
    pSPIHandle->pSPIx->CR1 = temp;
    
    //pSPIHandle->pSPIx->CR1 |= (1 << 6);
	
}


void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
    

}




// spi get flag status 
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return 1;
	}
	return 0;
}

//send data
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuff, uint8_t Len)
{
    while (Len > 0)
    {
        
        while (SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == 0);
        
        //check dff bit
        if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
        {
            // 16 bit dff
            pSPIx->DR = *((uint16_t *)pTxBuff);
            Len --;
            Len --;
            (uint16_t*)pTxBuff ++;
        }
        else
        {
             // 8 bit dff
            pSPIx->DR = *pTxBuff;
            Len --;
            pTxBuff++;
        }
    
    }

}

/*********************************************************************
 * @fn      		  - SPI_ReceiveData
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
void SPI_Receive(SPI_RegDef_t *pSPIx, uint8_t *pRxBuff, uint8_t Len)
{
	while (Len > 0)
    {
        // 1. wait until RXNE is set
        while (SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG) == 0);
        
        // 2. check dff bit
        if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
        {
            // 16 bit dff
			//load the data from data register to rx buffer address
            *((uint16_t *)pRxBuff) = pSPIx->DR ;
            Len --;
            Len --;
            (uint16_t*)pRxBuff ++;
        }
        else
        {
            // 8 bit dff
            *(pRxBuff) = pSPIx->DR;
            Len --;
            pRxBuff++;
        }
    
    }
    
}


//Spi peripheral enable or disable function
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SPE);
    }
    else 
    {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
    }


}


//ssi enable
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SSI);
    }
    else 
    {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
    }
}


//ssOE enable
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
    }
    else 
    {
        pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
    }
}

/*********************************************************************
 * @fn      		  - SPI_IRQInterruptConfig
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
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(IRQNumber <= 31)								//0-31
		{
			// program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
			//*(uint32_t *)0xE000E100 |= (1 << 6);
		}
		else if (IRQNumber > 31 && IRQNumber < 64)		//32-63
		{
			// program ISER1 register
			*NVIC_ISER1 |= (1<<(IRQNumber%32));
			//*(uint32_t *)0xE000E104 |= (1<<(IRQNumber%32));
		}
		else if (IRQNumber > 64 && IRQNumber < 96)		//64-95
		{
			// program ISER2 register
			*NVIC_ISER2 |= (1<<(IRQNumber%32));
		}
	}
	else 
	{
		if(IRQNumber <= 31)								//0-31
		{
			// program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64)		//32-63
		{
			// program ICER1 register
			*NVIC_ICER1 |= (1<<(IRQNumber%32));
		}
		else if (IRQNumber > 64 && IRQNumber < 96)		//64-95
		{
			// program ICER2 register
			*NVIC_ICER2 |= (1<<(IRQNumber%32));
		}
	}

}

/*********************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// 1. find out the IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	
	uint8_t shift_amount = (8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED);
	
	*(NVIC_PR_BASE_ADDR + iprx )	|= (IRQPriority << shift_amount);
	//*(NVIC_PR_BASE_ADDR + (iprx * 4))	|= (IRQPriority <<(8*iprx_section));

}

/*********************************************************************
 * @fn      		  - SPI_SendDataIT
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
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuff, uint8_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX)
	{
	  //1 . Save the Tx buffer address and Len information in some global variables
	  pSPIHandle->pTxBuffer = pTxBuff;
	  pSPIHandle->TxLen = Len;
	  
	  //2.  Mark the SPI state as busy in transmission so that
	  //    no other code can take over same SPI peripheral until transmission is over
	  pSPIHandle->TxState = SPI_BUSY_IN_TX;
	  
	  //3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
	  pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return state;
}
/*********************************************************************
 * @fn      		  - SPI_ReceiveIT
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
uint8_t SPI_ReceiveIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuff, uint8_t Len)
{

	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX)
	{
	  //1 . Save the Tx buffer address and Len information in some global variables
	  pSPIHandle->pRxBuffer = pRxBuff;
	  pSPIHandle->TxLen = Len;
	  
	  //2.  Mark the SPI state as busy in transmission so that
	  //    no other code can take over same SPI peripheral until transmission is over
	  pSPIHandle->RxState = SPI_BUSY_IN_RX;
	  
	  //3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
	  pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	return state;




}



/*********************************************************************
 * @fn      		  - SPI_ReceiveIT
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
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1, temp2;
	//1 check TXE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	
	if (temp1 && temp2)
	{
		//handle TXE
		//spi_txe_interrupt_handle();
	}
	
	//1 check RXNE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
	
	if (temp1 && temp2)
	{
		//handle RXNE
		//spi_rxne_interrupt_handle();
	}
	
	//1 check OVR flag
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
	
	if (temp1 && temp2)
	{
		//handle RXNE
		//spi_ovr_interrupt_handle();
	}

}
