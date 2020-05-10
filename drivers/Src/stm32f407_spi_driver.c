/*
 * stm32f407_spi_driver.c
 *
 *  Created on: Apr 4, 2020
 *      Author: Irvin Ramirez
 */

#include "stm32f407_spi_driver.h"

/*
 * Peripheral clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t * pSPIx, uint8_t EnOrDi)
{
	if(ENABLE == EnOrDi)
	{
		if(SPI1 == pSPIx)
		{
			SPI1_PCLK_EN();
		}
		else if(SPI2 == pSPIx)
		{
			SPI2_PCLK_EN();
		}
		else if(SPI3 == pSPIx)
		{
			SPI3_PCLK_EN();
		}
	}
	else
	{
		// TODO
	}
}

/*
 * Init and de-init
 */
void SPI_Init(SPI_Handle_t * pSPIHandle)
{
	// 1. Configuration of SPI_CR1 register
	uint32_t tempReg = 0;

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// 1.1 Configuration of device mode
	tempReg |= pSPIHandle->SPIConfig.SPI_DevideMode << SPI_CR1_MSTR;

	// 1.2 Configuration of bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// BIDIMODE Should be cleared
		tempReg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// BIDIMODE should be set
		tempReg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_S_RX)
	{
		// BIDIMODE should be cleared
		// RXONLY must be set
		tempReg &= ~(1 << SPI_CR1_BIDIMODE);
		tempReg |= (1 << SPI_CR1_RXONLY);
	}

	// 1.3 Configuration of SPI serial clock speed (baud rate)
	tempReg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	// 1.4 Configuration of DFF
	tempReg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	// 1.5 Configuration of CPOL
	tempReg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	// 1.6 Configuration of CPHA
	tempReg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	// 1.7 Configuration of SSM
	tempReg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempReg;
}

void SPI_DeInit(SPI_RegDef_t * pSPIx)
{
	// TODO
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t * pSPIx, uint32_t flagName)
{
	if(pSPIx->SR & flagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * Data send and receive (Blocking call)
 */
void SPI_SendData(SPI_RegDef_t * pSPIx, uint8_t * pTxBuffer, uint32_t length)
{
	while(length > 0)
	{
		// 1. Wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		// 2. Check DFF bit CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			// 16 bit DFF
			// Load data into DR
			pSPIx->DR = *((uint16_t *) pTxBuffer);
			length--;
			length--;
			(uint16_t *)pTxBuffer++;
		}
		else
		{
			// 8 bit DFF
			pSPIx->DR = *pTxBuffer;
			length--;
			pTxBuffer++;
		}
	}
}

void SPI_ReceiveData(SPI_RegDef_t * pSPIx, uint8_t * pRxBuffer, uint32_t length)
{
	while(length > 0)
	{
		// 1. Wait until RXNE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		// 2. Check DFF bit CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			// 16 bit DFF
			// Load data into pRxBuffer
			*((uint16_t *) pRxBuffer) = pSPIx->DR;
			length--;
			length--;
			(uint16_t *)pRxBuffer++;
		}
		else
		{
			// 8 bit DFF
			*pRxBuffer = pSPIx->DR;
			length--;
			pRxBuffer++;
		}
	}
}

/*
 * IRQ configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t * pHandle);

/*
 * Other peripheral control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, int8_t EnOrDi)
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

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, int8_t EnOrDi)
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

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, int8_t EnOrDi)
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
