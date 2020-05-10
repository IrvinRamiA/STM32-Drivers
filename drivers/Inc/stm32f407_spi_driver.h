/*
 * stm32f407_spi_driver.h
 *
 *  Created on: Apr 4, 2020
 *      Author: Irvin Ramirez
 */

#ifndef INC_STM32F407_SPI_DRIVER_H_
#define INC_STM32F407_SPI_DRIVER_H_

#include "stm32f407xx.h"

/*
 * Configuration structure for SPIx peripheral
 */
typedef struct
{
	uint8_t SPI_DevideMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
} SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t * pSPIx;		//This holds the base address of SPIx (x:1,2,3) peripheral
	SPI_Config_t SPIConfig;
} SPI_Handle_t;

/*
 * @SPI_DevideMode
 */
#define SPI_DEVICE_MODE_MASTER		1
#define	SPI_DEVICE_MODE_SLAVE		0

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD			1
#define SPI_BUS_CONFIG_HD			2
#define SPI_BUS_CONFIG_S_RX			3

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2			0
#define SPI_SCLK_SPEED_DIV4			1
#define SPI_SCLK_SPEED_DIV8			2
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS				0
#define SPI_DFF_16BITS				1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_HIGH				1
#define SPI_CPOL_LOW				0

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_HIGH				1
#define SPI_CPHA_LOW				0

/*
 * @SPI_SSM
 */
#define SPI_SSM_DI					0
#define SPI_SSM_EN					1

/*
 * SPI status flags definitions
 */
#define SPI_TXE_FLAG				(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG				(1 << SPI_SR_RXNE)
#define SPI_BSY_FLAG				(1 << SPI_SR_BSY)

/******************************************************************************
 * API supported by this driver
 * For more information about the APIs check the function definitions
 *****************************************************************************/

/*
 * Peripheral clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t * pSPIx, uint8_t EnOrDi);

/*
 * Init and de-init
 */
void SPI_Init(SPI_Handle_t * pSPIHandle);
void SPI_DeInit(SPI_RegDef_t * pSPIx);

/*
 * Data send and receive
 */
void SPI_SendData(SPI_RegDef_t * pSPIx, uint8_t * pTxBuffer, uint32_t length);
void SPI_ReceiveData(SPI_RegDef_t * pSPIx, uint8_t * pRxBuffer, uint32_t length);

/*
 * IRQ configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t * pHandle);

/*
 * Other peripheral control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, int8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, int8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, int8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t * pSPIx, uint32_t flagName);


#endif /* INC_STM32F407_SPI_DRIVER_H_ */
