/*!
 * @file   spi_003.c
 * @brief  Application code to send data over SPI2 to an Arduino in Full Duplex
 * Communication mode
 *
 * @date   April 11th, 2020
 * @author Irvin Ramirez
 */

#include "stm32f407xx.h"
#include <string.h>
#include <stdio.h>

extern void initialise_monitor_handles();

enum
{
	COMMAND_LED_CTRL = 0x50,
	COMMAND_SENSOR_READ,
	COMMAND_LED_READ,
	COMMAND_PRINT,
	COMMAND_ID_READ
};

enum
{
	LED_OFF,
	LED_ON 
};

enum
{
	ANALOG_PIN0,
	ANALOG_PIN1,
	ANALOG_PIN2,
	ANALOG_PIN3,
	ANALOG_PIN4,
	ANALOG_PIN5
};

enum
{
	LED_PIN	= 9
};

void delay(void)
{
	for(uint32_t i = 0; i < 500000 / 2; i++);
}

void SPI2_GPIOInit(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&SPIPins);
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPIPins);
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&SPIPins);
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPIPins);
}

void SPI2_Init(void)
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DevideMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPI2Handle);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn;

	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);
}

uint8_t SPI_VerifyResponse(uint8_t ackByte)
{
	if(ackByte == 0xF5)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}


int main(void)
{
	uint8_t dummyWrite = 0xFF;
	uint8_t dummyRead;

	initialise_monitor_handles();

	printf("Application is running\n");

	GPIO_ButtonInit();

	SPI2_GPIOInit();
	SPI2_Init();

	printf("SPI init done\n");

	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));
		delay();
		SPI_PeripheralControl(SPI2, ENABLE);

		// 1. CMD_LED_CTRL <pin no> <value>
		uint8_t commandCode = COMMAND_LED_CTRL;
		uint8_t ackByte;
		uint8_t args[2];

		SPI_SendData(SPI2, &commandCode, 1);
		SPI_ReceiveData(SPI2, &dummyRead, 1);
		SPI_SendData(SPI2, &dummyWrite, 1);
		SPI_ReceiveData(SPI2, &ackByte, 1);

		if(SPI_VerifyResponse(ackByte))
		{
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI2, args, 2);
			printf("COMMAND_LED_CTRL executed");
		}

		// 2. CMD_SENSOR_READ <analog pin number>
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));
		delay();

		commandCode = COMMAND_SENSOR_READ;

		SPI_SendData(SPI2, &commandCode, 1);
		SPI_ReceiveData(SPI2, &dummyRead, 1);
		SPI_SendData(SPI2, &dummyWrite, 1);
		SPI_ReceiveData(SPI2, &ackByte, 1);

		if(SPI_VerifyResponse(ackByte))
		{
			args[0] = ANALOG_PIN0;
			SPI_SendData(SPI2, args, 1);

			SPI_ReceiveData(SPI2, &dummyRead, 1);
			delay();
			SPI_SendData(SPI2, &dummyWrite, 1);
			uint8_t analogRead;
			SPI_ReceiveData(SPI2, &analogRead, 1);
			printf("COMMAND_SENSOR_READ executed %d\n", analogRead);
		}

		while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));
		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}
