/*!
 * @file stm32f407_gpio_driver.c
 * @brief I2C Driver API
 * @date April 18, 2020
 */

#include "stm32f407_i2c_driver.h"
#include "utils.h"

uint16_t AHB_PRESCALER[] = {2, 4, 8, 16, 32, 64, 128, 256, 512};
uint8_t APB1_PRESCALER[4] = {2, 4, 8, 16};

uint32_t RCC_GetPllOutputClock()
{
	return 1;
}

uint32_t RCC_GetPclk1Value(void)
{
	uint32_t pclk1;
	uint32_t systemClock;
	uint8_t clockSource;
	uint8_t temp;
	uint8_t ahbp;
	uint8_t apb1p;

	clockSource = (RCC->CFGR >> 2) & 0x03;

	if(0 == clockSource)
	{
		systemClock = 16000000;
	}
	else if(1 == clockSource)
	{
		systemClock = 8000000;
	}
	else if(2 == clockSource)
	{
		systemClock = RCC_GetPllOutputClock();
	}

	// AHB
	temp = (RCC->CFGR >> 4) & 0x0F;

	if(temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_PRESCALER[temp - 8];
	}

	// APB1
	temp = (RCC->CFGR >> 10) & 0x07;

	if(temp < 4)
	{
		apb1p = 1;
	}
	else
	{
		apb1p = APB1_PRESCALER[temp - 4];
	}

	pclk1 = (systemClock / ahbp) / apb1p;

	return pclk1;
}

/*!
 * Enable or disable the I2C peripheral clock
 * @param i2cPeripheral The I2C peripheral
 * @param state The state of the I2C peripheral clock
 */
void I2C_PeriClockControl(I2C_RegDef_t *i2cPeripheral, uint8_t state)
{
	if(ENABLE == state)
	{
		if(I2C1 == i2cPeripheral)
		{
			I2C1_PCLK_EN();
		}
		else if(I2C2 == i2cPeripheral)
		{
			I2C2_PCLK_EN();
		}
		else if(I2C3 == i2cPeripheral)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		// TODO
	}
}

/*!
 * Initialize the I2C communication
 * @param instance The instance of the I2C driver
 */
void I2C_Init(I2C_Driver_t *instance)
{
	uint32_t tempReg = 0;

	// ACK control bit of CR1
	tempReg |= (instance->i2cConfig.I2C_AckControl << 10);
	instance->i2cPeripheral->CR1 = tempReg;

	// Configure the FREQ field of CR2
	tempReg = 0;
	tempReg |= RCC_GetPclk1Value() / 1000000U;
	instance->i2cPeripheral->CR2 = tempReg & 0x3F;

	// Program the device own address in OAR1
	tempReg |= instance->i2cConfig.I2C_DeviceAddress << 1;
	tempReg |= (1 << 14);
	instance->i2cPeripheral->OAR1 = tempReg;

	// CCR calculation
	uint16_t ccrValue = 0;
	tempReg = 0;
	if(instance->i2cConfig.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		ccrValue = RCC_GetPclk1Value() / (2 * instance->i2cConfig.I2C_SCLSpeed);
		tempReg |= ccrValue & 0xFFF;
	}
	else
	{
		tempReg |= (1 << 15);
		tempReg |= (instance->i2cConfig.I2C_FMDutyCycle << 14);
		if(I2C_FM_DUTY_2 == instance->i2cConfig.I2C_FMDutyCycle)
		{
			ccrValue = RCC_GetPclk1Value() / (3 * instance->i2cConfig.I2C_SCLSpeed);
		}
		else
		{
			ccrValue = RCC_GetPclk1Value() / (25 * instance->i2cConfig.I2C_SCLSpeed);
		}
		tempReg |= ccrValue & 0xFFF;
	}
	instance->i2cPeripheral->CCR |= tempReg;
}

/*!
 * De-initialize the I2C communication
 * @param i2cPeripheral The I2C peripheral
 */
void I2C_DeInit(I2C_RegDef_t *i2cPeripheral)
{
	// TODO
}

/*!
 * Enable or disable the I2C peripheral.
 * @param i2cPeripheral The I2C peripheral
 * @param state The state of the I2C peripheral
 */
void I2C_PeripheralControl(I2C_RegDef_t *i2cPeripheral, uint8_t state)
{
	if(ENABLE == state)
	{
		i2cPeripheral->CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		i2cPeripheral->CR1 &= ~(1 << I2C_CR1_PE);
	}
}
