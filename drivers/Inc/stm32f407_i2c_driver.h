/*!
 * @file stm32f407_gpio_driver.h
 * @brief I2C Driver API
 * @date April 18, 2020
 */

#ifndef STM32F407_I2C_DRIVER_H
#define STM32F407_I2C_DRIVER_H

#include "stm32f407xx.h"

enum
{
	I2C_SCL_SPEED_SM = 100000,
	I2C_SCL_SPEED_FM_2K = 200000,
	I2C_SCL_SPEED_FM_4K = 400000
};

enum
{
	I2C_ACK_DISABLE,
	I2C_ACK_ENABLE
};

enum
{
	I2C_FM_DUTY_2,
	I2C_FM_DUTY_16_9
};

typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_AckControl;
	uint8_t I2C_FMDutyCycle;
} I2C_Config_t;

typedef struct
{
	I2C_RegDef_t *i2cPeripheral;
	I2C_Config_t i2cConfig;
} I2C_Driver_t;

void I2C_PeriClockControl(I2C_RegDef_t *i2cPeripheral, uint8_t state);
void I2C_Init(I2C_Driver_t *instance);
void I2C_DeInit(I2C_RegDef_t *i2cPeripheral);
void I2C_SendData(I2C_RegDef_t *i2cPeripheral, uint8_t *txBuffer, uint32_t length);
void I2C_ReceiveData(I2C_RegDef_t *i2cPeripheral, uint8_t *rxBuffer, uint32_t length);
void I2C_IRQInterruptConfig(uint8_t irqNumber, uint8_t state);
void I2C_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority);
void I2C_PeripheralControl(I2C_RegDef_t *i2cPeripheral, uint8_t state);
void I2C_SSIConfig(I2C_RegDef_t *i2cPeripheral, int8_t state);
void I2C_SSOEConfig(I2C_RegDef_t *i2cPeripheral, int8_t state);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *i2cPeripheral, uint32_t flagName);

#endif
