################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32f407_gpio_driver.c \
../drivers/Src/stm32f407_i2c_driver.c \
../drivers/Src/stm32f407_spi_driver.c \
../drivers/Src/stm32f407_usart_driver.c \
../drivers/Src/utils.c 

OBJS += \
./drivers/Src/stm32f407_gpio_driver.o \
./drivers/Src/stm32f407_i2c_driver.o \
./drivers/Src/stm32f407_spi_driver.o \
./drivers/Src/stm32f407_usart_driver.o \
./drivers/Src/utils.o 

C_DEPS += \
./drivers/Src/stm32f407_gpio_driver.d \
./drivers/Src/stm32f407_i2c_driver.d \
./drivers/Src/stm32f407_spi_driver.d \
./drivers/Src/stm32f407_usart_driver.d \
./drivers/Src/utils.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/stm32f407_gpio_driver.o: ../drivers/Src/stm32f407_gpio_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"/home/irvin/STM32CubeIDE/workspace_1.1.0/stm32f407-drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f407_gpio_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/Src/stm32f407_i2c_driver.o: ../drivers/Src/stm32f407_i2c_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"/home/irvin/STM32CubeIDE/workspace_1.1.0/stm32f407-drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f407_i2c_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/Src/stm32f407_spi_driver.o: ../drivers/Src/stm32f407_spi_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"/home/irvin/STM32CubeIDE/workspace_1.1.0/stm32f407-drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f407_spi_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/Src/stm32f407_usart_driver.o: ../drivers/Src/stm32f407_usart_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"/home/irvin/STM32CubeIDE/workspace_1.1.0/stm32f407-drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f407_usart_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/Src/utils.o: ../drivers/Src/utils.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"/home/irvin/STM32CubeIDE/workspace_1.1.0/stm32f407-drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/utils.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

