################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/PuttyInterface.c \
../Src/gpio.c \
../Src/i2c.c \
../Src/main.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_it.c \
../Src/system_stm32f4xx.c \
../Src/usart.c \
../Src/vl6180x_api.c \
../Src/vl6180x_i2c.c 

OBJS += \
./Src/PuttyInterface.o \
./Src/gpio.o \
./Src/i2c.o \
./Src/main.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o \
./Src/system_stm32f4xx.o \
./Src/usart.o \
./Src/vl6180x_api.o \
./Src/vl6180x_i2c.o 

C_DEPS += \
./Src/PuttyInterface.d \
./Src/gpio.d \
./Src/i2c.d \
./Src/main.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d \
./Src/system_stm32f4xx.d \
./Src/usart.d \
./Src/vl6180x_api.d \
./Src/vl6180x_i2c.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F446xx -I"C:/Users/Gebruiker/Documents/werk/Roboteam/stm32/testproject/Inc" -I"C:/Users/Gebruiker/Documents/werk/Roboteam/stm32/testproject/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Gebruiker/Documents/werk/Roboteam/stm32/testproject/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Gebruiker/Documents/werk/Roboteam/stm32/testproject/Drivers/CMSIS/Include" -I"C:/Users/Gebruiker/Documents/werk/Roboteam/stm32/testproject/Drivers/STM32F4xx_HAL_Driver/Inc" -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


