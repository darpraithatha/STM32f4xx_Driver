################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/011usart_tx.c 

OBJS += \
./Src/011usart_tx.o 

C_DEPS += \
./Src/011usart_tx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/011usart_tx.o: ../Src/011usart_tx.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I"C:/Users/Darp/Documents/Study/Embedded system Device Driver - UDEMY/Course - workspace/Hello-world/stm32f4xx_driver/drivers/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/011usart_tx.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

