################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../PrivteCode/KEY/delay.c 

OBJS += \
./PrivteCode/KEY/delay.o 

C_DEPS += \
./PrivteCode/KEY/delay.d 


# Each subdirectory must supply rules for building sources it contributes
PrivteCode/KEY/delay.o: ../PrivteCode/KEY/delay.c PrivteCode/KEY/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Administrator/Desktop/ultrasonic_scan/PrivteCode/STEPPER" -I"C:/Users/Administrator/Desktop/ultrasonic_scan/PrivteCode/KEY" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"PrivteCode/KEY/delay.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

