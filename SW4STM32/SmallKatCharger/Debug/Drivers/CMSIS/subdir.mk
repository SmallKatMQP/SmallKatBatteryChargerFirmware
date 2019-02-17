################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/kbisland/SmallKatCharger/Core/Src/system_stm32l4xx.c 

OBJS += \
./Drivers/CMSIS/system_stm32l4xx.o 

C_DEPS += \
./Drivers/CMSIS/system_stm32l4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/system_stm32l4xx.o: /home/kbisland/SmallKatCharger/Core/Src/system_stm32l4xx.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32L431xx -I"/home/kbisland/SmallKatCharger/Core/Inc" -I"/home/kbisland/SmallKatCharger/Drivers/STM32L4xx_HAL_Driver/Inc" -I"/home/kbisland/SmallKatCharger/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"/home/kbisland/SmallKatCharger/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"/home/kbisland/SmallKatCharger/Drivers/CMSIS/Include" -I"/home/kbisland/SmallKatCharger/Core/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


