################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/kbisland/SmallKatCharger/Core/Src/main.c \
/home/kbisland/SmallKatCharger/Core/Src/stm32l4xx_hal_msp.c \
/home/kbisland/SmallKatCharger/Core/Src/stm32l4xx_it.c 

OBJS += \
./Application/User/Core/main.o \
./Application/User/Core/stm32l4xx_hal_msp.o \
./Application/User/Core/stm32l4xx_it.o 

C_DEPS += \
./Application/User/Core/main.d \
./Application/User/Core/stm32l4xx_hal_msp.d \
./Application/User/Core/stm32l4xx_it.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/Core/main.o: /home/kbisland/SmallKatCharger/Core/Src/main.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32L431xx -I"/home/kbisland/SmallKatCharger/Core/Inc" -I"/home/kbisland/SmallKatCharger/Drivers/STM32L4xx_HAL_Driver/Inc" -I"/home/kbisland/SmallKatCharger/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"/home/kbisland/SmallKatCharger/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"/home/kbisland/SmallKatCharger/Drivers/CMSIS/Include" -I"/home/kbisland/SmallKatCharger/Core/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/Core/stm32l4xx_hal_msp.o: /home/kbisland/SmallKatCharger/Core/Src/stm32l4xx_hal_msp.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32L431xx -I"/home/kbisland/SmallKatCharger/Core/Inc" -I"/home/kbisland/SmallKatCharger/Drivers/STM32L4xx_HAL_Driver/Inc" -I"/home/kbisland/SmallKatCharger/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"/home/kbisland/SmallKatCharger/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"/home/kbisland/SmallKatCharger/Drivers/CMSIS/Include" -I"/home/kbisland/SmallKatCharger/Core/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/Core/stm32l4xx_it.o: /home/kbisland/SmallKatCharger/Core/Src/stm32l4xx_it.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32L431xx -I"/home/kbisland/SmallKatCharger/Core/Inc" -I"/home/kbisland/SmallKatCharger/Drivers/STM32L4xx_HAL_Driver/Inc" -I"/home/kbisland/SmallKatCharger/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"/home/kbisland/SmallKatCharger/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"/home/kbisland/SmallKatCharger/Drivers/CMSIS/Include" -I"/home/kbisland/SmallKatCharger/Core/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


