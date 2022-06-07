################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
OBJS += \
./Drivers/CMSIS/system_stm32h7xx.o 

C_DEPS += \
./Drivers/CMSIS/system_stm32h7xx.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/system_stm32h7xx.o: /home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Core/Src/system_stm32h7xx.c Drivers/CMSIS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Drivers/CMSIS/Include" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Drivers/STM32H7xx_HAL_Driver/Inc" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/USB_DEVICE/App" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Middlewares/USB_Device_Library" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Core/Inc" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/USB_DEVICE/Target" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/FreeRTOS/include" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Core/Inc" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/USB_DEVICE/Target" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/FreeRTOS/portable" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/FreeRTOS/portable/GCC/ARM_CM4F" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/FreeRTOS/portable/MemMang" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/FreeRTOS" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS

clean-Drivers-2f-CMSIS:
	-$(RM) ./Drivers/CMSIS/system_stm32h7xx.d ./Drivers/CMSIS/system_stm32h7xx.o ./Drivers/CMSIS/system_stm32h7xx.su

.PHONY: clean-Drivers-2f-CMSIS

