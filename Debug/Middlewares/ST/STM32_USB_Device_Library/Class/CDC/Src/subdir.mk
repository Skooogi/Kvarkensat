################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c 

OBJS += \
./Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.o 

C_DEPS += \
./Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/%.o Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/%.su: ../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/%.c Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Drivers/CMSIS/Include" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Drivers/STM32H7xx_HAL_Driver/Inc" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/USB_DEVICE/App" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Middlewares/USB_Device_Library" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Core/Inc" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/USB_DEVICE/Target" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/FreeRTOS/include" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Core/Inc" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/USB_DEVICE/Target" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/FreeRTOS/portable" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/FreeRTOS/portable/GCC/ARM_CM4F" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/FreeRTOS/portable/MemMang" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/FreeRTOS" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Class-2f-CDC-2f-Src

clean-Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Class-2f-CDC-2f-Src:
	-$(RM) ./Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.d ./Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.o ./Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.su

.PHONY: clean-Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Class-2f-CDC-2f-Src

