################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
OBJS += \
./USB_DEVICE/App/usb_device.o \
./USB_DEVICE/App/usbd_cdc_if.o \
./USB_DEVICE/App/usbd_desc.o 

C_DEPS += \
./USB_DEVICE/App/usb_device.d \
./USB_DEVICE/App/usbd_cdc_if.d \
./USB_DEVICE/App/usbd_desc.d 


# Each subdirectory must supply rules for building sources it contributes
USB_DEVICE/App/%.o USB_DEVICE/App/%.su: ../USB_DEVICE/App/%.c USB_DEVICE/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Drivers/CMSIS/Include" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Drivers/STM32H7xx_HAL_Driver/Inc" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/USB_DEVICE/App" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Middlewares/USB_Device_Library" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Core/Inc" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/USB_DEVICE/Target" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/FreeRTOS/include" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Core/Inc" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/USB_DEVICE/Target" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/FreeRTOS/portable" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/FreeRTOS/portable/GCC/ARM_CM4F" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/FreeRTOS/portable/MemMang" -I"/home/lndn/Documents/Work/KvarkenSat/Kvarkensat/FreeRTOS" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-USB_DEVICE-2f-App

clean-USB_DEVICE-2f-App:
	-$(RM) ./USB_DEVICE/App/usb_device.d ./USB_DEVICE/App/usb_device.o ./USB_DEVICE/App/usb_device.su ./USB_DEVICE/App/usbd_cdc_if.d ./USB_DEVICE/App/usbd_cdc_if.o ./USB_DEVICE/App/usbd_cdc_if.su ./USB_DEVICE/App/usbd_desc.d ./USB_DEVICE/App/usbd_desc.o ./USB_DEVICE/App/usbd_desc.su

.PHONY: clean-USB_DEVICE-2f-App
