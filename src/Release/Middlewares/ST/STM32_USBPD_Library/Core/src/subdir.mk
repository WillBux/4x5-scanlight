################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/STM32_USBPD_Library/Core/src/usbpd_trace.c 

OBJS += \
./Middlewares/ST/STM32_USBPD_Library/Core/src/usbpd_trace.o 

C_DEPS += \
./Middlewares/ST/STM32_USBPD_Library/Core/src/usbpd_trace.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/STM32_USBPD_Library/Core/src/%.o Middlewares/ST/STM32_USBPD_Library/Core/src/%.su Middlewares/ST/STM32_USBPD_Library/Core/src/%.cyclo: ../Middlewares/ST/STM32_USBPD_Library/Core/src/%.c Middlewares/ST/STM32_USBPD_Library/Core/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -DUSE_FULL_LL_DRIVER -DUSBPD_PORT_COUNT=1 -D_RTOS -D_SNK -DUSE_HAL_DRIVER -DSTM32G0B1xx -DUSBPDCORE_LIB_PD3_FULL -c -I../Core/Inc -I../USBPD/App -I../USBPD/Target -I../USB_Device/App -I../USB_Device/Target -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../Middlewares/ST/STM32_USBPD_Library/Core/inc -I../Middlewares/ST/STM32_USBPD_Library/Devices/STM32G0XX/inc -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I../TCPP/App -I../TCPP -I../TCPP/Target -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Middlewares-2f-ST-2f-STM32_USBPD_Library-2f-Core-2f-src

clean-Middlewares-2f-ST-2f-STM32_USBPD_Library-2f-Core-2f-src:
	-$(RM) ./Middlewares/ST/STM32_USBPD_Library/Core/src/usbpd_trace.cyclo ./Middlewares/ST/STM32_USBPD_Library/Core/src/usbpd_trace.d ./Middlewares/ST/STM32_USBPD_Library/Core/src/usbpd_trace.o ./Middlewares/ST/STM32_USBPD_Library/Core/src/usbpd_trace.su

.PHONY: clean-Middlewares-2f-ST-2f-STM32_USBPD_Library-2f-Core-2f-src

