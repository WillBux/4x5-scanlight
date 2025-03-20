################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../TCPP/Target/usbpd_ADC_SNK.c 

OBJS += \
./TCPP/Target/usbpd_ADC_SNK.o 

C_DEPS += \
./TCPP/Target/usbpd_ADC_SNK.d 


# Each subdirectory must supply rules for building sources it contributes
TCPP/Target/%.o TCPP/Target/%.su TCPP/Target/%.cyclo: ../TCPP/Target/%.c TCPP/Target/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -DUSE_FULL_LL_DRIVER -DUSBPD_PORT_COUNT=1 -D_RTOS -D_SNK -DUSE_HAL_DRIVER -DSTM32G0B1xx -DUSBPDCORE_LIB_PD3_FULL -c -I../Core/Inc -I../USBPD/App -I../USBPD/Target -I../USB_Device/App -I../USB_Device/Target -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../Middlewares/ST/STM32_USBPD_Library/Core/inc -I../Middlewares/ST/STM32_USBPD_Library/Devices/STM32G0XX/inc -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I../TCPP/App -I../TCPP -I../TCPP/Target -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-TCPP-2f-Target

clean-TCPP-2f-Target:
	-$(RM) ./TCPP/Target/usbpd_ADC_SNK.cyclo ./TCPP/Target/usbpd_ADC_SNK.d ./TCPP/Target/usbpd_ADC_SNK.o ./TCPP/Target/usbpd_ADC_SNK.su

.PHONY: clean-TCPP-2f-Target

