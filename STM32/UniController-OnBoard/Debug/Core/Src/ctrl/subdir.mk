################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ctrl/kalman_filter.c \
../Core/Src/ctrl/regulator.c 

OBJS += \
./Core/Src/ctrl/kalman_filter.o \
./Core/Src/ctrl/regulator.o 

C_DEPS += \
./Core/Src/ctrl/kalman_filter.d \
./Core/Src/ctrl/regulator.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/ctrl/%.o Core/Src/ctrl/%.su Core/Src/ctrl/%.cyclo: ../Core/Src/ctrl/%.c Core/Src/ctrl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-ctrl

clean-Core-2f-Src-2f-ctrl:
	-$(RM) ./Core/Src/ctrl/kalman_filter.cyclo ./Core/Src/ctrl/kalman_filter.d ./Core/Src/ctrl/kalman_filter.o ./Core/Src/ctrl/kalman_filter.su ./Core/Src/ctrl/regulator.cyclo ./Core/Src/ctrl/regulator.d ./Core/Src/ctrl/regulator.o ./Core/Src/ctrl/regulator.su

.PHONY: clean-Core-2f-Src-2f-ctrl

