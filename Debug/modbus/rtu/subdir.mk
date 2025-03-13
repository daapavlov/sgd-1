################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../modbus/rtu/mbcrc.c \
../modbus/rtu/mbrtu.c 

OBJS += \
./modbus/rtu/mbcrc.o \
./modbus/rtu/mbrtu.o 

C_DEPS += \
./modbus/rtu/mbcrc.d \
./modbus/rtu/mbrtu.d 


# Each subdirectory must supply rules for building sources it contributes
modbus/rtu/%.o: ../modbus/rtu/%.c modbus/rtu/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F030x8 -c -I../Inc -I../modbus/function -I../modbus/include -I../modbus/port -I../modbus/rtu -I../modbus -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -Og -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-modbus-2f-rtu

clean-modbus-2f-rtu:
	-$(RM) ./modbus/rtu/mbcrc.d ./modbus/rtu/mbcrc.o ./modbus/rtu/mbrtu.d ./modbus/rtu/mbrtu.o

.PHONY: clean-modbus-2f-rtu

