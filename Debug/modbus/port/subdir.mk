################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../modbus/port/mt_port.c \
../modbus/port/portevent.c \
../modbus/port/portserial.c \
../modbus/port/porttimer.c 

OBJS += \
./modbus/port/mt_port.o \
./modbus/port/portevent.o \
./modbus/port/portserial.o \
./modbus/port/porttimer.o 

C_DEPS += \
./modbus/port/mt_port.d \
./modbus/port/portevent.d \
./modbus/port/portserial.d \
./modbus/port/porttimer.d 


# Each subdirectory must supply rules for building sources it contributes
modbus/port/%.o: ../modbus/port/%.c modbus/port/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F030x8 -c -I../Inc -I../modbus/function -I../modbus/include -I../modbus/port -I../modbus/rtu -I../modbus -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -Og -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-modbus-2f-port

clean-modbus-2f-port:
	-$(RM) ./modbus/port/mt_port.d ./modbus/port/mt_port.o ./modbus/port/portevent.d ./modbus/port/portevent.o ./modbus/port/portserial.d ./modbus/port/portserial.o ./modbus/port/porttimer.d ./modbus/port/porttimer.o

.PHONY: clean-modbus-2f-port

