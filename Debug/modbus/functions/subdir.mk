################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../modbus/functions/mbfunccoils.c \
../modbus/functions/mbfuncdiag.c \
../modbus/functions/mbfuncdisc.c \
../modbus/functions/mbfuncholding.c \
../modbus/functions/mbfuncinput.c \
../modbus/functions/mbfuncother.c \
../modbus/functions/mbutils.c 

OBJS += \
./modbus/functions/mbfunccoils.o \
./modbus/functions/mbfuncdiag.o \
./modbus/functions/mbfuncdisc.o \
./modbus/functions/mbfuncholding.o \
./modbus/functions/mbfuncinput.o \
./modbus/functions/mbfuncother.o \
./modbus/functions/mbutils.o 

C_DEPS += \
./modbus/functions/mbfunccoils.d \
./modbus/functions/mbfuncdiag.d \
./modbus/functions/mbfuncdisc.d \
./modbus/functions/mbfuncholding.d \
./modbus/functions/mbfuncinput.d \
./modbus/functions/mbfuncother.d \
./modbus/functions/mbutils.d 


# Each subdirectory must supply rules for building sources it contributes
modbus/functions/%.o: ../modbus/functions/%.c modbus/functions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F030x8 -c -I../Inc -I../modbus/function -I../modbus/include -I../modbus/port -I../modbus/rtu -I../modbus -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -Og -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-modbus-2f-functions

clean-modbus-2f-functions:
	-$(RM) ./modbus/functions/mbfunccoils.d ./modbus/functions/mbfunccoils.o ./modbus/functions/mbfuncdiag.d ./modbus/functions/mbfuncdiag.o ./modbus/functions/mbfuncdisc.d ./modbus/functions/mbfuncdisc.o ./modbus/functions/mbfuncholding.d ./modbus/functions/mbfuncholding.o ./modbus/functions/mbfuncinput.d ./modbus/functions/mbfuncinput.o ./modbus/functions/mbfuncother.d ./modbus/functions/mbfuncother.o ./modbus/functions/mbutils.d ./modbus/functions/mbutils.o

.PHONY: clean-modbus-2f-functions

