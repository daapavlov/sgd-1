################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/Mode_SGD-1.c \
../Src/TimerConf.c \
../Src/indicator_7s_sr.c \
../Src/main.c \
../Src/stm32f0xx_hal_msp.c \
../Src/stm32f0xx_hal_timebase_TIM.c \
../Src/stm32f0xx_it.c 

OBJS += \
./Src/Mode_SGD-1.o \
./Src/TimerConf.o \
./Src/indicator_7s_sr.o \
./Src/main.o \
./Src/stm32f0xx_hal_msp.o \
./Src/stm32f0xx_hal_timebase_TIM.o \
./Src/stm32f0xx_it.o 

C_DEPS += \
./Src/Mode_SGD-1.d \
./Src/TimerConf.d \
./Src/indicator_7s_sr.d \
./Src/main.d \
./Src/stm32f0xx_hal_msp.d \
./Src/stm32f0xx_hal_timebase_TIM.d \
./Src/stm32f0xx_it.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F030x8 -c -I../Inc -I../modbus/function -I../modbus/include -I../modbus/port -I../modbus/rtu -I../modbus -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -Og -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/Mode_SGD-1.d ./Src/Mode_SGD-1.o ./Src/TimerConf.d ./Src/TimerConf.o ./Src/indicator_7s_sr.d ./Src/indicator_7s_sr.o ./Src/main.d ./Src/main.o ./Src/stm32f0xx_hal_msp.d ./Src/stm32f0xx_hal_msp.o ./Src/stm32f0xx_hal_timebase_TIM.d ./Src/stm32f0xx_hal_timebase_TIM.o ./Src/stm32f0xx_it.d ./Src/stm32f0xx_it.o

.PHONY: clean-Src

