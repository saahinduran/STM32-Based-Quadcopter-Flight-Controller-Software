################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/adc_Battery.c \
../Core/Src/compass.c \
../Core/Src/e220.c \
../Core/Src/failsafe.c \
../Core/Src/gps.c \
../Core/Src/gy63-i2c.c \
../Core/Src/log_to_flash.c \
../Core/Src/main.c \
../Core/Src/mpu6050.c \
../Core/Src/pid.c \
../Core/Src/pwm_esc.c \
../Core/Src/receiver.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/adc_Battery.o \
./Core/Src/compass.o \
./Core/Src/e220.o \
./Core/Src/failsafe.o \
./Core/Src/gps.o \
./Core/Src/gy63-i2c.o \
./Core/Src/log_to_flash.o \
./Core/Src/main.o \
./Core/Src/mpu6050.o \
./Core/Src/pid.o \
./Core/Src/pwm_esc.o \
./Core/Src/receiver.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/adc_Battery.d \
./Core/Src/compass.d \
./Core/Src/e220.d \
./Core/Src/failsafe.d \
./Core/Src/gps.d \
./Core/Src/gy63-i2c.d \
./Core/Src/log_to_flash.d \
./Core/Src/main.d \
./Core/Src/mpu6050.d \
./Core/Src/pid.d \
./Core/Src/pwm_esc.d \
./Core/Src/receiver.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F413xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/adc_Battery.d ./Core/Src/adc_Battery.o ./Core/Src/adc_Battery.su ./Core/Src/compass.d ./Core/Src/compass.o ./Core/Src/compass.su ./Core/Src/e220.d ./Core/Src/e220.o ./Core/Src/e220.su ./Core/Src/failsafe.d ./Core/Src/failsafe.o ./Core/Src/failsafe.su ./Core/Src/gps.d ./Core/Src/gps.o ./Core/Src/gps.su ./Core/Src/gy63-i2c.d ./Core/Src/gy63-i2c.o ./Core/Src/gy63-i2c.su ./Core/Src/log_to_flash.d ./Core/Src/log_to_flash.o ./Core/Src/log_to_flash.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mpu6050.d ./Core/Src/mpu6050.o ./Core/Src/mpu6050.su ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/pwm_esc.d ./Core/Src/pwm_esc.o ./Core/Src/pwm_esc.su ./Core/Src/receiver.d ./Core/Src/receiver.o ./Core/Src/receiver.su ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

