################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/Sensors/ICM29048.cpp \
../Core/Src/Sensors/mpu9250.cpp 

OBJS += \
./Core/Src/Sensors/ICM29048.o \
./Core/Src/Sensors/mpu9250.o 

CPP_DEPS += \
./Core/Src/Sensors/ICM29048.d \
./Core/Src/Sensors/mpu9250.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Sensors/%.o Core/Src/Sensors/%.su Core/Src/Sensors/%.cyclo: ../Core/Src/Sensors/%.cpp Core/Src/Sensors/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Sensors

clean-Core-2f-Src-2f-Sensors:
	-$(RM) ./Core/Src/Sensors/ICM29048.cyclo ./Core/Src/Sensors/ICM29048.d ./Core/Src/Sensors/ICM29048.o ./Core/Src/Sensors/ICM29048.su ./Core/Src/Sensors/mpu9250.cyclo ./Core/Src/Sensors/mpu9250.d ./Core/Src/Sensors/mpu9250.o ./Core/Src/Sensors/mpu9250.su

.PHONY: clean-Core-2f-Src-2f-Sensors

