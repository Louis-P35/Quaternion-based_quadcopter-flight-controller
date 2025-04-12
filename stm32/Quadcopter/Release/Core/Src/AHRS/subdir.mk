################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/AHRS/complementaryFilter.cpp \
../Core/Src/AHRS/kalman.cpp \
../Core/Src/AHRS/madgwick.cpp 

OBJS += \
./Core/Src/AHRS/complementaryFilter.o \
./Core/Src/AHRS/kalman.o \
./Core/Src/AHRS/madgwick.o 

CPP_DEPS += \
./Core/Src/AHRS/complementaryFilter.d \
./Core/Src/AHRS/kalman.d \
./Core/Src/AHRS/madgwick.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/AHRS/%.o Core/Src/AHRS/%.su Core/Src/AHRS/%.cyclo: ../Core/Src/AHRS/%.cpp Core/Src/AHRS/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-AHRS

clean-Core-2f-Src-2f-AHRS:
	-$(RM) ./Core/Src/AHRS/complementaryFilter.cyclo ./Core/Src/AHRS/complementaryFilter.d ./Core/Src/AHRS/complementaryFilter.o ./Core/Src/AHRS/complementaryFilter.su ./Core/Src/AHRS/kalman.cyclo ./Core/Src/AHRS/kalman.d ./Core/Src/AHRS/kalman.o ./Core/Src/AHRS/kalman.su ./Core/Src/AHRS/madgwick.cyclo ./Core/Src/AHRS/madgwick.d ./Core/Src/AHRS/madgwick.o ./Core/Src/AHRS/madgwick.su

.PHONY: clean-Core-2f-Src-2f-AHRS

