################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/PID/pid.cpp 

OBJS += \
./Core/Src/PID/pid.o 

CPP_DEPS += \
./Core/Src/PID/pid.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/PID/%.o Core/Src/PID/%.su Core/Src/PID/%.cyclo: ../Core/Src/PID/%.cpp Core/Src/PID/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-PID

clean-Core-2f-Src-2f-PID:
	-$(RM) ./Core/Src/PID/pid.cyclo ./Core/Src/PID/pid.d ./Core/Src/PID/pid.o ./Core/Src/PID/pid.su

.PHONY: clean-Core-2f-Src-2f-PID

