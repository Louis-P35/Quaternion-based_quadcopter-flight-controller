################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/Utils/utilsAlgebra.cpp \
../Core/Src/Utils/utilsTimer.cpp 

OBJS += \
./Core/Src/Utils/utilsAlgebra.o \
./Core/Src/Utils/utilsTimer.o 

CPP_DEPS += \
./Core/Src/Utils/utilsAlgebra.d \
./Core/Src/Utils/utilsTimer.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Utils/%.o Core/Src/Utils/%.su Core/Src/Utils/%.cyclo: ../Core/Src/Utils/%.cpp Core/Src/Utils/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Utils

clean-Core-2f-Src-2f-Utils:
	-$(RM) ./Core/Src/Utils/utilsAlgebra.cyclo ./Core/Src/Utils/utilsAlgebra.d ./Core/Src/Utils/utilsAlgebra.o ./Core/Src/Utils/utilsAlgebra.su ./Core/Src/Utils/utilsTimer.cyclo ./Core/Src/Utils/utilsTimer.d ./Core/Src/Utils/utilsTimer.o ./Core/Src/Utils/utilsTimer.su

.PHONY: clean-Core-2f-Src-2f-Utils

