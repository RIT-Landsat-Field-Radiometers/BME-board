################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32l4r9vgtx.s 

S_DEPS += \
./Core/Startup/startup_stm32l4r9vgtx.d 

OBJS += \
./Core/Startup/startup_stm32l4r9vgtx.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/startup_stm32l4r9vgtx.o: ../Core/Startup/startup_stm32l4r9vgtx.s
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -x assembler-with-cpp -MMD -MP -MF"Core/Startup/startup_stm32l4r9vgtx.d" -MT"$@"  -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

