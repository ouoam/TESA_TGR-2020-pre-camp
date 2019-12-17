################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
D:/project/TESA-Top-Gun-Rally-2020/STM32CubeExpansion_LRWAN_V1.3.1/Projects/B-L072Z-LRWAN1/Applications/LoRa/End_Node/SW4STM32/startup_stm32l072xx.s 

OBJS += \
./Application/SW4STM32/startup_stm32l072xx.o 


# Each subdirectory must supply rules for building sources it contributes
Application/SW4STM32/startup_stm32l072xx.o: D:/project/TESA-Top-Gun-Rally-2020/STM32CubeExpansion_LRWAN_V1.3.1/Projects/B-L072Z-LRWAN1/Applications/LoRa/End_Node/SW4STM32/startup_stm32l072xx.s
	arm-none-eabi-gcc -mcpu=cortex-m0plus -g3 -c -x assembler-with-cpp --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

