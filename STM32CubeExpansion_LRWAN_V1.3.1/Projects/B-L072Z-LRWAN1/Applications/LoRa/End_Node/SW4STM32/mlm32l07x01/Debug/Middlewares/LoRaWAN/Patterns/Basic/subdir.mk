################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/project/TESA-Top-Gun-Rally-2020/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Patterns/Basic/lora-test.c \
D:/project/TESA-Top-Gun-Rally-2020/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Patterns/Basic/lora.c 

OBJS += \
./Middlewares/LoRaWAN/Patterns/Basic/lora-test.o \
./Middlewares/LoRaWAN/Patterns/Basic/lora.o 

C_DEPS += \
./Middlewares/LoRaWAN/Patterns/Basic/lora-test.d \
./Middlewares/LoRaWAN/Patterns/Basic/lora.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/LoRaWAN/Patterns/Basic/lora-test.o: D:/project/TESA-Top-Gun-Rally-2020/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Patterns/Basic/lora-test.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DSTM32L072xx -DUSE_B_L072Z_LRWAN1 -DUSE_HAL_DRIVER -DREGION_AS923 -c -I../../../Core/inc -I../../../LoRaWAN/App/inc -I../../../../../../../../Drivers/BSP/CMWX1ZZABZ-0xx -I../../../../../../../../Drivers/STM32L0xx_HAL_Driver/Inc -I../../../../../../../../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../../../../../../../../Drivers/CMSIS/Include -I../../../../../../../../Middlewares/Third_Party/LoRaWAN/Crypto -I../../../../../../../../Middlewares/Third_Party/LoRaWAN/Mac -I../../../../../../../../Middlewares/Third_Party/LoRaWAN/Phy -I../../../../../../../../Middlewares/Third_Party/LoRaWAN/Utilities -I../../../../../../../../Middlewares/Third_Party/LoRaWAN/Patterns/Basic -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../../Drivers/BSP/Components/lps25hb -I../../../../../../../../Drivers/BSP/Components/sx1276 -I../../../../../../../../Drivers/BSP/X_NUCLEO_IKS01A1 -I../../../../../../../../Drivers/BSP/X_NUCLEO_IKS01A2 -I../../../../../../../../Drivers/BSP/B-L072Z-LRWAN1 -I../../../../../../../../Middlewares/Third_Party/LoRaWAN/Mac/region -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/LoRaWAN/Patterns/Basic/lora-test.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Middlewares/LoRaWAN/Patterns/Basic/lora.o: D:/project/TESA-Top-Gun-Rally-2020/STM32CubeExpansion_LRWAN_V1.3.1/Middlewares/Third_Party/LoRaWAN/Patterns/Basic/lora.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DSTM32L072xx -DUSE_B_L072Z_LRWAN1 -DUSE_HAL_DRIVER -DREGION_AS923 -c -I../../../Core/inc -I../../../LoRaWAN/App/inc -I../../../../../../../../Drivers/BSP/CMWX1ZZABZ-0xx -I../../../../../../../../Drivers/STM32L0xx_HAL_Driver/Inc -I../../../../../../../../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../../../../../../../../Drivers/CMSIS/Include -I../../../../../../../../Middlewares/Third_Party/LoRaWAN/Crypto -I../../../../../../../../Middlewares/Third_Party/LoRaWAN/Mac -I../../../../../../../../Middlewares/Third_Party/LoRaWAN/Phy -I../../../../../../../../Middlewares/Third_Party/LoRaWAN/Utilities -I../../../../../../../../Middlewares/Third_Party/LoRaWAN/Patterns/Basic -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../../Drivers/BSP/Components/lps25hb -I../../../../../../../../Drivers/BSP/Components/sx1276 -I../../../../../../../../Drivers/BSP/X_NUCLEO_IKS01A1 -I../../../../../../../../Drivers/BSP/X_NUCLEO_IKS01A2 -I../../../../../../../../Drivers/BSP/B-L072Z-LRWAN1 -I../../../../../../../../Middlewares/Third_Party/LoRaWAN/Mac/region -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/LoRaWAN/Patterns/Basic/lora.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

