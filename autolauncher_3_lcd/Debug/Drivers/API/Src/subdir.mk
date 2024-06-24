################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/API/Src/API_lcd_i2c.c 

OBJS += \
./Drivers/API/Src/API_lcd_i2c.o 

C_DEPS += \
./Drivers/API/Src/API_lcd_i2c.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/API/Src/%.o Drivers/API/Src/%.su Drivers/API/Src/%.cyclo: ../Drivers/API/Src/%.c Drivers/API/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/christian.saiz/STM32CubeIDE/workspace_1.13.1/autolauncher_3_lcd/Drivers/API" -I"C:/Users/christian.saiz/STM32CubeIDE/workspace_1.13.1/autolauncher_3_lcd/Drivers/API/Inc" -I"C:/Users/christian.saiz/STM32CubeIDE/workspace_1.13.1/autolauncher_3_lcd/Drivers/API/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-API-2f-Src

clean-Drivers-2f-API-2f-Src:
	-$(RM) ./Drivers/API/Src/API_lcd_i2c.cyclo ./Drivers/API/Src/API_lcd_i2c.d ./Drivers/API/Src/API_lcd_i2c.o ./Drivers/API/Src/API_lcd_i2c.su

.PHONY: clean-Drivers-2f-API-2f-Src

