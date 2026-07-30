################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../SOES/stack/hal/esc_hw.c 

OBJS += \
./SOES/stack/hal/esc_hw.o 

C_DEPS += \
./SOES/stack/hal/esc_hw.d 


# Each subdirectory must supply rules for building sources it contributes
SOES/stack/hal/%.o SOES/stack/hal/%.su SOES/stack/hal/%.cyclo: ../SOES/stack/hal/%.c SOES/stack/hal/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I"C:/Users/SHJ/Desktop/HD_hand/fw/hand_ctl/SOES/SSC" -I"C:/Users/SHJ/Desktop/HD_hand/fw/hand_ctl/SOES/stack" -I"C:/Users/SHJ/Desktop/HD_hand/fw/hand_ctl/SOES/stack/hal" -I"C:/Users/SHJ/Desktop/HD_hand/fw/hand_ctl/SOES/stack/include/sys/gcc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-SOES-2f-stack-2f-hal

clean-SOES-2f-stack-2f-hal:
	-$(RM) ./SOES/stack/hal/esc_hw.cyclo ./SOES/stack/hal/esc_hw.d ./SOES/stack/hal/esc_hw.o ./SOES/stack/hal/esc_hw.su

.PHONY: clean-SOES-2f-stack-2f-hal

