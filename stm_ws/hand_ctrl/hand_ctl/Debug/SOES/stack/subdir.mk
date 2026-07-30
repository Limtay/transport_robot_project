################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../SOES/stack/ecat_slv.c \
../SOES/stack/esc.c \
../SOES/stack/esc_coe.c \
../SOES/stack/esc_eep.c \
../SOES/stack/esc_eoe.c \
../SOES/stack/esc_foe.c 

OBJS += \
./SOES/stack/ecat_slv.o \
./SOES/stack/esc.o \
./SOES/stack/esc_coe.o \
./SOES/stack/esc_eep.o \
./SOES/stack/esc_eoe.o \
./SOES/stack/esc_foe.o 

C_DEPS += \
./SOES/stack/ecat_slv.d \
./SOES/stack/esc.d \
./SOES/stack/esc_coe.d \
./SOES/stack/esc_eep.d \
./SOES/stack/esc_eoe.d \
./SOES/stack/esc_foe.d 


# Each subdirectory must supply rules for building sources it contributes
SOES/stack/%.o SOES/stack/%.su SOES/stack/%.cyclo: ../SOES/stack/%.c SOES/stack/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I"C:/Users/SHJ/Desktop/HD_hand/fw/hand_ctl/SOES/SSC" -I"C:/Users/SHJ/Desktop/HD_hand/fw/hand_ctl/SOES/stack" -I"C:/Users/SHJ/Desktop/HD_hand/fw/hand_ctl/SOES/stack/hal" -I"C:/Users/SHJ/Desktop/HD_hand/fw/hand_ctl/SOES/stack/include/sys/gcc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-SOES-2f-stack

clean-SOES-2f-stack:
	-$(RM) ./SOES/stack/ecat_slv.cyclo ./SOES/stack/ecat_slv.d ./SOES/stack/ecat_slv.o ./SOES/stack/ecat_slv.su ./SOES/stack/esc.cyclo ./SOES/stack/esc.d ./SOES/stack/esc.o ./SOES/stack/esc.su ./SOES/stack/esc_coe.cyclo ./SOES/stack/esc_coe.d ./SOES/stack/esc_coe.o ./SOES/stack/esc_coe.su ./SOES/stack/esc_eep.cyclo ./SOES/stack/esc_eep.d ./SOES/stack/esc_eep.o ./SOES/stack/esc_eep.su ./SOES/stack/esc_eoe.cyclo ./SOES/stack/esc_eoe.d ./SOES/stack/esc_eoe.o ./SOES/stack/esc_eoe.su ./SOES/stack/esc_foe.cyclo ./SOES/stack/esc_foe.d ./SOES/stack/esc_foe.o ./SOES/stack/esc_foe.su

.PHONY: clean-SOES-2f-stack

