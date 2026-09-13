################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/angle_controller.c \
../Core/Src/dshot.c \
../Core/Src/e22.c \
../Core/Src/gps.c \
../Core/Src/icm42670.c \
../Core/Src/mahony.c \
../Core/Src/main.c \
../Core/Src/mixer.c \
../Core/Src/mmc5983ma.c \
../Core/Src/pid.c \
../Core/Src/rate_controller.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c 

OBJS += \
./Core/Src/angle_controller.o \
./Core/Src/dshot.o \
./Core/Src/e22.o \
./Core/Src/gps.o \
./Core/Src/icm42670.o \
./Core/Src/mahony.o \
./Core/Src/main.o \
./Core/Src/mixer.o \
./Core/Src/mmc5983ma.o \
./Core/Src/pid.o \
./Core/Src/rate_controller.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o 

C_DEPS += \
./Core/Src/angle_controller.d \
./Core/Src/dshot.d \
./Core/Src/e22.d \
./Core/Src/gps.d \
./Core/Src/icm42670.d \
./Core/Src/mahony.d \
./Core/Src/main.d \
./Core/Src/mixer.d \
./Core/Src/mmc5983ma.d \
./Core/Src/pid.d \
./Core/Src/rate_controller.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/angle_controller.cyclo ./Core/Src/angle_controller.d ./Core/Src/angle_controller.o ./Core/Src/angle_controller.su ./Core/Src/dshot.cyclo ./Core/Src/dshot.d ./Core/Src/dshot.o ./Core/Src/dshot.su ./Core/Src/e22.cyclo ./Core/Src/e22.d ./Core/Src/e22.o ./Core/Src/e22.su ./Core/Src/gps.cyclo ./Core/Src/gps.d ./Core/Src/gps.o ./Core/Src/gps.su ./Core/Src/icm42670.cyclo ./Core/Src/icm42670.d ./Core/Src/icm42670.o ./Core/Src/icm42670.su ./Core/Src/mahony.cyclo ./Core/Src/mahony.d ./Core/Src/mahony.o ./Core/Src/mahony.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mixer.cyclo ./Core/Src/mixer.d ./Core/Src/mixer.o ./Core/Src/mixer.su ./Core/Src/mmc5983ma.cyclo ./Core/Src/mmc5983ma.d ./Core/Src/mmc5983ma.o ./Core/Src/mmc5983ma.su ./Core/Src/pid.cyclo ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/rate_controller.cyclo ./Core/Src/rate_controller.d ./Core/Src/rate_controller.o ./Core/Src/rate_controller.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su

.PHONY: clean-Core-2f-Src

