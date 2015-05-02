################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Descriptors.c \
../Random.c \
../VirtualSerial.c 

O_SRCS += \
../Descriptors.o \
../VirtualSerial.o 

OBJS += \
./Descriptors.o \
./Random.o \
./VirtualSerial.o 

C_DEPS += \
./Descriptors.d \
./Random.d \
./VirtualSerial.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -I"C:\Users\jackc\Documents\lufa-LUFA-140928" -DARCH=AVR8 -DBOARD=BOARD_MINIMUS -DF_USB=16000000 -D"LUFA_PATH=C:\Users\jackc\Documents\lufa-LUFA-140928" -DFIXED_CONTROL_ENDPOINT_SIZE=8 -DFIXED_NUM_CONFIGURATIONS=1 -DUSE_FLASH_DESCRIPTORS -DUSE_STATIC_OPTIONS -UF_CPU -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega32u2 -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


