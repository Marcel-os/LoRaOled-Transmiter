################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
LoRaOled-Transmiter.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
LoRaOled-Transmiter.list \

OBJCOPY_BIN += \
LoRaOled-Transmiter.bin \


# All Target
all: LoRaOled-Transmiter.elf secondary-outputs

# Tool invocations
LoRaOled-Transmiter.elf: $(OBJS) $(USER_OBJS) /Users/marceldomagala/Documents/STM32-Projects/LoRaOled-Transmiter/STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "LoRaOled-Transmiter.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"/Users/marceldomagala/Documents/STM32-Projects/LoRaOled-Transmiter/STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="LoRaOled-Transmiter.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

LoRaOled-Transmiter.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "LoRaOled-Transmiter.list"
	@echo 'Finished building: $@'
	@echo ' '

LoRaOled-Transmiter.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "LoRaOled-Transmiter.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
