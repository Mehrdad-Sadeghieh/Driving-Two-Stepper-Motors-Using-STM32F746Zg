################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F7xx_HAL_Driver/Src/subdir.mk
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
Steper_motor\ 2.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Steper_motor\ 2.list \

OBJCOPY_BIN += \
Steper_motor\ 2.bin \


# All Target
all: Steper_motor\ 2.elf secondary-outputs

# Tool invocations
Steper_motor\ 2.elf: $(OBJS) $(USER_OBJS) S:\CUBE\ project\Steper_motor\ 2\Steper_motor\ 2\STM32F746ZGTX_FLASH.ld
	arm-none-eabi-gcc -o "Steper_motor 2.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"S:\CUBE project\Steper_motor 2\Steper_motor 2\STM32F746ZGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Steper_motor 2.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Steper_motor\ 2.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Steper_motor 2.list"
	@echo 'Finished building: $@'
	@echo ' '

Steper_motor\ 2.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Steper_motor 2.bin"
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
