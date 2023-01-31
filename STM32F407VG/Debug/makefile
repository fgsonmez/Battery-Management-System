################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
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
ESTU_BMS.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
ESTU_BMS.list \

OBJCOPY_BIN += \
ESTU_BMS.bin \


# All Target
all: ESTU_BMS.elf secondary-outputs

# Tool invocations
ESTU_BMS.elf: $(OBJS) $(USER_OBJS) C:\Users\Fahrettin\ GÜLTEKİN\Desktop\ESTU\ BMS\ GITHUB\ESTU_BMS\STM32F407VG\STM32F407VGTX_FLASH.ld
	arm-none-eabi-gcc -o "ESTU_BMS.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\Fahrettin GÜLTEKİN\Desktop\ESTU BMS GITHUB\ESTU_BMS\STM32F407VG\STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="ESTU_BMS.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

ESTU_BMS.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "ESTU_BMS.list"
	@echo 'Finished building: $@'
	@echo ' '

ESTU_BMS.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "ESTU_BMS.bin"
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
