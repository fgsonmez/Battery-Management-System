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
BMS_Prototip.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
BMS_Prototip.list \

OBJCOPY_BIN += \
BMS_Prototip.bin \


# All Target
all: BMS_Prototip.elf secondary-outputs

# Tool invocations
BMS_Prototip.elf: $(OBJS) $(USER_OBJS) C:\Users\erenc\OneDrive\Desktop\AST\ESTU_BMS\ESTU_BMS\BMS_Prototip\STM32F4\STM32F407VGTX_FLASH.ld
	arm-none-eabi-gcc -o "BMS_Prototip.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\erenc\OneDrive\Desktop\AST\ESTU_BMS\ESTU_BMS\BMS_Prototip\STM32F4\STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="BMS_Prototip.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

BMS_Prototip.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "BMS_Prototip.list"
	@echo 'Finished building: $@'
	@echo ' '

BMS_Prototip.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "BMS_Prototip.bin"
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
