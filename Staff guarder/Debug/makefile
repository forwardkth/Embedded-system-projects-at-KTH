################################################################################
# Automatically-generated file. Do not edit!
################################################################################

EMPTY := 
SPACE := $(EMPTY) $(EMPTY)

ORDERED_OBJS += \
$(GEN_CMDS_QUOTED) \
"./wasa_1_5_at.obj" \
-l"libc.a" \
"../lnk_msp430f2416.cmd" \

-include ../makefile.init

RM := DEL /F

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: guarder.out

# Tool invocations
guarder.out: $(GEN_CMDS) $(CMD_SRCS) $(OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Linker'
	"C:/Program Files/Texas Instruments/ccsv4/tools/compiler/msp430/bin/cl430" --silicon_version=mspx -g --diag_warning=225 --printf_support=minimal -z -m"guarder.map" --stack_size=80 --heap_size=80 --use_hw_mpy=16 --warn_sections -i"C:/Program Files/Texas Instruments/ccsv4/msp430/include" -i"C:/Program Files/Texas Instruments/ccsv4/tools/compiler/msp430/lib" -i"C:/Program Files/Texas Instruments/ccsv4/tools/compiler/msp430/include" --reread_libs --rom_model -o "guarder.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(S62_DEPS__QTD)$(OBJS__QTD)$(C64_DEPS__QTD)$(ASM_DEPS__QTD)$(C62_DEPS__QTD)$(C_UPPER_DEPS__QTD)$(S??_DEPS__QTD)$(S55_DEPS__QTD)$(S64_DEPS__QTD)$(C55_DEPS__QTD)$(C67_DEPS__QTD)$(CPP_DEPS__QTD)$(C43_DEPS__QTD)$(C_DEPS__QTD)$(S43_DEPS__QTD)$(CC_DEPS__QTD)$(MSP430_EXECUTABLE_OUTPUTS__QTD)$(S67_DEPS__QTD)$(OPT_DEPS__QTD)$(S_UPPER_DEPS__QTD)$(C??_DEPS__QTD)$(CXX_DEPS__QTD)$(C++_DEPS__QTD)$(S_DEPS__QTD)$(SA_DEPS__QTD)$(ASM_UPPER_DEPS__QTD) "guarder.out"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
