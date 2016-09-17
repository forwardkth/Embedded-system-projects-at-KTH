################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../wasa_1_5_at.c 

CMD_SRCS += \
../lnk_msp430f2416.cmd 

OBJS += \
./wasa_1_5_at.obj 

C_DEPS += \
./wasa_1_5_at.pp 

OBJS__QTD += \
".\wasa_1_5_at.obj" 

C_DEPS__QTD += \
".\wasa_1_5_at.pp" 

C_SRCS_QUOTED += \
"../wasa_1_5_at.c" 


# Each subdirectory must supply rules for building sources it contributes
wasa_1_5_at.obj: ../wasa_1_5_at.c $(GEN_SRCS) $(GEN_OPTS)
	@echo 'Building file: $<'
	@echo 'Invoking: Compiler'
	"C:/Program Files/Texas Instruments/ccsv4/tools/compiler/msp430/bin/cl430" --silicon_version=mspx -g --include_path="C:/Program Files/Texas Instruments/ccsv4/msp430/include" --include_path="C:/Program Files/Texas Instruments/ccsv4/tools/compiler/msp430/include" --diag_warning=225 --printf_support=minimal --preproc_with_compile --preproc_dependency="wasa_1_5_at.pp" $(GEN_OPTS_QUOTED) $(subst #,$(wildcard $(subst $(SPACE),\$(SPACE),$<)),"#")
	@echo 'Finished building: $<'
	@echo ' '


