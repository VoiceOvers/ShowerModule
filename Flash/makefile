################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := c:/ti/ccsv6/tools/compiler/c2000_6.2.9

ORDERED_OBJS += \
$(GEN_CMDS__FLAG) \
"./ShowerModule.obj" \
"C:/ti/controlSUITE/development_kits/C2000_LaunchPad/f2802x_headers/cmd/F2802x_Headers_nonBIOS.cmd" \
"C:/ti/controlSUITE/development_kits/C2000_LaunchPad/f2802x_common/lib/driverlib.lib" \
-l"C:/ti/controlSUITE/development_kits/C2000_LaunchPad/f2802x_examples/C2kLaunchPadDemo/../../f2802x_common/cmd/F28027.cmd" \
-l"rts2800_ml.lib" \
-l"IQmath.lib" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
Example_F2802xLaunchPad_demo.out \

EXE_OUTPUTS__QUOTED += \
"Example_F2802xLaunchPad_demo.out" \


# All Target
all: Example_F2802xLaunchPad_demo.out

# Tool invocations
Example_F2802xLaunchPad_demo.out: $(OBJS) $(CMD_SRCS) $(LIB_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: C2000 Linker'
	"c:/ti/ccsv6/tools/compiler/c2000_6.2.9/bin/cl2000" -v28 -ml -mt -O0 -g --define="_FLASH" --define=NDEBUG --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --gen_func_subsections=on --output_all_syms --cdebug_asm_data -z --stack_size=0x300 -m"Example_F2802xLaunchPad_demo.map" --heap_size=0x200 --warn_sections -i"c:/ti/ccsv6/tools/compiler/c2000_6.2.9/lib" -i"c:/ti/ccsv6/tools/compiler/c2000_6.2.9/include" -i"C:/Users/Edward/Documents/CodeComposerWorkspace_v6_0/ShowerModule" -i"C:/ti/controlSUITE/development_kits/C2000_LaunchPad/F2802x_common/lib" --reread_libs --verbose_diagnostics --issue_remarks --xml_link_info="Example_F2802xLaunchPad_demo_linkInfo.xml" --entry_point=code_start --rom_model -o "Example_F2802xLaunchPad_demo.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)
	-$(RM) "ShowerModule.pp" 
	-$(RM) "ShowerModule.obj" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

