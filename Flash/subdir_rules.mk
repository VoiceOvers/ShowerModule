################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
ShowerModule.obj: ../ShowerModule.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"c:/ti/ccsv6/tools/compiler/c2000_6.2.8/bin/cl2000" -v28 -ml -mt -O0 -g --include_path="c:/ti/ccsv6/tools/compiler/c2000_6.2.8/include" --include_path="C:/ti/controlSUITE/development_kits/C2000_LaunchPad" --include_path="C:/ti/xdais_7_24_00_04/xdais_7_24_00_04/packages/ti/xdais" --define="_FLASH" --define=NDEBUG --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --gen_func_subsections=on --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="ShowerModule.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


