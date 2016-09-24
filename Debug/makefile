################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"../modules/lib/2802xRevB_Fast_IQ_ROMSymbols.lib" \
"../modules/lib/2802xRevB_Fast_ONLY_ROMSymbols.lib" \
"../modules/lib/2802xRevB_Fast_RTS_ROMSymbols.lib" \
-lrts2800_ml.lib \
"../modules/lib/fast_public.lib" \
"../modules/lib/IQmath.lib" \
"./main.obj" \
"./drivers/source/CodeStartBranch.obj" \
"./drivers/source/adc.obj" \
"./drivers/source/cap.obj" \
"./drivers/source/clk.obj" \
"./drivers/source/comp.obj" \
"./drivers/source/cpu.obj" \
"./drivers/source/drv8301.obj" \
"./drivers/source/flash.obj" \
"./drivers/source/gpio.obj" \
"./drivers/source/osc.obj" \
"./drivers/source/pie.obj" \
"./drivers/source/pll.obj" \
"./drivers/source/pwm.obj" \
"./drivers/source/pwr.obj" \
"./drivers/source/sci.obj" \
"./drivers/source/spi.obj" \
"./drivers/source/timer.obj" \
"./drivers/source/wdog.obj" \
"./modules/source/clarke.obj" \
"./modules/source/ctrl.obj" \
"./modules/source/filter_fo.obj" \
"./modules/source/hal.obj" \
"./modules/source/ipark.obj" \
"./modules/source/memCopy.obj" \
"./modules/source/offset.obj" \
"./modules/source/park.obj" \
"./modules/source/pid.obj" \
"./modules/source/svgen.obj" \
"./modules/source/traj.obj" \
"./modules/source/usDelay.obj" \
"./modules/source/user.obj" \
"../F28027F.cmd" \
$(GEN_CMDS__FLAG) \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include drivers/source/subdir_vars.mk
-include modules/include/subdir_vars.mk
-include modules/lib/subdir_vars.mk
-include modules/source/subdir_vars.mk
-include subdir_rules.mk
-include drivers/source/subdir_rules.mk
-include modules/include/subdir_rules.mk
-include modules/lib/subdir_rules.mk
-include modules/source/subdir_rules.mk
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
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
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
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
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
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
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
LegStrong.out \

EXE_OUTPUTS__QUOTED += \
"LegStrong.out" \

BIN_OUTPUTS += \
LegStrong.hex \

BIN_OUTPUTS__QUOTED += \
"LegStrong.hex" \


# All Target
all: LegStrong.out

# Tool invocations
LegStrong.out: $(OBJS) $(CMD_SRCS) $(LIB_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: C2000 Linker'
	"C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -ml -mt -O2 --opt_for_speed=3 --advice:performance=all -g --define=FLASH --define=FAST_ROM_V1p7 --define=F2802xF --display_error_number --diag_warning=225 --asm_listing -z -m"LegStrong.map" --stack_size=0x200 --warn_sections -i"C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/lib" -i"C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --reread_libs --diag_wrap=off --display_error_number --xml_link_info="LegStrong_linkInfo.xml" --rom_model -o "LegStrong.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

LegStrong.hex: $(EXE_OUTPUTS)
	@echo 'Invoking: C2000 Hex Utility'
	"C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/hex2000"  -o "LegStrong.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)$(BIN_OUTPUTS__QUOTED)
	-$(RM) "main.d" "drivers\source\adc.d" "drivers\source\cap.d" "drivers\source\clk.d" "drivers\source\comp.d" "drivers\source\cpu.d" "drivers\source\drv8301.d" "drivers\source\flash.d" "drivers\source\gpio.d" "drivers\source\osc.d" "drivers\source\pie.d" "drivers\source\pll.d" "drivers\source\pwm.d" "drivers\source\pwr.d" "drivers\source\sci.d" "drivers\source\spi.d" "drivers\source\timer.d" "drivers\source\wdog.d" "modules\source\clarke.d" "modules\source\ctrl.d" "modules\source\filter_fo.d" "modules\source\hal.d" "modules\source\ipark.d" "modules\source\memCopy.d" "modules\source\offset.d" "modules\source\park.d" "modules\source\pid.d" "modules\source\svgen.d" "modules\source\traj.d" "modules\source\user.d" 
	-$(RM) "main.obj" "drivers\source\CodeStartBranch.obj" "drivers\source\adc.obj" "drivers\source\cap.obj" "drivers\source\clk.obj" "drivers\source\comp.obj" "drivers\source\cpu.obj" "drivers\source\drv8301.obj" "drivers\source\flash.obj" "drivers\source\gpio.obj" "drivers\source\osc.obj" "drivers\source\pie.obj" "drivers\source\pll.obj" "drivers\source\pwm.obj" "drivers\source\pwr.obj" "drivers\source\sci.obj" "drivers\source\spi.obj" "drivers\source\timer.obj" "drivers\source\wdog.obj" "modules\source\clarke.obj" "modules\source\ctrl.obj" "modules\source\filter_fo.obj" "modules\source\hal.obj" "modules\source\ipark.obj" "modules\source\memCopy.obj" "modules\source\offset.obj" "modules\source\park.obj" "modules\source\pid.obj" "modules\source\svgen.obj" "modules\source\traj.obj" "modules\source\usDelay.obj" "modules\source\user.obj" 
	-$(RM) "drivers\source\CodeStartBranch.d" "modules\source\usDelay.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

