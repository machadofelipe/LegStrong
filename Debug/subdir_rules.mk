################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
main.obj: ../main.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -ml -mt -O2 --opt_for_speed=3 --include_path="C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="D:/Users/MaCHaDo/Dropbox/Legstrong/drv8301/legstrong/LegStrong/drivers/include" --include_path="D:/Users/MaCHaDo/Dropbox/Legstrong/drv8301/legstrong/LegStrong" --include_path="D:/Users/MaCHaDo/Dropbox/Legstrong/drv8301/legstrong/LegStrong/modules/include" --advice:performance=all -g --define=FLASH --define=FAST_ROM_V1p7 --define=F2802xF --display_error_number --diag_warning=225 --asm_listing --preproc_with_compile --preproc_dependency="main.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


