################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
MCAL/ADC/%.obj: ../MCAL/ADC/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/MYCOMPUTER/NEWEST CCS/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="C:/MYCOMPUTER/Advanced_MT_Embedded Systems Diploma/RTOS/FreeRTOS_Project 2/FreeRTOS_Project" --include_path="C:/MYCOMPUTER/Advanced_MT_Embedded Systems Diploma/RTOS/FreeRTOS_Project 2/FreeRTOS_Project/MCAL/GPTM" --include_path="C:/MYCOMPUTER/Advanced_MT_Embedded Systems Diploma/RTOS/FreeRTOS_Project 2/FreeRTOS_Project/Common" --include_path="C:/MYCOMPUTER/Advanced_MT_Embedded Systems Diploma/RTOS/FreeRTOS_Project 2/FreeRTOS_Project/MCAL" --include_path="C:/MYCOMPUTER/Advanced_MT_Embedded Systems Diploma/RTOS/FreeRTOS_Project 2/FreeRTOS_Project/MCAL/GPIO" --include_path="C:/MYCOMPUTER/Advanced_MT_Embedded Systems Diploma/RTOS/FreeRTOS_Project 2/FreeRTOS_Project/MCAL/UART" --include_path="C:/MYCOMPUTER/Advanced_MT_Embedded Systems Diploma/RTOS/FreeRTOS_Project 2/FreeRTOS_Project/FreeRTOS/Source/include" --include_path="C:/MYCOMPUTER/Advanced_MT_Embedded Systems Diploma/RTOS/FreeRTOS_Project 2/FreeRTOS_Project/FreeRTOS/Source/portable/CCS/ARM_CM4F" --include_path="C:/MYCOMPUTER/NEWEST CCS/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/include" --define=ccs="ccs" --define=PART_TM4C123GH6PM -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="MCAL/ADC/$(basename $(<F)).d_raw" --obj_directory="MCAL/ADC" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


