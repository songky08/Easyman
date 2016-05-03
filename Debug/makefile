################################################################################
# Automatically-generated file. Do not edit!
################################################################################

CG_TOOL_ROOT := /opt/ti/ccsv6/tools/compiler/arm_5.1.6

ORDERED_OBJS += \
$(GEN_CMDS__FLAG) \
"./main.obj" \
"./rom_i2c0_func.obj" \
"./tm4c123gh6pm_startup_ccs.obj" \
"./utils/cmdline.obj" \
"./utils/cpu_usage.obj" \
"./utils/flash_pb.obj" \
"./utils/isqrt.obj" \
"./utils/random.obj" \
"./utils/ringbuf.obj" \
"./utils/scheduler.obj" \
"./utils/sine.obj" \
"./utils/smbus.obj" \
"./utils/softi2c.obj" \
"./utils/softssi.obj" \
"./utils/softuart.obj" \
"./utils/uartstdio.obj" \
"./utils/ustdlib.obj" \
"./driverlib/adc.obj" \
"./driverlib/can.obj" \
"./driverlib/comp.obj" \
"./driverlib/cpu.obj" \
"./driverlib/eeprom.obj" \
"./driverlib/flash.obj" \
"./driverlib/fpu.obj" \
"./driverlib/gpio.obj" \
"./driverlib/hibernate.obj" \
"./driverlib/i2c.obj" \
"./driverlib/interrupt.obj" \
"./driverlib/mpu.obj" \
"./driverlib/pwm.obj" \
"./driverlib/qei.obj" \
"./driverlib/ssi.obj" \
"./driverlib/sw_crc.obj" \
"./driverlib/sysctl.obj" \
"./driverlib/sysexc.obj" \
"./driverlib/systick.obj" \
"./driverlib/timer.obj" \
"./driverlib/uart.obj" \
"./driverlib/udma.obj" \
"./driverlib/usb.obj" \
"./driverlib/watchdog.obj" \
"../tm4c123gh6pm.cmd" \
-l"libc.a" \

-include ../makefile.init

RM := rm -rf
RMDIR := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include utils/subdir_vars.mk
-include driverlib/subdir_vars.mk
-include subdir_rules.mk
-include utils/subdir_rules.mk
-include driverlib/subdir_rules.mk
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
Easyman.out \

EXE_OUTPUTS__QUOTED += \
"Easyman.out" \

BIN_OUTPUTS += \
Easyman.hex \

BIN_OUTPUTS__QUOTED += \
"Easyman.hex" \


# All Target
all: Easyman.out

# Tool invocations
Easyman.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Linker'
	"/opt/ti/ccsv6/tools/compiler/arm_5.1.6/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -g --gcc --define=ccs="ccs" --define=PART_TM4C123GH6PM --display_error_number --diag_warning=225 --diag_wrap=off -z -m"Easyman.map" --heap_size=0 --stack_size=512 -i"/opt/ti/ccsv6/tools/compiler/arm_5.1.6/lib" -i"/opt/ti/ccsv6/tools/compiler/arm_5.1.6/include" --reread_libs --warn_sections --display_error_number --diag_wrap=off --xml_link_info="Easyman_linkInfo.xml" --rom_model -o "Easyman.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

Easyman.hex: $(EXE_OUTPUTS)
	@echo 'Invoking: ARM Hex Utility'
	"/opt/ti/ccsv6/tools/compiler/arm_5.1.6/bin/armhex"  -o "Easyman.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)$(BIN_OUTPUTS__QUOTED)
	-$(RM) "main.pp" "rom_i2c0_func.pp" "tm4c123gh6pm_startup_ccs.pp" "utils/cmdline.pp" "utils/cpu_usage.pp" "utils/flash_pb.pp" "utils/isqrt.pp" "utils/random.pp" "utils/ringbuf.pp" "utils/scheduler.pp" "utils/sine.pp" "utils/smbus.pp" "utils/softi2c.pp" "utils/softssi.pp" "utils/softuart.pp" "utils/uartstdio.pp" "utils/ustdlib.pp" "driverlib/adc.pp" "driverlib/can.pp" "driverlib/comp.pp" "driverlib/cpu.pp" "driverlib/eeprom.pp" "driverlib/flash.pp" "driverlib/fpu.pp" "driverlib/gpio.pp" "driverlib/hibernate.pp" "driverlib/i2c.pp" "driverlib/interrupt.pp" "driverlib/mpu.pp" "driverlib/pwm.pp" "driverlib/qei.pp" "driverlib/ssi.pp" "driverlib/sw_crc.pp" "driverlib/sysctl.pp" "driverlib/sysexc.pp" "driverlib/systick.pp" "driverlib/timer.pp" "driverlib/uart.pp" "driverlib/udma.pp" "driverlib/usb.pp" "driverlib/watchdog.pp" 
	-$(RM) "main.obj" "rom_i2c0_func.obj" "tm4c123gh6pm_startup_ccs.obj" "utils/cmdline.obj" "utils/cpu_usage.obj" "utils/flash_pb.obj" "utils/isqrt.obj" "utils/random.obj" "utils/ringbuf.obj" "utils/scheduler.obj" "utils/sine.obj" "utils/smbus.obj" "utils/softi2c.obj" "utils/softssi.obj" "utils/softuart.obj" "utils/uartstdio.obj" "utils/ustdlib.obj" "driverlib/adc.obj" "driverlib/can.obj" "driverlib/comp.obj" "driverlib/cpu.obj" "driverlib/eeprom.obj" "driverlib/flash.obj" "driverlib/fpu.obj" "driverlib/gpio.obj" "driverlib/hibernate.obj" "driverlib/i2c.obj" "driverlib/interrupt.obj" "driverlib/mpu.obj" "driverlib/pwm.obj" "driverlib/qei.obj" "driverlib/ssi.obj" "driverlib/sw_crc.obj" "driverlib/sysctl.obj" "driverlib/sysexc.obj" "driverlib/systick.obj" "driverlib/timer.obj" "driverlib/uart.obj" "driverlib/udma.obj" "driverlib/usb.obj" "driverlib/watchdog.obj" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

