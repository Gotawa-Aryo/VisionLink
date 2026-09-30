C_SOURCES_GCC += Platform/Gcc/console.c
C_SOURCES_GCC += Platform/Gcc/utk_libc.c
C_SOURCES_GCC += Platform/Gcc/syscalls.c

ASM_SOURCES_GCC += $(FW_REL_DIR)/Drivers/CMSIS/Device/ST/STM32N6xx/Source/Templates/gcc/startup_stm32n657xx.s

C_SOURCES += $(C_SOURCES_GCC)
ASM_SOURCES += $(ASM_SOURCES_GCC)
