# SPDX-License-Identifier: Apache-2.0
# micro T-Kernel 3.0 (TRON Forum) + BSP2 for STM32Cube / STM32N6570-DK
#
# Replaces mks/freertos.mk. Two vendored trees:
#   ThirdParty/mtk3_bsp2/mtkernel  - the kernel itself           (T-License 2.2)
#   ThirdParty/mtk3_bsp2           - board support for STM32Cube (T-License 2.2)
#
# BSP2 supersedes the kernel's own sys/machine.h, so its include directory
# MUST come first. Getting this order wrong silently selects the wrong target.

# BSP2 includes the kernel as <mtkernel/...>, so mtkernel_3 must live INSIDE
# mtk3_bsp2 as mtkernel/ (upstream has it as a git submodule at that path).
BSP2_REL_DIR := $(TP_DIR)/mtk3_bsp2
MTK_REL_DIR  := $(BSP2_REL_DIR)/mtkernel

# Board selection, consumed by mtk3_bsp2/include/sys/machine.h
C_DEFS_MTK += -D_STM32CUBE_DISCOVERY_N657_

# Make the kernel's entry point knl_main() instead of main(), so the ST
# application keeps its own main() and calls knl_start_mtkernel() (through
# os_start() in Platform/Src/os_utk.c) once the hardware is up. Without this
# the kernel defines main() too and the link fails with a duplicate symbol.
C_DEFS_MTK += -DADD_PREFIX_MAIN_FUNC=1

# Port bring-up tracing, off by default: `make UTK_TRACE=1` to enable. It turns
# on the camera start-up markers in App/Src/app_cam.c and the 2 s idle-task
# heartbeat in Platform/Src/utk_bsp.c, printed over the console UART by kdbg()
# in Platform/Src/main.c. It also defines MTK_TRACE, which only matters in a
# kernel patched with trace points; the upstream kernel in ThirdParty/mtk3_bsp2
# has none, so here it does nothing.
UTK_TRACE ?= 0
C_DEFS_MTK += -DUTK_TRACE=$(UTK_TRACE)
ifeq ($(UTK_TRACE),1)
C_DEFS_MTK += -DMTK_TRACE
endif

# ---- kernel ---------------------------------------------------------------
C_SOURCES_MTK += $(wildcard $(MTK_REL_DIR)/kernel/tkernel/*.c)
C_SOURCES_MTK += $(wildcard $(MTK_REL_DIR)/kernel/tstdlib/*.c)
C_SOURCES_MTK += $(MTK_REL_DIR)/kernel/sysinit/sysinit.c
C_SOURCES_MTK += $(MTK_REL_DIR)/kernel/inittask/inittask.c
C_SOURCES_MTK += $(wildcard $(MTK_REL_DIR)/lib/libtk/*.c)
C_SOURCES_MTK += $(wildcard $(MTK_REL_DIR)/lib/libtm/*.c)
# usermain.c is deliberately omitted: Platform/Src/main.c provides usermain().

# ---- board support (STM32Cube group + ARMv8-M core) -----------------------
C_SOURCES_MTK += $(BSP2_REL_DIR)/sysdepend/stm32_cube/devinit.c
C_SOURCES_MTK += $(BSP2_REL_DIR)/sysdepend/stm32_cube/hw_setting.c
C_SOURCES_MTK += $(BSP2_REL_DIR)/sysdepend/stm32_cube/power_save.c
C_SOURCES_MTK += $(BSP2_REL_DIR)/sysdepend/stm32_cube/cpu/core/armv8m/cpu_cntl.c
C_SOURCES_MTK += $(BSP2_REL_DIR)/sysdepend/stm32_cube/cpu/core/armv8m/exc_hdr.c
C_SOURCES_MTK += $(BSP2_REL_DIR)/sysdepend/stm32_cube/cpu/core/armv8m/interrupt.c
C_SOURCES_MTK += $(BSP2_REL_DIR)/sysdepend/stm32_cube/cpu/core/armv8m/sys_start.c
# libtk/libtm are nested per core and per SoC -- a top-level wildcard silently
# matches nothing and the link then fails on disint/set_basepri/get_basepri.
C_SOURCES_MTK += $(wildcard $(BSP2_REL_DIR)/sysdepend/stm32_cube/lib/libtk/cpu/core/armv8m/*.c)
C_SOURCES_MTK += $(wildcard $(BSP2_REL_DIR)/sysdepend/stm32_cube/lib/libtk/cpu/stm32n6/*.c)
C_SOURCES_MTK += $(BSP2_REL_DIR)/sysdepend/stm32_cube/lib/libtm/discovery_stm32n657/tm_com.c

# The context switch. .S (capital) so the preprocessor runs -- it includes
# sysdef.h for TRUSTZONE_ENABLE and the FPU flags.
ASM_SOURCES_S_MTK += $(BSP2_REL_DIR)/sysdepend/stm32_cube/cpu/core/armv8m/dispatch.S

# ---- includes: BSP2 first ------------------------------------------------
C_INCLUDES_MTK += -I$(BSP2_REL_DIR)/include
C_INCLUDES_MTK += -I$(BSP2_REL_DIR)/config
C_INCLUDES_MTK += -I$(BSP2_REL_DIR)
C_INCLUDES_MTK += -I$(MTK_REL_DIR)/include
C_INCLUDES_MTK += -I$(MTK_REL_DIR)/config
C_INCLUDES_MTK += -I$(MTK_REL_DIR)/kernel/knlinc
C_INCLUDES_MTK += -I$(MTK_REL_DIR)

C_SOURCES    += $(C_SOURCES_MTK)
ASM_SOURCES_S += $(ASM_SOURCES_S_MTK)
C_INCLUDES   += $(C_INCLUDES_MTK)
C_DEFS       += $(C_DEFS_MTK)
