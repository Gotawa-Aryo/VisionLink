# SPDX-License-Identifier: Apache-2.0
# Touchscreen: GT911 controller behind the BSP TS driver.
#
# The vendored STM32Cube_FW_N6 tree is a trimmed copy with no Components/gt911,
# so the matching driver set is vendored separately in ThirdParty/touchscreen,
# taken byte for byte (modulo line endings) from STM32Cube_FW_N6 V1.2.0.
#
# stm32n6570_discovery_ts.h includes "../Components/gt911/gt911.h", which is
# why the two directories mirror the BSP layout. The include paths are added in
# the top-level Makefile BEFORE mks/fw.mk, so this copy of
# stm32n6570_discovery_ts.h wins over STM32Cube_FW_N6's, which would try to
# reach a Components/gt911 that is not there.
TS_REL_DIR := $(TP_DIR)/touchscreen

C_SOURCES_TS += $(TS_REL_DIR)/STM32N6570-DK/stm32n6570_discovery_ts.c
C_SOURCES_TS += $(TS_REL_DIR)/Components/gt911/gt911.c
C_SOURCES_TS += $(TS_REL_DIR)/Components/gt911/gt911_reg.c

C_SOURCES += $(C_SOURCES_TS)
