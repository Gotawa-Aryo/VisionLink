NEMA_REL_DIR := $(TP_DIR)/NemaGFX

C_SOURCES_NEMA += Platform/Src/nema_hal_utkernel.c

C_INCLUDES_NEMA += -I$(NEMA_REL_DIR)/include

C_SOURCES += $(C_SOURCES_NEMA)
C_INCLUDES += $(C_INCLUDES_NEMA)

LIBDIR += -L$(NEMA_REL_DIR)/lib/core/cortex_m55/gcc
LIBS += -lnemagfx-float-abi-hard
