# SPDX-License-Identifier: Apache-2.0
# USBX device stack in standalone mode + HID class, for the USB gamepad on
# CN18 (USB1 OTG HS). Same source selection as the UVC library's
# uvc_lib_usbx.mk, with the video class swapped for HID. USBX is driven from a
# micro T-Kernel task (App/Src/usb_hid.c), not from ThreadX.
USBX_REL_DIR := $(TP_DIR)/STM32Cube_FW_N6/Middlewares/ST/usbx

C_SOURCES_USBX += $(wildcard $(USBX_REL_DIR)/common/core/src/ux_system_*.c)
C_SOURCES_USBX += $(wildcard $(USBX_REL_DIR)/common/core/src/ux_utility_*.c)
C_SOURCES_USBX += $(wildcard $(USBX_REL_DIR)/common/core/src/ux_device_*.c)
C_SOURCES_USBX += $(wildcard $(USBX_REL_DIR)/common/usbx_stm32_device_controllers/ux_dcd_stm32_*.c)
C_SOURCES_USBX += $(wildcard $(USBX_REL_DIR)/common/usbx_device_classes/src/ux_device_class_hid_*.c)
C_SOURCES_USBX += App/Src/usb_hid.c

C_INCLUDES_USBX += -I$(USBX_REL_DIR)/common/core/inc
C_INCLUDES_USBX += -I$(USBX_REL_DIR)/ports/generic/inc
C_INCLUDES_USBX += -I$(USBX_REL_DIR)/common/usbx_device_classes/inc
C_INCLUDES_USBX += -I$(USBX_REL_DIR)/common/usbx_stm32_device_controllers
# ux_user.h and ux_stm32_config.h are in Platform/Inc, already on the path.

C_DEFS_USBX += -DUX_INCLUDE_USER_DEFINE_FILE
C_DEFS_USBX += -DUX_STANDALONE
C_DEFS_USBX += -DUSE_USB_HID_GAMEPAD=1

C_SOURCES += $(C_SOURCES_USBX)
C_INCLUDES += $(C_INCLUDES_USBX)
C_DEFS += $(C_DEFS_USBX)
