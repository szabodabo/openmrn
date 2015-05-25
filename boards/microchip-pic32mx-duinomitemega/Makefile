APP_PATH ?= $(realpath ../..)
-include $(APP_PATH)/config.mk
export TARGET := freertos.mips4k.pic32mx

OBJEXTRA = $(OPENMRNPATH)/targets/freertos.mips4k.pic32mx/freertos_drivers/drivers_pic32mx795/Pic32mxCan.o \
	$(OPENMRNPATH)/targets/freertos.mips4k.pic32mx/freertos_drivers/drivers_pic32mx795/p32mx795f512h.o \

SYSLIBRARIESEXTRA = -lfreertos_drivers_plib_pic32mx795

include $(OPENMRNPATH)/etc/prog.mk

