APP_PATH ?= $(realpath ../..)
-include $(APP_PATH)/config.mk
-include local_config.mk

OPENMRNPATH ?= $(shell \
sh -c "if [ \"X`printenv OPENMRNPATH`\" != \"X\" ]; then printenv OPENMRNPATH; \
     elif [ -d /opt/openmrn/src ]; then echo /opt/openmrn; \
     elif [ -d ~/openmrn/src ]; then echo ~/openmrn; \
     elif [ -d ../../../src ]; then echo ../../..; \
     else echo OPENMRNPATH not found; fi" \
)

# Find STM32CubeF3 libraries
include $(OPENMRNPATH)/etc/stm32cubef3.mk

LDFLAGSEXTRA += 
SYSLIBRARIESEXTRA += -lfreertos_drivers_stm32cubef303xe
OBJEXTRA += 

CFLAGS += -DSTM32F303xE
CXXFLAGS += -DSTM32F303xE -DNODE_NUMBER=$(NODE_NUMBER)
OPENOCDARGS = -f board/st_nucleo_f3.cfg

export TARGET := freertos.armv7m
include $(OPENMRNPATH)/etc/prog.mk

include extract_node_uuid.mk

ifneq ($(shell cat node_number.output 2>/dev/null),node_number_prefix $(NODE_NUMBER))
LOL := $(shell touch NodeId.cxx)
LOL := $(shell echo 'node_number_prefix $(NODE_NUMBER)' >node_number.output &)
LOL := $(shell echo "Recompiling for NODE_NUMBER $(NODE_NUMBER)" >&2)
endif

# How to use: make multibin ADDRESS=0x20 ADDRHIGH=0x45 NUM=3
# starting address, high bits (user range), count
multibin:
	for i in $$(seq 1 $(NUM)) ; do $(MAKE) $(EXECUTABLE).bin ADDRESS=$$(printf 0x%02x $$(($(ADDRESS)+$$i))) ; cp $(EXECUTABLE).bin $(EXECUTABLE).f303.$$(printf %02x%02x $(ADDRHIGH) $$(($(ADDRESS)+$$i-1))).bin ; done

ifeq ($(call find_missing_deps,OPENOCDPATH OPENOCDSCRIPTSPATH),)
all: $(EXECUTABLE).bin

flash: $(EXECUTABLE)$(EXTENTION) $(EXECUTABLE).lst
	@if ps ax -o comm | grep -q openocd ; then echo openocd already running. quit existing first. ; exit 1 ; fi
	$(GDB) $< -ex "target remote | $(OPENOCDPATH)/openocd -c \"gdb_port pipe\" --search $(OPENOCDSCRIPTSPATH) $(OPENOCDARGS)" -ex "monitor reset halt" -ex "load" -ex "monitor reset init" -ex "monitor reset run"  -ex "detach" -ex "quit"

gdb:
	@if ps ax -o comm | grep -q openocd ; then echo openocd already running. quit existing first. ; exit 1 ; fi
	$(GDB)-py $(EXECUTABLE)$(EXTENTION) -ex "target remote | $(OPENOCDPATH)/openocd -c \"gdb_port pipe\" --search $(OPENOCDSCRIPTSPATH) $(OPENOCDARGS)" -ex "continue" # -ex "monitor reset halt"

gdbserver:
	@if ps ax -o comm | grep -q openocd ; then echo openocd already running. quit existing first. ; exit 1 ; fi
	gdbserver localhost:8000 $(EXECUTABLE)$(EXTENTION) --gdb-args "-ex 'target remote | $(OPENOCDPATH)/openocd -c \"gdb_port pipe\" --search $(OPENOCDSCRIPTSPATH) $(OPENOCDARGS)' -ex 'continue'"

else

flash gdb:
	echo OPENOCD not found ; exit 1

endif
