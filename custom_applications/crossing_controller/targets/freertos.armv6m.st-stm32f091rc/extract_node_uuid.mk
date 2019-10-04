LOL := $(shell if ps ax -o comm | grep -q openocd ; then echo openocd already running. quit existing first. ; exit 1 ; fi)

STM32_F09x1_IDCODE_LOC := "0x40015800"

$(shell rm -f device_identifiers.output)

LOL := $(shell $(GDB) $< -ex "target remote | $(OPENOCDPATH)/openocd -c \"gdb_port pipe\" --search $(OPENOCDSCRIPTSPATH) $(OPENOCDARGS)" \
	-ex "monitor reset halt" \
	-ex "monitor reset init" \
	-ex "monitor log_output device_identifiers.output" \
	-ex "monitor mdh phys $(STM32_F09x1_IDCODE_LOC) 2" \
	-ex "monitor mdw phys 0x1FFFF7AC 2" \
	-ex "monitor log_output" \
	-ex "monitor reset run" \
	-ex "detach" \
	-ex "quit")

NODE_NUMBER := $(shell python3 ./extract_node_uuid.py)

$(info UUID extraction retcode: $(.SHELLSTATUS))

ifneq ($(.SHELLSTATUS),0)
$(warning UUID extraction failed, using 0)
NODE_NUMBER := 0
endif

$(info NODE_NUMBER: $(NODE_NUMBER))

ifneq ($(shell cat node_number.output 2>/dev/null),node_number_prefix $(NODE_NUMBER))
LOL := $(shell touch NodeId.cxx)
LOL := $(shell echo 'node_number_prefix $(NODE_NUMBER)' >node_number.output &)
LOL := $(shell echo "Recompiling for NODE_NUMBER $(NODE_NUMBER)" >&2)
endif