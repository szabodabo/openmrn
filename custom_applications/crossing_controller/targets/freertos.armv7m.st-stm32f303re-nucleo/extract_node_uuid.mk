LOL := $(shell if ps ax -o comm | grep -q openocd ; then echo openocd already running. quit existing first. ; exit 1 ; fi)

LOL := $(shell $(GDB) $< -ex "target remote | $(OPENOCDPATH)/openocd -c \"gdb_port pipe\" --search $(OPENOCDSCRIPTSPATH) $(OPENOCDARGS)" \
	-ex "monitor reset halt" \
	-ex "monitor reset init" \
	-ex "monitor log_output device_identifiers.output" \
	-ex "monitor mdh phys 0xE0042000 2" \
	-ex "monitor mdw phys 0x1FFFF7AC 2" \
	-ex "monitor log_output" \
	-ex "monitor reset run" \
	-ex "detach" \
	-ex "quit")

NODE_NUMBER := $(shell python3 ./extract_node_uuid.py)