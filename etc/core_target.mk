ifeq ($(strip $(TARGET)),)
TARGET := $(shell basename `pwd`)
endif
ifndef OPENMRNPATH
OPENMRNPATH:=$(realpath ../..)
endif
export OPENMRNPATH

#GRABDIRLOCK=$(shell $(OPENMRNPATH)/etc/dirguard.sh )
#$(info dirlock $(GRABDIRLOCK))

include $(OPENMRNPATH)/etc/config.mk
include $(OPENMRNPATH)/etc/path.mk
include $(OPENMRNPATH)/etc/$(TARGET).mk

# lib here is only needed for clean to work properly. Libraries are copied
# there by the original build rules.
SUBDIRS = $(CORELIBS) $(SYSLIB_SUBDIRS) lib

# This defines how to create nonexistant directories.
MKSUBDIR_OPENMRNINCLUDE=lib.mk

DEPS += TOOLPATH
MISSING_DEPS:=$(call find_missing_deps,$(DEPS))

ifneq ($(MISSING_DEPS),)

all docs clean veryclean tests mksubdirs:
	@echo "******************************************************************"
	@echo "*"
	@echo "*   Unable to build for $(TARGET)/$(REL_DIR), missing dependencies: $(MISSING_DEPS)"
	@echo "*"
	@echo "******************************************************************"

else
include $(OPENMRNPATH)/etc/recurse.mk
endif
