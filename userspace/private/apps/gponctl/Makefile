EXE = gponctl
dynamic: generic_exe_install

clean:
	rm -f gponctl


gponctl:
	cp gponctl_$(PROFILE)_save gponctl

CURR_DIR := $(shell pwd)
BUILD_DIR := $(subst /userspace, /userspace,$(CURR_DIR))
BUILD_DIR := $(word 1, $(BUILD_DIR))
include $(BUILD_DIR)/make.common
