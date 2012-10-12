LIB = libomci.so
default: dynamic


libomci.so:
	cp libomci.so_$(PROFILE)_save libomci.so

dynamic: generic_private_lib_install

clean:
	rm -f libomci.so

CURR_DIR := $(shell pwd)
BUILD_DIR := $(subst /userspace, /userspace,$(CURR_DIR))
BUILD_DIR := $(word 1, $(BUILD_DIR))
include $(BUILD_DIR)/make.common
