LIB = libomcipm_drv.so
default: dynamic


libomcipm_drv.so:
	cp libomcipm_drv.so_$(PROFILE)_save libomcipm_drv.so

dynamic: generic_private_lib_install

clean:
	rm -f libomcipm_drv.so

CURR_DIR := $(shell pwd)
BUILD_DIR := $(subst /userspace, /userspace,$(CURR_DIR))
BUILD_DIR := $(word 1, $(BUILD_DIR))
include $(BUILD_DIR)/make.common
