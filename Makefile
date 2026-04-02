obj-m += kpm_oc.o

# Use build.sh for a guided build. Direct make invocation:
#   make KERNEL_DIR=/path/to/android14-6.1/common ARCH=arm64 LLVM=1 LLVM_IAS=1 KBUILD_MODPOST_WARN=1 modules
KERNEL_DIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

all modules:
	@if [ ! -d "$(KERNEL_DIR)" ]; then \
		echo "ERROR: Kernel source directory '$(KERNEL_DIR)' not found."; \
		echo "Please provide the path to your Android GKI kernel source/headers."; \
		echo "Example: make KERNEL_DIR=/path/to/android-kernel/out/android14-6.1/common"; \
		exit 1; \
	fi
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) clean
	rm -rf *.o *.ko *.mod.c *.symvers *.order .*.cmd
