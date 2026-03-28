obj-m += kpm_oc.o

# Set this to your GKI kernel source/headers path.
# If building on a native Linux host for the current kernel:
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
