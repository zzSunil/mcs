obj-m := mcs_km.o

SRC := $(shell pwd)

ifeq ($(TARGET),K1)
EXTRA_CFLAGS += -DCONFIG_K1
endif

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC)

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c *.mod
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers
