obj-m := mcs_km.o

SRC := $(shell pwd)
KERNEL_SRC=/lib/modules/6.4.0-10.1.0.20.oe2309.riscv64/build

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC)

milkvduo:
	$(eval obj-m += mcs_cmdqu/rtos_cmdqu_km.o)
	$(MAKE) -C $(SRC)/mcs_cmdqu -j
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) EXTRA_CFLAGS="-DMILKVDUO"

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	make -C $(SRC)/mcs_cmdqu clean
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c *.mod
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers
