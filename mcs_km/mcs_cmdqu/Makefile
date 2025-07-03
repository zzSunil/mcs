obj-m := rtos_cmdqu_km.o
rtos_cmdqu_km-objs := rtos_cmdqu.o cvi_spinlock.o

SRC := $(shell pwd)
KERNEL_SRC=/lib/modules/6.4.0-10.1.0.20.oe2309.riscv64/build

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC)


modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c *.mod
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers
