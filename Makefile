obj-m += sound/firewire/

ifndef KERNELRELEASE
KERNELRELEASE := $(shell uname -r)
endif

KDIR := /lib/modules/$(KERNELRELEASE)/build
PWD := $(shell pwd)

#force to build all modules
export CONFIG_SND_FIREWIRE_LIB=m
export CONFIG_SND_DICE=m
export CONFIG_SND_ISIGHT=m
export CONFIG_SND_BEBOB=m
export CONFIG_SND_FIREWORKS=m
export CONFIG_SND_OXFW=m
export CONFIG_SND_FIREWIRE_DIGI00X=m
export CONFIG_SND_FIREWIRE_TASCAM=m
export CONFIG_SND_FIREWIRE_MOTU=m
export CONFIG_SND_FIREFACE=m

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

check: clean
	$(MAKE) C=1 CF=-D__CHECK_ENDIAN__ -C $(KDIR) M=$(PWD) modules

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
