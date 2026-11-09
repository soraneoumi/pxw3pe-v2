# SPDX-License-Identifier: GPL-2.0
#
# Out-of-tree build. KDIR must point at a configured and prepared kernel tree (or its build
# directory) with CONFIG_RUST and CONFIG_DVB_CORE enabled. Pass the same
# LLVM=1 (and ARCH=... when cross-compiling) that was used to build that kernel.

KDIR ?= /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean

.PHONY: all clean
