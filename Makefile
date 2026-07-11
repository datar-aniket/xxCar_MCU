# Thin wrapper over the NuttX build for xxCar_MCU (FMUv6C).
.PHONY: all build flash menuconfig clean distclean submodules

NUTTX := deps/nuttx

all: build

submodules:
	git submodule update --init --recursive

build:
	tools/build.sh

flash:
	tools/flash.sh

# Interactive Kconfig editor against the current NuttX config.
menuconfig:
	$(MAKE) -C $(NUTTX) menuconfig

# Re-run configure.sh from scratch next build.
reconfigure:
	RECONFIGURE=1 tools/build.sh

clean:
	-$(MAKE) -C $(NUTTX) clean
	rm -rf build

distclean:
	-$(MAKE) -C $(NUTTX) distclean
	rm -rf build
