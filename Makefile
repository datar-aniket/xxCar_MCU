# Thin wrapper over the NuttX build for xxCar_MCU.
.PHONY: all build flash menuconfig reconfigure clean distclean submodules

NUTTX := deps/nuttx
BOARD ?= pixhawk6c

all: build

submodules:
	git submodule update --init --recursive

build:
	tools/build.sh $(BOARD)

flash:
	tools/flash.sh $(BOARD)

# Interactive Kconfig editor against the current NuttX config.
menuconfig:
	$(MAKE) -C $(NUTTX) menuconfig

# Re-run configure.sh from scratch next build.
reconfigure:
	RECONFIGURE=1 tools/build.sh $(BOARD)

clean:
	-$(MAKE) -C $(NUTTX) clean
	rm -rf build

distclean:
	-$(MAKE) -C $(NUTTX) distclean
	rm -rf build
