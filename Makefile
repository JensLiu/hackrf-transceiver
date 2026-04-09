# Master Makefile for ASK Project
FIRMWARE_DIR = firmware/hackrf_usb/build

.PHONY: build flash

all: build flash

build:
	@echo "Compiling firmware..."
	$(MAKE) -C $(FIRMWARE_DIR) -j$$(nproc)

flash: build
	@echo "Flashing HackRF..."
	hackrf_spiflash -w $(FIRMWARE_DIR)/hackrf_usb.bin

clean:
	$(MAKE) -C $(FIRMWARE_DIR) clean