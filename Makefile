.DEFAULT_GOAL := compile

ARDUINO_CLI ?= arduino-cli
FQBN ?= esp32:esp32:esp32:CPUFreq=240,FlashFreq=80,FlashSize=4M,PartitionScheme=min_spiffs
BUILD_DIR ?= $(CURDIR)/build
JOBS ?= 2
# Use an explicit PORT when multiple USB serial devices are connected.
PORT ?= $(if $(filter 1,$(words $(wildcard /dev/ttyUSB* /dev/ttyACM* /dev/cu.usb*))),$(wildcard /dev/ttyUSB* /dev/ttyACM* /dev/cu.usb*))

.PHONY: compile program ports check-config check-port

check-config:
	@test -f config.h || { echo 'Copy config-example.h to config.h and configure WiFi/MQTT first.' >&2; exit 1; }

compile: check-config
	version=$$(git describe --tags --always --dirty) || exit 1; \
	echo "Building firmware $$version"; \
	$(ARDUINO_CLI) compile --verbose --fqbn "$(FQBN)" --jobs "$(JOBS)" --build-path "$(BUILD_DIR)" \
		--build-property "compiler.cpp.extra_flags=-DGIT_VERSION=\"$$version\"" .

check-port:
	@test -n "$(PORT)" || { echo 'No unique USB serial port. Run make ports, then make program PORT=/dev/ttyUSB0.' >&2; exit 1; }

# Recursive make ensures port validation and compilation finish before upload,
# including when the caller uses make -j.
program: check-port
	$(MAKE) compile
	$(ARDUINO_CLI) upload --verbose --fqbn "$(FQBN)" --port "$(PORT)" --input-dir "$(BUILD_DIR)" .

ports:
	$(ARDUINO_CLI) board list
