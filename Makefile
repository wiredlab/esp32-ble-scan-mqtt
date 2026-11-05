.DEFAULT_GOAL := compile

ARDUINO_CLI ?= arduino-cli
FQBN ?= esp32:esp32:esp32:CPUFreq=240,FlashFreq=80,FlashSize=4M,PartitionScheme=min_spiffs
BUILD_DIR ?= $(CURDIR)/build
JOBS ?= 2
# Optional override; otherwise resolve esptool from the installed Arduino core.
ESPTOOL ?=
# Use an explicit PORT when multiple USB serial devices are connected.
PORT ?= $(if $(filter 1,$(words $(wildcard /dev/ttyUSB* /dev/ttyACM* /dev/cu.usb*))),$(wildcard /dev/ttyUSB* /dev/ttyACM* /dev/cu.usb*))

.PHONY: compile program erase ports check-config check-port

check-config:
	@test -f config.h || { echo 'Copy config-example.h to config.h and configure WiFi/MQTT first.' >&2; exit 1; }

compile: check-config
	version=$$(git describe --tags --always --dirty) || exit 1; \
	echo "Building firmware $$version"; \
	$(ARDUINO_CLI) compile --verbose --fqbn "$(FQBN)" --jobs "$(JOBS)" --build-path "$(BUILD_DIR)" \
		--build-property "compiler.cpp.extra_flags=-DGIT_VERSION=\"$$version\"" .

check-port:
	@test -n "$(PORT)" || { echo 'No unique USB serial port. Run make ports, then specify PORT=/dev/ttyUSB0.' >&2; exit 1; }

# Erase all flash without compiling or uploading firmware.
erase: check-port
	tool="$(ESPTOOL)"; \
	if [ -z "$$tool" ]; then \
		properties=$$($(ARDUINO_CLI) compile --fqbn "$(FQBN)" --show-properties) || exit 1; \
		tool_dir=$$(printf '%s\n' "$$properties" | sed -n 's/^tools.esptool_py.path=//p'); \
		tool_cmd=$$(printf '%s\n' "$$properties" | sed -n 's/^tools.esptool_py.cmd=//p'); \
		test -n "$$tool_dir" && test -n "$$tool_cmd" || { echo 'Cannot locate esptool; set ESPTOOL explicitly.' >&2; exit 1; }; \
		tool="$$tool_dir/$$tool_cmd"; \
	fi; \
	set -x; \
	"$$tool" --chip esp32 --port "$(PORT)" erase-flash

# Recursive make ensures port validation and compilation finish before upload,
# including when the caller uses make -j.
program: check-port
	$(MAKE) compile
	$(ARDUINO_CLI) upload --verbose --fqbn "$(FQBN)" --port "$(PORT)" --input-dir "$(BUILD_DIR)" .

ports:
	$(ARDUINO_CLI) board list
