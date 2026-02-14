#
# Setup all targets to repeatedly run as tasks
#
.PHONY: default all arduino-cli config-file cores libs clean distclean sketches flash monitor
default: help

#
# Global Variables
#
ROOT_DIR := $(patsubst %/,%,$(dir $(realpath $(lastword $(MAKEFILE_LIST)))))
BOARD_MANAGER_URLS := https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json https://raw.githubusercontent.com/pdcook/nRFMicro-Arduino-Core/main/package_nRFMicro_index.json
ARDUINO_CLI ?= $(shell command -v arduino-cli || echo "/usr/local/bin/arduino-cli")
ARDUINO_CLI_VERSION ?= 1.1.1
ARDUINO_CONFIG_FILE ?= $(ROOT_DIR)/arduino-cli.yaml
export ARDUINO_CONFIG_FILE
CORES := "nRFMicro-like Boards:nrf52@1.0.2"
BOARD ?= shorepine:nrf52:supermini
SKETCHES := $(wildcard *.ino)
PORT ?= $(firstword $(wildcard /dev/cu.usbmodem*) /dev/cu.usbmodemNotFound)


#
# Dymanically generated targets
#

define SKETCH_template
.PHONY: $(1)
$(1): toolchain
	@echo Building sketch: $$@
	$(ARDUINO_CLI) compile --fqbn $(BOARD) --export-binaries $$@
GENERATED_FILES += build
endef

$(foreach sketch,$(SKETCHES),$(eval $(call SKETCH_template,$(sketch))))


#
# Makefile targets
#

$(ARDUINO_CLI):
	curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=$(dir $(ARDUINO_CLI)) sh -s $(ARDUINO_CLI_VERSION)

arduino-cli: $(ARDUINO_CLI)  ## Install arduino-cli
	@$(ARDUINO_CLI) version

$(ARDUINO_CONFIG_FILE): $(ARDUINO_CLI)
	@$(ARDUINO_CLI) config init --overwrite --dest-file $(ARDUINO_CONFIG_FILE)
	@$(ARDUINO_CLI) config set board_manager.additional_urls $(BOARD_MANAGER_URLS)
	@$(ARDUINO_CLI) config set directories.data $(ROOT_DIR)/Arduino15
	@$(ARDUINO_CLI) config set directories.downloads $(ROOT_DIR)/Arduino15/staging
	@$(ARDUINO_CLI) config set directories.user $(ROOT_DIR)

config-file: $(ARDUINO_CONFIG_FILE)  ## Create local config file, add BOARD_MANAGER_URLS
	@$(ARDUINO_CLI) config add board_manager.additional_urls $(BOARD_MANAGER_URLS)

toolchain: $(ARDUINO_CONFIG_FILE) $(ARDUINO_CLI)

cores: toolchain  ## Install the required platform cores
	@$(ARDUINO_CLI) core update-index
	@$(ARDUINO_CLI) core install $(CORES)

libs: toolchain  ## Install required libraries
	@$(ARDUINO_CLI) config set library.enable_unsafe_install true
	@echo Installing AMY_Synthesizer using --git-url
	@$(ARDUINO_CLI) lib install --git-url https://github.com/jgartrel/amy_synth.git#a209c428b146684c29a5afaa4c321b00ee324409
	@$(ARDUINO_CLI) config set library.enable_unsafe_install false
	@$(ARDUINO_CLI) lib install "Time"@1.6.1

sketches: toolchain $(SKETCHES)  ## Build all sketches

all: arduino-cli config-file cores libs sketches  ## make arduino-cli config-file cores libs sketches

flash: sketches  ## Upload build to device for nrf52_blink_info
	@echo Flash using port: $(PORT)
	@$(ARDUINO_CLI) upload --fqbn $(BOARD) --port $(PORT)

monitor:  ## Connect to serial console on device
	@echo Monitor using port: $(PORT)
	@$(ARDUINO_CLI) monitor --fqbn $(BOARD) --port $(PORT)

clean:  ## Remove all generated files
	rm -rf $(GENERATED_FILES)

distclean:  ## Remove all non-versioned files
	git clean -f -x -d

help:  ## Display the common make targets
	@grep -E '^[a-zA-Z0-9_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-20s\033[0m %s\n", $$1, $$2}'
