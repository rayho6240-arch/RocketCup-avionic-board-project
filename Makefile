# ============================================================
#  RocketCom Top-level Makefile
#  Usage:
#    make            — compile only
#    make flash      — compile + upload via ST-Link
#    make monitor    — open serial monitor (Ctrl+A K to quit)
#    make all        — compile + flash + monitor
#    make clean      — remove build artifacts
# ============================================================

# ---- Toolchain (STM32CubeIDE built-in, must NOT use Homebrew version) ----
CUBEIDE_PLUGINS := /Applications/STM32CubeIDE.app/Contents/Eclipse/plugins
GCC_PATH        := $(CUBEIDE_PLUGINS)/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.macos64_1.0.0.202411102158/tools/bin
CUBEPROG        := $(CUBEIDE_PLUGINS)/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.macos64_2.2.200.202503041107/tools/bin/STM32_Programmer_CLI

# ---- Project paths ----
BUILD_DIR := Main_Code/Debug
ELF       := $(BUILD_DIR)/Main_AV_F407.elf

# ---- Serial monitor ----
PORT      := /dev/cu.usbserial-0001
BAUD      := 460800

# ============================================================

.PHONY: build flash monitor all clean

## Default target: compile only
build:
	export PATH="$(GCC_PATH):$$PATH" && \
	$(MAKE) -C $(BUILD_DIR) all

## Alias so bare `make` also compiles
default: build

## Upload .elf to MCU via ST-Link SWD
flash: build
	"$(CUBEPROG)" -c port=SWD -w $(ELF) -rst

## Open serial monitor (quit with Ctrl+A then K)
monitor:
	screen $(PORT) $(BAUD)

## Compile + flash + monitor
all: flash monitor

## Remove build artifacts
clean:
	$(MAKE) -C $(BUILD_DIR) clean
