# Makefile for TI M0G3507 Multi-Sensor Safety Warning System
# Author: TI M0G3507 Safety System
# Date: 2024

# Project name
PROJECT = ti_m0g3507_safety_system

# Target microcontroller
MCU = M0G3507

# Compiler and tools
CC = arm-none-eabi-gcc
AR = arm-none-eabi-ar
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE = arm-none-eabi-size

# Directories
SRC_DIR = .
BUILD_DIR = build
OBJ_DIR = $(BUILD_DIR)/obj

# Source files
SOURCES = main.c \
          i2c_slave.c \
          adc_sensors.c \
          gpio_sensors.c \
          safety_warning.c \
          voice_module.c

# Header files
HEADERS = system_config.h \
          i2c_slave.h \
          adc_sensors.h \
          gpio_sensors.h \
          safety_warning.h \
          voice_module.h

# Object files
OBJECTS = $(SOURCES:%.c=$(OBJ_DIR)/%.o)

# Compiler flags
CFLAGS = -mcpu=cortex-m0plus \
         -mthumb \
         -std=c99 \
         -Wall \
         -Wextra \
         -Werror \
         -O2 \
         -g3 \
         -ffunction-sections \
         -fdata-sections \
         -DDEBUG \
         -DTI_M0G3507

# Include paths
INCLUDES = -I.

# Linker flags
LDFLAGS = -mcpu=cortex-m0plus \
          -mthumb \
          -specs=nano.specs \
          -T$(MCU).ld \
          -Wl,--gc-sections \
          -Wl,--print-memory-usage

# Libraries
LIBS = -lc -lm -lnosys

# Target files
ELF_FILE = $(BUILD_DIR)/$(PROJECT).elf
HEX_FILE = $(BUILD_DIR)/$(PROJECT).hex
BIN_FILE = $(BUILD_DIR)/$(PROJECT).bin
MAP_FILE = $(BUILD_DIR)/$(PROJECT).map

# Default target
all: $(HEX_FILE) $(BIN_FILE) size

# Create hex file
$(HEX_FILE): $(ELF_FILE)
	@echo "Creating hex file: $@"
	$(OBJCOPY) -O ihex $< $@

# Create binary file
$(BIN_FILE): $(ELF_FILE)
	@echo "Creating binary file: $@"
	$(OBJCOPY) -O binary $< $@

# Link ELF file
$(ELF_FILE): $(OBJECTS) | $(BUILD_DIR)
	@echo "Linking: $@"
	$(CC) $(LDFLAGS) -Wl,-Map=$(MAP_FILE) -o $@ $^ $(LIBS)

# Compile source files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c $(HEADERS) | $(OBJ_DIR)
	@echo "Compiling: $<"
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Create build directories
$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

$(OBJ_DIR):
	@mkdir -p $(OBJ_DIR)

# Display size information
size: $(ELF_FILE)
	@echo "Size information:"
	$(SIZE) $<

# Disassemble
disasm: $(ELF_FILE)
	@echo "Disassembling: $<"
	$(OBJDUMP) -D $< > $(BUILD_DIR)/$(PROJECT).disasm

# Clean build files
clean:
	@echo "Cleaning build files..."
	rm -rf $(BUILD_DIR)

# Static analysis with cppcheck (if available)
check:
	@echo "Running static analysis..."
	@if command -v cppcheck >/dev/null 2>&1; then \
		cppcheck --enable=all --std=c99 --platform=unix32 \
		--error-exitcode=1 --inline-suppr \
		$(SOURCES); \
	else \
		echo "cppcheck not found, skipping static analysis"; \
	fi

# Format code with clang-format (if available)
format:
	@echo "Formatting code..."
	@if command -v clang-format >/dev/null 2>&1; then \
		clang-format -i -style="{BasedOnStyle: LLVM, IndentWidth: 4, TabWidth: 4, UseTab: Never}" \
		$(SOURCES) $(HEADERS); \
	else \
		echo "clang-format not found, skipping code formatting"; \
	fi

# Generate documentation with doxygen (if available)
docs:
	@echo "Generating documentation..."
	@if command -v doxygen >/dev/null 2>&1; then \
		if [ ! -f Doxyfile ]; then \
			doxygen -g; \
			sed -i 's/PROJECT_NAME           = "My Project"/PROJECT_NAME           = "TI M0G3507 Safety System"/' Doxyfile; \
			sed -i 's/INPUT                  =/INPUT                  = . /' Doxyfile; \
			sed -i 's/EXTRACT_ALL            = NO/EXTRACT_ALL            = YES/' Doxyfile; \
		fi; \
		doxygen; \
	else \
		echo "doxygen not found, skipping documentation generation"; \
	fi

# Install/flash target (placeholder - depends on programmer)
install: $(HEX_FILE)
	@echo "Installing to target device..."
	@echo "Flash $(HEX_FILE) to TI M0G3507 using appropriate programmer"
	@echo "Example: uniflash -ccxml ccxml_file -program $(HEX_FILE)"

# Debug target (placeholder - depends on debugger)
debug: $(ELF_FILE)
	@echo "Starting debug session..."
	@echo "Use appropriate debugger for TI M0G3507"
	@echo "Example: gdb $(ELF_FILE)"

# Test target (basic compile test)
test: all
	@echo "Basic compilation test passed"
	@echo "Flash size: $(shell $(SIZE) $(ELF_FILE) | tail -1 | awk '{print $$1}')"
	@echo "RAM usage: $(shell $(SIZE) $(ELF_FILE) | tail -1 | awk '{print $$2}')"

# Help target
help:
	@echo "Available targets:"
	@echo "  all      - Build hex and binary files (default)"
	@echo "  clean    - Remove all build files"
	@echo "  size     - Display size information"
	@echo "  disasm   - Generate disassembly file"
	@echo "  check    - Run static analysis (requires cppcheck)"
	@echo "  format   - Format source code (requires clang-format)"
	@echo "  docs     - Generate documentation (requires doxygen)"
	@echo "  install  - Flash to target device"
	@echo "  debug    - Start debug session"
	@echo "  test     - Run basic tests"
	@echo "  help     - Show this help message"

# Mark targets as phony
.PHONY: all clean size disasm check format docs install debug test help

# Dependencies
$(OBJ_DIR)/main.o: system_config.h i2c_slave.h adc_sensors.h gpio_sensors.h safety_warning.h voice_module.h
$(OBJ_DIR)/i2c_slave.o: i2c_slave.h system_config.h
$(OBJ_DIR)/adc_sensors.o: adc_sensors.h system_config.h
$(OBJ_DIR)/gpio_sensors.o: gpio_sensors.h system_config.h
$(OBJ_DIR)/safety_warning.o: safety_warning.h system_config.h adc_sensors.h gpio_sensors.h
$(OBJ_DIR)/voice_module.o: voice_module.h system_config.h safety_warning.h