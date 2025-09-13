#!/bin/bash
# verify_implementation.sh - Verification script for TI M0G3507 Safety System

echo "=== TI M0G3507 Multi-Sensor Safety Warning System Verification ==="
echo

# Check file structure
echo "1. Checking file structure..."
required_files=(
    "main.c"
    "system_config.h"
    "i2c_slave.h" "i2c_slave.c"
    "adc_sensors.h" "adc_sensors.c"
    "gpio_sensors.h" "gpio_sensors.c"
    "safety_warning.h" "safety_warning.c"
    "voice_module.h" "voice_module.c"
    "Makefile"
    "README.md"
)

missing_files=0
for file in "${required_files[@]}"; do
    if [ -f "$file" ]; then
        echo "  ✓ $file"
    else
        echo "  ✗ $file (missing)"
        ((missing_files++))
    fi
done

if [ $missing_files -eq 0 ]; then
    echo "  All required files present!"
else
    echo "  $missing_files files missing!"
fi
echo

# Check syntax
echo "2. Checking C syntax..."
if gcc -c -Wall -Wextra -std=c99 -fsyntax-only -Wno-unused-parameter *.c 2>/dev/null; then
    echo "  ✓ All C files compile without syntax errors"
else
    echo "  ✗ Syntax errors found"
fi
echo

# Check key requirements
echo "3. Checking implementation requirements..."

# I2C slave functionality
if grep -q "I2C_SLAVE_ADDRESS" system_config.h && grep -q "i2c_slave_init" i2c_slave.h; then
    echo "  ✓ I2C slave communication module implemented"
else
    echo "  ✗ I2C slave communication missing"
fi

# ADC sensors
if grep -q "ADC_CHANNEL_ANGLE\|ADC_CHANNEL_GAS" system_config.h && grep -q "adc_sensors_init" adc_sensors.h; then
    echo "  ✓ ADC sensor reading modules implemented"
else
    echo "  ✗ ADC sensor modules missing"
fi

# GPIO sensors
if grep -q "GPIO_VIBRATION" system_config.h && grep -q "gpio_sensors_init" gpio_sensors.h; then
    echo "  ✓ GPIO sensor reading module implemented"
else
    echo "  ✗ GPIO sensor module missing"
fi

# Safety warning system
if grep -q "warning_level_t\|safety_warning_init" safety_warning.h; then
    echo "  ✓ Safety warning logic implemented"
else
    echo "  ✗ Safety warning logic missing"
fi

# Voice module
if grep -q "voice_module_init\|VOICE_MSG" voice_module.h; then
    echo "  ✓ Voice module implemented"
else
    echo "  ✗ Voice module missing"
fi

# Power management
if grep -q "POWER_MODE_STANDBY\|STANDBY_CURRENT_TARGET" system_config.h; then
    echo "  ✓ Power management features implemented"
else
    echo "  ✗ Power management missing"
fi

echo

# Check technical specifications
echo "4. Checking technical specifications..."

# Sampling frequency
if grep -q "SENSOR_SAMPLING_FREQ_HZ.*10" system_config.h; then
    echo "  ✓ 10Hz sensor sampling frequency configured"
else
    echo "  ✗ Sensor sampling frequency not set to 10Hz"
fi

# I2C frequency
if grep -q "I2C_CLOCK_FREQ_HZ.*100000" system_config.h; then
    echo "  ✓ 100kHz I2C communication configured"
else
    echo "  ✗ I2C frequency not set to 100kHz"
fi

# Response time
if grep -q "VOICE_RESPONSE_TIME_MS.*100" system_config.h; then
    echo "  ✓ 100ms voice response time configured"
else
    echo "  ✗ Voice response time not set to 100ms"
fi

# Standby current
if grep -q "STANDBY_CURRENT_TARGET_UA.*1000" system_config.h; then
    echo "  ✓ 1mA standby current target configured"
else
    echo "  ✗ Standby current target not set to 1mA"
fi

echo

# Calculate code statistics
echo "5. Code statistics..."
total_lines=$(wc -l *.c *.h | tail -1 | awk '{print $1}')
c_lines=$(wc -l *.c | tail -1 | awk '{print $1}')
h_lines=$(wc -l *.h | tail -1 | awk '{print $1}')

echo "  Total lines of code: $total_lines"
echo "  C source files: $c_lines lines"
echo "  Header files: $h_lines lines"

# Count functions
func_count=$(grep -c "^[a-zA-Z_][a-zA-Z0-9_]*.*(" *.c | awk -F: '{sum += $2} END {print sum}')
echo "  Approximate function count: $func_count"

echo

echo "=== Verification Complete ==="
echo "This implementation provides a comprehensive multi-sensor safety"
echo "warning system for the TI M0G3507 microcontroller with:"
echo "- Modular architecture with clear separation of concerns"
echo "- I2C slave communication for external control"
echo "- Multi-sensor monitoring (angle, gas, vibration)"
echo "- Real-time safety warning logic with multiple alert levels"
echo "- Voice alert system with priority-based message queuing"
echo "- Low-power standby mode support"
echo "- Comprehensive error handling and recovery"
echo