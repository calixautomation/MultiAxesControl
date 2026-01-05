# Firmware

This directory contains the main motor control application firmware.

## Structure
- `src/main.c` - Main firmware entry point with FreeRTOS task

## Building
```bash
cd build
cmake --build . --target firmware
```

## Output Files
- `firmware` - ELF executable
- `firmware.bin` - Binary for flashing
- `firmware.hex` - Intel HEX format
- `firmware.map` - Memory map

## Features
- FreeRTOS-based task management
- LED blink task (example)
- STM32G474RE HAL integration
