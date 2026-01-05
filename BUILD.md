# Build System

Cross-platform Python build scripts for isolated target builds.

## Quick Start

```bash
# Build blinky example
python build_blinky.py

# Build firmware
python build_firmware.py

# Build all targets
python build_all.py
```

## Build Scripts

### `build_blinky.py`
Builds the blinky example in its own isolated build directory.

**Options:**
- `--clean` - Clean build directory before building
- `--rebuild` - Clean and rebuild (same as --clean)

**Output:** `builds/build_blinky/`

### `build_firmware.py`
Builds the firmware in its own isolated build directory.

**Options:**
- `--clean` - Clean build directory before building
- `--rebuild` - Clean and rebuild

**Output:** `builds/build_firmware/`

### `build_all.py`
Builds all targets in separate directories.

**Options:**
- `--clean` - Clean all build directories before building

## Directory Structure

```
MultiAxesControl/
├── builds/                    # All build outputs
│   ├── build_blinky/          # Blinky build directory
│   │   ├── CMakeFiles/        # CMake build files
│   │   ├── build_blinky/      # Output files
│   │   │   ├── blinky
│   │   │   ├── blinky.bin
│   │   │   ├── blinky.hex
│   │   │   └── blinky.map
│   │   └── [Ninja, CMake cache, etc.]
│   │
│   └── build_firmware/        # Firmware build directory
│       ├── CMakeFiles/        # CMake build files
│       ├── build_firmware/    # Output files
│       │   ├── firmware
│       │   ├── firmware.bin
│       │   ├── firmware.hex
│       │   └── firmware.map
│       └── [Ninja, CMake cache, FreeRTOS lib, etc.]
│
├── build_blinky.py           # Blinky build script
├── build_firmware.py         # Firmware build script
└── build_all.py              # Master build script
```

## Benefits

✅ **Cross-platform** - Works on Windows, Linux, and macOS  
✅ **Complete isolation** - Each target has its own CMakeFiles  
✅ **Parallel builds** - Can build targets independently  
✅ **Clean separation** - No shared build artifacts  
✅ **Easy cleanup** - Delete individual build directories  
✅ **CI/CD friendly** - Each target can be built in separate jobs  

## Requirements

- Python 3.7+
- CMake 3.15+
- Ninja build system
- ARM GCC toolchain (arm-none-eabi-gcc)

## Examples

```bash
# Clean build of blinky
python build_blinky.py --clean

# Rebuild firmware
python build_firmware.py --rebuild

# Clean build all targets
python build_all.py --clean
```

## Batch Files (Windows)

For Windows users, `.bat` files are also provided:
- `build_blinky.bat [clean]`
- `build_firmware.bat [clean]`
