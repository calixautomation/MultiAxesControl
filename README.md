# MultiAxesControl

A modular, layered motor control system designed for automated roughness measurement of curved surfaces. This project implements a platform-agnostic architecture with FreeRTOS support and CMake build system.

## Features

- **Modular Architecture**: Layered design with clear separation of concerns
- **Platform Agnostic**: Generic register configuration system supporting multiple platforms
- **HAL (Hardware Abstraction Layer)**: Platform-specific implementations for Arduino, STM32, and ESP32
- **FreeRTOS Integration**: Real-time operating system support with task management
- **CMake Build System**: Cross-platform build configuration
- **Python GUI**: User-friendly interface for motor control
- **Generic Register Mapping**: Platform-independent hardware access

## Project Structure

```
MultiAxesControl/
├── src/
│   ├── application/          # Application layer
│   │   ├── motor_control.h
│   │   └── motor_control.c
│   ├── config/               # Configuration system
│   │   └── register_config.h
│   ├── hal/                  # Hardware Abstraction Layer
│   │   ├── hal_interface.h
│   │   └── platforms/
│   │       ├── arduino/
│   │       │   ├── hal_arduino.h
│   │       │   ├── hal_arduino.c
│   │       │   └── CMakeLists.txt
│   │       ├── stm32/
│   │       └── esp32/
│   ├── os/                   # Operating System layer
│   │   └── freertos/
│   │       ├── rtos_interface.h
│   │       ├── rtos_freertos.h
│   │       └── rtos_freertos.c
│   └── middleware/           # Middleware layer
├── gui_app/                  # Python GUI application
│   └── motor_control_gui.py
├── examples/                 # Example applications
│   └── main.c
├── docs/                     # Documentation
├── tests/                    # Unit tests
├── build/                    # Build output directory
├── CMakeLists.txt           # Main CMake configuration
└── README.md
```

## Architecture Overview

### Layer 1: Hardware Abstraction Layer (HAL)
- **Purpose**: Platform-specific hardware access
- **Platforms**: Arduino, STM32, ESP32
- **Features**: Generic register mapping, interrupt handling, peripheral control

### Layer 2: Operating System (OS)
- **Purpose**: Real-time task management
- **RTOS**: FreeRTOS
- **Features**: Task management, queues, semaphores, timers

### Layer 3: Application Layer
- **Purpose**: Motor control logic and command processing
- **Features**: Command parsing, motor control modes, system configuration

### Layer 4: User Interface
- **Purpose**: Human-machine interface
- **Implementation**: Python GUI with Tkinter
- **Features**: Real-time control, status monitoring, command interface

## Hardware Requirements

### Arduino Platform
- Arduino Uno or compatible
- Stepper motor drivers (TB6600)
- Linear and rotary stages
- Serial communication interface

### STM32 Platform
- STM32F4 series microcontroller
- Stepper motor drivers
- UART interface

### ESP32 Platform
- ESP32 development board
- Stepper motor drivers
- WiFi/Bluetooth communication (optional)

## Software Requirements

### Build System
- CMake 3.16 or later
- Platform-specific toolchains:
  - Arduino: Arduino IDE or PlatformIO
  - STM32: ARM GCC toolchain
  - ESP32: ESP-IDF

### Python GUI
- Python 3.7 or later
- Required packages:
  - tkinter (usually included with Python)
  - pyserial
  - json (built-in)

## Building the Project

### Using CMake

1. **Create build directory**:
   ```bash
   mkdir build
   cd build
   ```

2. **Configure for target platform**:
   ```bash
   # For Arduino
   cmake .. -DPLATFORM=arduino
   
   # For STM32
   cmake .. -DPLATFORM=stm32
   
   # For ESP32
   cmake .. -DPLATFORM=esp32
   ```

3. **Build the project**:
   ```bash
   cmake --build .
   ```

### Platform-Specific Builds

#### Arduino
```bash
# Using PlatformIO
pio run -e arduino_uno

# Using Arduino IDE
# Import the project and compile
```

#### STM32
```bash
# Using STM32CubeIDE
# Import project and build

# Using command line
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -o firmware.elf src/*.c
```

#### ESP32
```bash
# Using ESP-IDF
idf.py build

# Using PlatformIO
pio run -e esp32dev
```

## Running the Application

### Python GUI
```bash
cd gui_app
python motor_control_gui.py
```

### Command Line Interface
```bash
# Build and run the example
cd build
./multi_axes_control_test
```

## Motor Control Commands

### Basic Commands
- `C1Z` - Move Linear motor forward (jog)
- `D1Z` - Move Linear motor reverse (jog)
- `S1Z` - Stop Linear motor
- `F1Z` - Step Linear motor forward (incremental)
- `G1Z` - Step Linear motor reverse (incremental)

### Rotary Motor Commands
- `C2Z` - Move Rotary motor clockwise (jog)
- `D2Z` - Move Rotary motor counter-clockwise (jog)
- `S2Z` - Stop Rotary motor
- `F2Z` - Step Rotary motor clockwise (incremental)
- `G2Z` - Step Rotary motor counter-clockwise (incremental)
- `E2:length:cycles:Z` - Cyclic mode (length in mm, cycles 1-100)

### System Commands
- `RZ` - Reset system

## Configuration

### System Configuration
The system can be configured through the `system_config_t` structure:

```c
typedef struct {
    uint32_t max_frequency_linear;    // Max frequency for linear motor
    uint32_t max_frequency_rotary;    // Max frequency for rotary motor
    uint32_t microsteps_linear;       // Microsteps per mm for linear
    uint32_t microsteps_rotary;       // Microsteps per revolution for rotary
    float disc_radius;                // Disc radius for rotary calculations
    float circumference;              // Calculated circumference
    uint32_t cyclic_delay_ms;         // Delay between cyclic movements
} system_config_t;
```

### Register Configuration
Platform-specific register mappings are defined in `register_config.h`:

```c
typedef struct {
    volatile uint32_t *base_addr;    // Base address of peripheral
    uint32_t offset;                 // Register offset
    uint32_t mask;                   // Bit mask
    uint32_t shift;                  // Bit shift
} register_config_t;
```

## Adding New Platforms

### 1. Create Platform Directory
```bash
mkdir src/hal/platforms/your_platform
```

### 2. Implement HAL Functions
Create `hal_your_platform.c` and `hal_your_platform.h` implementing the HAL interface.

### 3. Add CMake Configuration
Create `CMakeLists.txt` in the platform directory with platform-specific settings.

### 4. Update Main CMakeLists.txt
Add the new platform to the platform selection logic.

## Testing

### Unit Tests
```bash
# Enable tests in CMake
cmake .. -DBUILD_TESTS=ON
cmake --build .
ctest
```

### Integration Tests
Run the example application and test motor control commands.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Authors

- **MultiAxesControl** - *Initial work* - [GitHub](https://github.com/multiaxescontrol)

## Acknowledgments

- FreeRTOS community for the excellent RTOS
- Arduino community for hardware abstraction ideas
- STM32 and ESP32 communities for platform support

## Changelog

### v1.0.0 (2025-01-10)
- Initial release
- Modular architecture implementation
- Arduino platform support
- FreeRTOS integration
- Python GUI interface
- CMake build system
