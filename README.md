# MultiAxesControl

A modular multi-axis motor control system, designed for precise control of stepper motors movement for industrial applications

## Features

- **Dual-Axis Control**: Linear and rotary stepper motor control and scalable
- **Multiple Operating Modes**: Jog, incremental, cyclic movement
- **FreeRTOS Integration**: Real-time task management
- **Modular Architecture**: Clean separation between HAL, OS, and application layers
- **VS Code Debugging**: Full Cortex-Debug integration with peripheral viewing

## Hardware

| Component | Specification |
|-----------|---------------|
| MCU | STM32G474RE @ 170MHz |
| Board | NUCLEO-G474RE |
| Flash/RAM | 512KB / 128KB |
| Motor 1 (Linear) | TIM2, PA0=STEP, PA1=DIR, 1600 microsteps, max 200Hz |
| Motor 2 (Rotary) | TIM3, PA2=STEP, PA3=DIR, 6400 microsteps, max 36Hz |
| UART | USART2 @ 9600 baud, 8E1 |
| Status LED | PA5 (LD2) |
| Watchdog | IWDG, 2s timeout |

## Project Structure

```
MultiAxesControl/
├── firmware/              # Main FreeRTOS application
│   └── src/main.c
├── examples/blinky/       # Simple LED blink example
├── application/           # Motor control business logic
│   ├── motor_control.h
│   └── motor_control.c
├── hal/                   # Hardware Abstraction Layer
│   ├── hal_interface.h
│   └── platforms/stm32/
├── os/                    # FreeRTOS task management
│   ├── task_manager.h
│   └── task_manager.c
├── config/                # FreeRTOS configuration
├── scripts/build/         # Python build scripts
└── .vscode/               # VS Code debug configuration
```

## Quick Start

### Prerequisites

1. **STM32CubeCLT** (Command Line Tools) - [Download](https://www.st.com/en/development-tools/stm32cubeclt.html)
   - Includes ARM GCC Toolchain, ST-LINK GDB Server, STM32CubeProgrammer CLI, and SVD files

2. **Set environment variable** `STM32CLT_PATH` pointing to your installation:

   **Windows** (run in PowerShell as Admin):
   ```powershell
   [Environment]::SetEnvironmentVariable("STM32CLT_PATH", "C:\ST\STM32CubeCLT_1.20.0", "User")
   ```

   **Linux/macOS** (add to `~/.bashrc` or `~/.zshrc`):
   ```bash
   export STM32CLT_PATH="/opt/st/stm32cubeclt_1.20.0"
   ```

   Restart VS Code after setting the variable.

3. **CMake 3.16+** - [Download](https://cmake.org/download/)

4. **Ninja build system** - [Download](https://ninja-build.org/) or `pip install ninja`

5. **Python 3.7+**

6. **Latest Git**

6. **VS Code Extensions**:
   - **Cortex-Debug** (by marus25) - Required for debugging
   - **C/C++** (by Microsoft) - Recommended for IntelliSense

### Build

```bash
### Initialize submodules
git submodule update --init --recursive

# Build main firmware
python scripts/build/build_firmware.py

# Build blinky example
mkdir -p builds/build_blinky && cd builds/build_blinky
cmake ../.. -G Ninja -DFWTYPE=blinky
ninja blinky

# Clean build
python scripts/build/build_firmware.py --clean
```

Output files are generated in `builds/build_firmware/executables/`:
- `firmware.elf` - Debug executable
- `firmware.bin` - Raw binary
- `firmware.hex` - Intel HEX format

---
## Flashing & Debugging

### Prerequisites

Debugging requires **STM32CubeCLT** and the `STM32CLT_PATH` environment variable (see Prerequisites above).

The `.vscode` files use `${env:STM32CLT_PATH}` to locate:
- `GNU-tools-for-STM32/bin/` - ARM GCC toolchain and GDB
- `STLink-gdb-server/bin/` - ST-LINK GDB server
- `STM32CubeProgrammer/bin/` - Flash programmer
- `STMicroelectronics_CMSIS_SVD/` - SVD files for peripheral viewing

### Setup

1. Install **Cortex-Debug** extension in VS Code (Ctrl+Shift+X → search "Cortex-Debug")
2. Ensure STM32CubeCLT is installed at the expected path
3. Connect NUCLEO board via USB

### Start Debugging

1. Press **Ctrl+Shift+D** in VS Code to open the debug extension
2. Select **"Debug Firmware (ST-Link)"** or **"Debug Blinky (ST-Link)"** and play
3. Debugger automatically builds, flashes, and stops at `main()`

### Debug Configurations

| Configuration | Description |
|---------------|-------------|
| Debug Firmware (ST-Link) | Full FreeRTOS firmware with thread awareness |
| Debug Blinky (ST-Link) | Simple LED blink example |
| Attach to Target | Attach without reset |

### Keyboard Shortcuts

| Action | Shortcut |
|--------|----------|
| Start/Continue | F5 |
| Toggle Breakpoint | F9 |
| Step Over | F10 |
| Step Into | F11 |
| Step Out | Shift+F11 |
| Stop | Shift+F5 |

### Features

- **Breakpoints**: Click left of line number
- **Variable Watch**: Debug sidebar → Watch → Add expression
- **Peripheral Registers**: Cortex Peripherals view (requires SVD)
- **FreeRTOS Threads**: View all tasks, states, and stack usage
- **Memory View**: Debug sidebar → Add Memory → Enter address
- **SWO Printf**: Add ITM_SendChar implementation for printf debugging

### SWO Printf Setup

Add to your code for printf via debug probe:

```c
#include <stdio.h>

int _write(int file, char *ptr, int len) {
    for (int i = 0; i < len; i++) {
        ITM_SendChar(*ptr++);
    }
    return len;
}

// Now printf() outputs to SWO console
printf("Debug: value = %d\n", my_variable);
```

---
## Standalone Flashing

### Method 1: Drag & Drop (Easiest)

1. Connect NUCLEO board via USB
2. Board appears as a USB drive
3. Drag `builds/build_firmware/executables/firmware.bin` to the drive
4. Wait for LED to stop blinking (programming complete)
5. Press black RESET button

### Method 2: STM32CubeProgrammer CLI (via STM32CubeCLT)

If you installed STM32CubeCLT and set `STM32CLT_PATH`, use:

```bash
# Windows (PowerShell)
& "$env:STM32CLT_PATH/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe" `
  -c port=SWD -w builds/build_firmware/executables/firmware.hex -v -rst

### Method 3: STM32CubeProgrammer GUI

1. Open STM32CubeProgrammer (installed with STM32CubeCLT or standalone)
2. Connect board via USB
3. Select **ST-LINK** connection, click **Connect**
4. Click **Open file** → select `firmware.hex`
5. Click **Download**

---

## Motor Commands

Commands are sent via UART (9600 baud, 8E1) and must end with `Z`.

### Movement Commands

| Command | Description |
|---------|-------------|
| `C1Z` | Motor 1 jog forward |
| `D1Z` | Motor 1 jog reverse |
| `C2Z` | Motor 2 jog clockwise |
| `D2Z` | Motor 2 jog counter-clockwise |
| `S1Z` | Stop Motor 1 |
| `S2Z` | Stop Motor 2 |
| `F1Z` / `F2Z` | Single step forward |
| `G1Z` / `G2Z` | Single step reverse |
| `E2:<arc>:<cycles>Z` | Cyclic mode (e.g., `E2:30:10Z` = 30mm arc, 10 cycles) |
| `RZ` | System reset |

### Example Session

```
> C1Z          # Start Motor 1 forward
> S1Z          # Stop Motor 1
> E2:30:5Z     # Motor 2 oscillate 30mm arc, 5 cycles
> RZ           # Reset system
```

---

## Architecture

### Layer Overview

```
┌─────────────────────────────────────────┐
│           main.c (Firmware)             │
│  System init, clock config, peripherals │
└─────────────┬───────────────────────────┘
              │
   ┌──────────┼──────────┬────────────┐
   │          │          │            │
   ▼          ▼          ▼            ▼
┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐
│  App   │ │   OS   │ │  HAL   │ │ Config │
│ Layer  │ │ Layer  │ │ Layer  │ │ Layer  │
└────────┘ └────────┘ └────────┘ └────────┘
```

**Application Layer** (`application/`): Pure business logic - command parsing, motor control state machine

**OS Layer** (`os/`): FreeRTOS tasks, queues, ISR handling

**HAL Layer** (`hal/`): Hardware abstraction - GPIO, timers, UART, watchdog

### FreeRTOS Tasks

| Task | Priority | Stack | Purpose |
|------|----------|-------|---------|
| UART Handler | 2 | 256 words | Process commands from queue |
| Motor Update | 1 | 256 words | Execute motor operations |
| Watchdog | 3 | 128 words | Refresh IWDG every 500ms |
| Status LED | 1 | 128 words | Blink PA5 LED |

### Command Flow

```
UART RX (ISR)
    └─> task_manager_uart_rx_isr()
        └─> xQueueSend(command_queue)
            └─> task_uart_handler()
                └─> motor_control_parse_command()
                └─> motor_control_process_command()
                    └─> hal_motor_start()
```

---

## Configuration

### FreeRTOS (`config/FreeRTOSConfig.h`)

| Setting | Value |
|---------|-------|
| `configTICK_RATE_HZ` | 1000 (1ms tick) |
| `configMINIMAL_STACK_SIZE` | 128 words |
| `configTOTAL_HEAP_SIZE` | 15KB |

### Motor Settings (`application/motor_control.h`)

```c
#define LINEAR_MAX_FREQ     200     // Hz
#define LINEAR_MICROSTEPS   1600    // steps/mm
#define ROTARY_MAX_FREQ     36      // Hz
#define ROTARY_MICROSTEPS   6400    // steps/rev
#define DISC_RADIUS         15.0    // mm
```

---

## Troubleshooting

### Build Issues

```bash
# Clean and rebuild
python scripts/build/build_firmware.py --clean
```

### ST-LINK Not Found

1. Check USB cable (use data cable, not charge-only)
2. Install ST-LINK drivers (included with STM32CubeIDE)
3. Update ST-LINK firmware via STM32CubeProgrammer
4. Check Device Manager for "STMicroelectronics STLink"

### Motor Not Moving

1. Verify UART settings: 9600 baud, 8 data bits, even parity, 1 stop bit
2. Commands must end with `Z`
3. Check motor driver power and connections
4. Verify step/direction pin connections (PA0/PA1 for Motor 1, PA2/PA3 for Motor 2)

### System Resets Frequently

1. Check watchdog is being refreshed (watchdog task running)
2. Verify stack sizes are sufficient
3. Check for blocking operations in tasks
4. Look for infinite loops

### Debug Session Won't Start

1. Verify Cortex-Debug extension installed
2. Verify STM32CubeCLT is installed
3. If installed elsewhere, update paths in:
   - `.vscode/settings.json` - `cortex-debug.armToolchainPath`, `cortex-debug.gdbPath`, `cortex-debug.stlinkPath`
   - `.vscode/launch.json` - `serverpath`, `stm32cubeprogrammer`, `svdFile`
4. Reload VS Code window (Ctrl+Shift+P → "Reload Window")
5. Verify board is connected and ST-LINK driver installed

### Breakpoints Not Hitting

1. Verify code is actually executed
2. Check optimization level (high optimization can skip code)
3. Ensure executable matches source (rebuild)
4. Try setting breakpoint earlier in code flow

---

## VS Code Settings

The project includes pre-configured VS Code settings:

| File | Purpose |
|------|---------|
| `.vscode/launch.json` | Debug configurations |
| `.vscode/tasks.json` | Build tasks |
| `.vscode/settings.json` | Toolchain paths, IntelliSense |

---

## Adding New Platforms

1. Create `hal/platforms/<platform>/CMakeLists.txt`
2. Implement `hal_<platform>.c` with HAL interface functions
3. Update `hal/CMakeLists.txt` to include new platform
4. Build with `-DPLATFORM=<platform>`

---

## License

MIT License - see LICENSE file.

## Acknowledgments

- STM32 HAL/LL Drivers - STMicroelectronics
- FreeRTOS Kernel - Amazon Web Services
- CMSIS - ARM Limited
