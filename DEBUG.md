# Debugging Guide for STM32G474RE

Complete guide to debugging your STM32G474RE-based projects with VS Code.

## Prerequisites

### Required Software

1. **VS Code** - [Download](https://code.visualstudio.com/)

2. **STM32CubeIDE** - [Download](https://www.st.com/en/development-tools/stm32cubeide.html)
   - Provides ST-Link GDB Server
   - Includes ST-Link drivers
   - **Note**: You don't need to use CubeIDE as an IDE, just install it for the tools

3. **ARM GCC Toolchain** - Already installed (you're using it for building)

4. **VS Code Extensions** (install from Extensions marketplace):
   - **Cortex-Debug** (by marus25) - *Required*
   - **C/C++** (by Microsoft) - Recommended
   - **CMake Tools** - Recommended

### Hardware

- NUCLEO-STM32G474RE board
- USB cable (data cable, not charge-only)

---

## Quick Start

### 1. Install Cortex-Debug Extension

1. Open VS Code
2. Press `Ctrl+Shift+X` (Extensions)
3. Search for "Cortex-Debug"
4. Install "Cortex-Debug" by **marus25**

### 2. Download SVD File (Optional but Recommended)

The SVD file enables peripheral register viewing.

**Option A: From STM32CubeIDE** (if installed):
```powershell
# Find the file at:
C:\ST\STM32CubeIDE\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.productdb.debug_*\resources\cmsis\STMicro\STM32G4\STM32G474xx.svd

# Copy to:
.vscode\STM32G474.svd
```

**Option B: From GitHub**:
- Visit: https://github.com/modm-io/cmsis-svd-stm32
- Navigate to `STM32G4/` folder
- Download `STM32G474xx.svd`
- Save to `.vscode/STM32G474.svd`

### 3. Start Debugging

1. **Connect** your NUCLEO board via USB
2. **Open** the project in VS Code
3. **Press F5** or click the "Run and Debug" icon (Ctrl+Shift+D)
4. **Select** "Debug Blinky (ST-Link)" from the dropdown
5. **Done!** Debugger should stop at `main()`

---

## Debug Configurations

The project includes three debug configurations:

### 1. Debug Blinky (ST-Link)
- Debugs the simple LED blink example
- Automatically builds before debugging
- Stops at `main()` function

### 2. Debug Firmware (ST-Link)
- Debugs the full FreeRTOS-based firmware
- Includes RTOS thread awareness
- Shows FreeRTOS tasks in debug view

### 3. Attach to Target (ST-Link)
- Attaches to a running program
- Useful for debugging without resetting
- No automatic build

---

## Debug Features

### Breakpoints

- **Set breakpoint**: Click left of line number (red dot appears)
- **Conditional breakpoint**: Right-click breakpoint → Edit Breakpoint
- **Function breakpoint**: Debug panel → Breakpoints → + (function name)

**Keyboard Shortcuts**:
- `F9` - Toggle breakpoint
- `F5` - Continue
- `F10` - Step over
- `F11` - Step into
- `Shift+F11` - Step out

### Variable Inspection

- **Watch**: Add variables to watch panel (Debug sidebar)
- **Hover**: Mouse over variable in code
- **Locals**: Automatic in "Variables" section
- **Modify**: Right-click variable → Set Value

### Peripheral Registers

> [!NOTE]
> Requires SVD file (see Quick Start Step 2)

1. Start debugging session
2. Open **Cortex Peripherals** view (bottom of debug sidebar)
3. Expand peripherals (GPIO, TIM, USART, etc.)
4. View/modify register values in real-time

Example - Check LED pin state:
```
Cortex Peripherals → GPIOA → ODR → ODR5 (should toggle)
```

### Call Stack

- View function call hierarchy
- Click any frame to see variables at that point
- Located in Debug sidebar

### SWO Trace (Printf Debugging)

SWO allows `printf()` output via the debug probe:

1. **Add to your code**:
```c
#include <stdio.h>

int _write(int file, char *ptr, int len) {
    for (int i = 0; i < len; i++) {
        ITM_SendChar((*ptr++));
    }
    return len;
}

// Now printf works!
printf("Debug: value = %d\n", my_variable);
```

2. **View output**: SWO console appears in debug terminal

---

## FreeRTOS Debugging (Firmware Target)

When debugging the firmware target with FreeRTOS:

### Thread View

- **Threads Panel**: Shows all FreeRTOS tasks
- **Switch threads**: Click different tasks to see their stack
- **Task states**: Running, Ready, Blocked, Suspended

### Task Information

Each thread shows:
- Task name
- Current state
- Priority
- Stack usage

---

## Troubleshooting

### ST-Link Not Found

**Symptoms**: "Error: Could not find ST-Link"

**Solutions**:
1. Check USB cable (try different port/cable)
2. Install ST-Link drivers:
   - From STM32CubeIDE installation
   - Or download from ST website
3. Update ST-Link firmware via STM32CubeProgrammer
4. Check Device Manager (should show "STMicroelectronics STLink Virtual COM Port")

### Breakpoints Not Hitting

**Symptoms**: Breakpoint turns grey or is never hit

**Solutions**:
1. Verify code is actually executed
2. Check optimization level (high optimization can skip code)
3. Ensure executable matches source code (rebuild)
4. Try setting breakpoint earlier in code flow

### Build Fails Before Debug

**Symptoms**: Pre-launch task fails

**Solutions**:
```bash
# Clean and rebuild manually
python build_blinky.py --clean

# Or build firmware
python build_firmware.py --clean
```

### Cannot Connect to GDB Server

**Symptoms**: "Failed to launch GDB server"

**Solutions**:
1. Verify STM32CubeIDE installed correctly
2. Check `.vscode/settings.json` paths:
   ```json
   "cortex-debug.stlinkPath": "C:/ST/STM32CubeIDE/.../stlink-gdb-server.../tools/bin"
   ```
3. Try manual path if CubeIDE in different location

### Reset Issues

**Symptoms**: Target doesn't reset or runs old code

**Solutions**:
1. Press black RESET button on NUCLEO board
2. Power cycle the board
3. In launch.json, ensure `"runToEntryPoint": "main"`

### Peripheral Registers Not Visible

**Symptoms**: Cortex Peripherals view empty or missing

**Solutions**:
1. Ensure SVD file downloaded (see Quick Start #2)
2. Check `.vscode/launch.json`:
   ```json
   "svdFile": "${workspaceFolder}/.vscode/STM32G474.svd"
   ```
3. Restart debug session

---

## Advanced Topics

### Memory View

View raw memory during debugging:

1. Debug sidebar → "+ Add Memory" 
2. Enter address (e.g., `0x08000000` for Flash start)
3. View hex/ASCII representation

### Disassembly View

See generated assembly code:

1. Right-click in editor → "Open Disassembly View"
2. Step through assembly instructions
3. Useful for understanding compiler output

### Live Expressions

Create expressions evaluated in real-time:

1. Debug panel → Watch → +
2. Enter any C expression: `my_struct.field`, `array[5]`, etc.
3. Updates automatically when paused

---

## Tips & Tricks

### Quick Debug Workflow

```
F5        → Start debugging
F9        → Set breakpoint at current line
F10       → Step to next line
Ctrl+K,I  → Inspect variable (hover)
Shift+F5  → Stop debugging
```

### Restart Without Rebuilding

- Use "Attach to Target" configuration
- Useful when just testing different inputs

### Debug Multiple Targets

1. Build both: `python build_all.py`
2. Switch between configs in debug dropdown
3. No need to rebuild if only changing configuration

### Save Debug Session

- Breakpoints persist across sessions
- Watch variables are saved
- Stored in workspace settings

---

## Quick Reference

| Action | Shortcut | Alternative |
|--------|----------|-------------|
| Start Debug | `F5` | Debug panel → Play |
| Stop Debug | `Shift+F5` | Debug toolbar → Stop |
| Toggle Breakpoint | `F9` | Click line gutter |
| Step Over | `F10` | Debug toolbar |
| Step Into | `F11` | Debug toolbar |
| Step Out | `Shift+F11` | Debug toolbar |
| Continue | `F5` | Debug toolbar |
| Restart | `Ctrl+Shift+F5` | Debug toolbar → Restart |

---

## Next Steps

- **Add more debug output**: Implement SWO printf
- **Profile performance**: Use SWO timing
- **Inspect peripherals**: Use SVD peripheral viewer
- **Debug crashes**: Check call stack and registers

For flashing (without debugging), see [FLASHING.md](FLASHING.md).
