# Multi-Axis Motor Control Firmware

This directory contains the main firmware application for the STM32G474RE-based multi-axis motor control system.

## Overview

The firmware implements a sophisticated motor control system adapted from Arduino/AVR to STM32G4 with FreeRTOS. It supports dual-axis control (Linear + Rotary) with multiple operating modes.

## Architecture

### FreeRTOS Task Structure

The firmware uses a multi-task architecture for efficient real-time operation:

| Task Name | Priority | Stack Size | Description |
|-----------|----------|------------|-------------|
| **UART Task** | 2 | 256 bytes | Receives commands from queue and processes them |
| **Motor Task** | 1 | 512 bytes | Executes motor control operations |
| **Watchdog Task** | 3 (Highest) | 128 bytes | Refreshes hardware watchdog every 500ms |
| **Status LED Task** | 1 | 128 bytes | Blinks LED to indicate system is alive |

### Communication Flow

```
UART RX (ISR) → Command Queue → UART Task → Motor Control → Motor Task
                                      ↓
                                Task Notification
```

## System Configuration

### Hardware Resources

- **MCU**: STM32G474RE @ 170MHz
- **Motors**: 2x Stepper motors with TB6600 drivers
  - Motor 1 (Linear): 1600 microsteps, max 200Hz
  - Motor 2 (Rotary): 6400 microsteps, max 36Hz
- **UART**: USART2 @ 9600 baud, 8E1 (8 data bits, even parity, 1 stop bit)
- **Watchdog**: IWDG with 2-second timeout
- **Status LED**: PA5 (500ms blink rate)

### Physical Constants

- **Disc Radius**: 15.0 mm
- **Circumference**: ~94.25 mm (2π × radius)

## Operating Modes

### 1. JOG Mode (Continuous Movement)
Continuous motor movement in specified direction until stopped.

**Commands:**
- `C1Z` - Motor 1 clockwise
- `D1Z` - Motor 1 counter-clockwise
- `C2Z` - Motor 2 clockwise
- `D2Z` - Motor 2 counter-clockwise

### 2. STOP Mode
Stops motor movement immediately.

**Command:**
- `S1Z` - Stop Motor 1
- `S2Z` - Stop Motor 2

### 3. CYCLIC Mode (Automated Back-and-Forth)
Performs automated oscillation for specified arc length and number of cycles.
Only supported on Motor 2 (Rotary).

**Command Format:** `E2:<arc_length>:<cycles>Z`

**Example:**
- `E2:30:10Z` - Move 30mm arc, 10 cycles (CW → delay → CCW → delay, repeated 10 times)

**Parameters:**
- Arc length: in millimeters (converted to steps based on circumference)
- Cycles: number of back-and-forth iterations
- Delay: 500ms between each movement

### 4. INCREMENTAL Mode (Single Step)
Executes a single step movement.

**Commands:**
- `F1Z` - Motor 1 single step forward
- `G1Z` - Motor 1 single step reverse
- `F2Z` - Motor 2 single step forward
- `G2Z` - Motor 2 single step reverse

### 5. System Reset
Performs a complete system reset.

**Command:**
- `RZ` - System reset (motor ID not required)

## Command Protocol

### Format
All commands follow the pattern: `<CMD><MOTOR_ID><PARAMS>Z`

- **CMD**: Single character command (C, D, S, E, F, G, R)
- **MOTOR_ID**: Motor number (1 or 2)
- **PARAMS**: Optional parameters (e.g., `:30:10` for cyclic mode)
- **Z**: Command terminator (required)

### Command Termination
Commands must end with 'Z' character. The UART ISR buffers incoming characters until:
1. 'Z' terminator is received, or
2. Buffer is full (16 characters)

### Error Handling
- Invalid commands return: `"Invalid Command\n"`
- Invalid motor ID returns: `"Invalid Motor\n"`
- UART errors return: `"UART Error\n"`
- Queue full returns: `"Queue Full\n"`

## Startup Sequence

1. **HAL Initialization**: Initialize STM32 HAL library
2. **Clock Configuration**: Configure system clock to 170MHz
3. **GPIO Initialization**: Setup LED and other pins
4. **UART Initialization**: Configure USART2 for communication
5. **Motor Control Init**: Initialize motor control subsystem
6. **Queue Creation**: Create command queue (10 commands × 16 bytes)
7. **Task Creation**: Create all FreeRTOS tasks
8. **Watchdog Init**: Initialize and start IWDG
9. **Scheduler Start**: Start FreeRTOS scheduler

### Startup Messages
```
System is preparing...
System is Ready
```

## Integration with Application Layer

The firmware integrates with the `motor_control` application layer:

### Key Functions Called
- `motor_control_init()` - Initialize motor control subsystem
- `motor_control_parse_command()` - Parse command string
- `motor_control_process_command()` - Execute motor command
- `motor_control_get_motor_state()` - Get current motor state

### Data Structures
- `motor_command_t` - Command structure
- `motor_state_t` - Motor state information
- `hal_status_t` - Return status codes

## Debug Mode

Uncomment `#define DEBUG_CODE` to enable command echo:
```c
#define DEBUG_CODE
```

When enabled, each received command is echoed back:
```
Cmd: C1Z
```

## Safety Features

### Watchdog Timer
- **Type**: Independent Watchdog (IWDG)
- **Timeout**: 2 seconds
- **Refresh Rate**: Every 500ms (high-priority task)
- **Purpose**: System recovery from hangs or crashes

### Error Detection
- UART parity, framing, and overrun error detection
- Queue overflow protection
- Command validation
- Parameter range checking

### FreeRTOS Hooks
- **Idle Hook**: Sends "System is Ready" message once
- **Malloc Failed Hook**: Halts system on memory allocation failure
- **Stack Overflow Hook**: Halts system on stack overflow
- **Assert Hook**: Halts system on assertion failure

## Build and Flash

See the main project documentation:
- [BUILD.md](../BUILD.md) - Build instructions
- [FLASHING.md](../FLASHING.md) - Flashing instructions
- [DEBUG.md](../DEBUG.md) - Debugging setup

## Testing

### Basic Communication Test
```
> C1Z
(Motor 1 should start moving clockwise)

> S1Z
(Motor 1 should stop)
```

### Cyclic Mode Test
```
> E2:30:5Z
(Motor 2 should oscillate 30mm arc, 5 times)
```

### System Reset Test
```
> RZ
System Reset...
(System should reboot)
```

## Differences from Arduino Version

| Feature | Arduino/AVR | STM32G4 |
|---------|-------------|---------|
| **MCU** | ATmega328P @ 16MHz | STM32G474RE @ 170MHz |
| **RTOS** | Arduino FreeRTOS | FreeRTOS Kernel |
| **Timers** | Timer0/Timer2 (8-bit) | TIM2/TIM3 (32-bit) |
| **UART** | USART0 | USART2 |
| **Watchdog** | AVR WDT | IWDG (Independent) |
| **ISR** | AVR ISR macros | CMSIS IRQ handlers |
| **Reset** | `wdt_enable()` + loop | `NVIC_SystemReset()` |
| **Delays** | `_delay_ms()` | `HAL_Delay()` / `vTaskDelay()` |

## Future Enhancements

- [ ] Add velocity control (acceleration/deceleration ramps)
- [ ] Implement position feedback
- [ ] Add homing/limit switch support
- [ ] Support for additional motors (3rd, 4th axis)
- [ ] CAN bus communication support
- [ ] Parameter configuration via UART
- [ ] Non-volatile storage for settings
- [ ] Emergency stop functionality

## Troubleshooting

### System doesn't respond to commands
- Check UART connection (PA2=TX, PA3=RX)
- Verify baud rate is 9600
- Ensure commands end with 'Z'
- Check parity is set to EVEN

### Motor doesn't move
- Verify motor control initialization succeeded
- Check motor driver connections
- Ensure motor driver is powered
- Check step/direction pins

### System resets frequently
- Check watchdog is being refreshed
- Verify stack sizes are sufficient
- Look for stack overflow conditions
- Check for infinite loops blocking tasks

### LED not blinking
- Verify PA5 is connected to LED
- Check LED polarity
- Ensure FreeRTOS scheduler started

## License

See main project LICENSE file.

## Authors

MultiAxesControl Team - 2026
