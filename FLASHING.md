# Flashing Guide for STM32G474RE (NUCLEO Board)

## Build Output Files

After building, you'll find the following files in `builds/build_blinky/executables/`:
- `blinky` - ELF executable (for debugging)
- `blinky.bin` - Raw binary for flashing
- `blinky.hex` - Intel HEX format
- `blinky.map` - Memory map (linker output)

## Memory Usage (Blinky)
- **Flash**: 3,680 bytes (0.7% of 512KB)
- **RAM**: 4,112 bytes (3% of 128KB)

## Flashing Methods

### Method 1: Using ST-Link Utility (Windows)
1. Download and install [STM32 ST-LINK Utility](https://www.st.com/en/development-tools/stsw-link004.html)
2. Connect your NUCLEO-STM32G474RE board via USB
3. Open ST-LINK Utility
4. Click **Target → Connect**
5. Click **Target → Program & Verify**
6. Browse to `builds/build_blinky/executables/blinky.bin`
7. Start address: `0x08000000`
8. Click **Start**

### Method 2: Using STM32CubeProgrammer (Recommended)
1. Download and install [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html)
2. Connect your NUCLEO board via USB
3. Open STM32CubeProgrammer
4. Select **ST-LINK** as connection method
5. Click **Connect**
6. Click **Open file** and select `builds/build_blinky/executables/blinky.hex`
7. Click **Download** to flash

### Method 3: Using OpenOCD (Cross-platform)
```bash
# Install OpenOCD first
# Windows: choco install openocd
# Linux: sudo apt-get install openocd

# Flash the firmware
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "program builds/build_blinky/executables/blinky.elf verify reset exit"
```

### Method 4: Drag & Drop (Easiest for Nucleo boards)
1. Connect your NUCLEO board
2. The board appears as a mass storage device (like a USB drive)
3. Simply **drag and drop** `blinky.bin` onto the drive
4. Wait for the LED to stop blinking (programming complete)
5. Press the black RESET button on the board

## Expected Behavior

After flashing successfully, you should see:
- **Green LED (LD2)** on PA5 blinking at 1Hz (500ms ON, 500ms OFF)

## Board Details

- **Board**: NUCLEO-STM32G474RE
- **MCU**: STM32G474RET6
- **User LED**: LD2 (Green) → PA5
- **System Clock**: 170MHz (configured via PLL)
- **Debug Interface**: ST-LINK V2-1 (on-board)

## Troubleshooting

### LED not blinking:
1.  Ensure the board is powered (USB connected)
2. Check if ST-LINK drivers are installed
3. Press the black RESET button
4. Try reflashing with STM32CubeProgrammer

### Cannot connect to board:
1. Check USB cable (use a data cable, not charge-only)
2. Install/update ST-LINK drivers
3. Try a different USB port
4. Update ST-LINK firmware via STM32CubeProgrammer

### Build errors:
```bash
# Clean and rebuild
python build_blinky.py --clean
```

## Next Steps

Once blinky is working:
1. Build and test the full firmware with FreeRTOS:
   ```bash
   python build_firmware.py
   ```
2. Modify LED blink rate in `examples/blinky/src/main.c`
3. Add more features to the firmware

## Quick Reference Commands

```bash
# Build blinky
python build_blinky.py

# Clean build
python build_blinky.py --clean

# Build firmware (with FreeRTOS)
python build_firmware.py

# Build both targets
python build_all.py
```
