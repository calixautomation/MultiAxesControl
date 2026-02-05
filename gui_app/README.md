# Motor Control GUI

A Python GUI application for controlling linear and rotary motor stages via serial communication.

## Requirements

- Python 3.x
- pyserial

Install dependencies:
```bash
pip install pyserial
```

## Running the Application

```bash
python motor_control_app.py
```

## Features

### COM Port Connection
1. Select a COM port from the dropdown (ports with Arduino/CH340/USB Serial are labeled as "Motor Control Box")
2. Click **Connect** to establish serial connection (9600 baud, even parity)
3. Use **Reset COM Port** to disconnect and refresh available ports
4. Use **Reset Device** to send a reset command to the connected device

### Stage Selection
- **Linear Stage**: Controls motor 1 with forward/reverse movement
- **Rotary Stage**: Controls motor 2 with clockwise/counter-clockwise movement, plus cyclical mode

### Control Modes

#### Jog Mode
Hold the button to move continuously, release to stop.
- Linear: Move Forward / Move Reverse
- Rotary: Move Clockwise / Move Counter Clockwise

#### Incremental Mode
Click to move one step.
- Linear: Step Forward / Step Reverse
- Rotary: Step Clockwise / Step Counter Clockwise

#### Cyclical Mode (Rotary Only)
Runs automated back-and-forth cycles.
- **Arc Length**: Distance per cycle (0-10 mm)
- **No. of Cycles**: Number of cycles to run (1-100)
- Click **Run** to execute

### Serial Monitor
Displays incoming messages from the device. Use **Clear Terminal** to clear the display.

## Serial Protocol

| Command | Description |
|---------|-------------|
| `C{id}Z` | Start continuous movement (forward/CW) |
| `D{id}Z` | Start continuous movement (reverse/CCW) |
| `S{id}Z` | Stop movement |
| `F{id}Z` | Step forward/CW |
| `G{id}Z` | Step reverse/CCW |
| `RZ` | Reset device |
| `E2:{len}:{cycles}:Z` | Run cyclical mode |

Where `{id}` is `1` for linear stage or `2` for rotary stage.
