# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Upload

This is a standard Arduino IDE project targeting the **Arduino Mega 2560**. There is no Makefile or CMake — the Arduino IDE auto-discovers all `.ino`, `.cpp`, and `.h` files in the project directory.

**To build and upload:**
1. Open `YoghurtPastuerizer.ino` in Arduino IDE
2. Select board: `Arduino Mega or Mega 2560`
3. Select the correct COM/serial port
4. Click **Upload** (or `Ctrl+U`)

**Required libraries** (install via Arduino Library Manager):
- `U8g2` — graphics LCD rendering
- `Keypad` — matrix keypad scanning
- `RTClib` (Adafruit) — DS3231 RTC driver
- `Wire`, `EEPROM` — bundled with Arduino IDE

## Architecture

The firmware implements a **multi-phase pasteurization state machine** across 7 modules:

| Module | Files | Role |
|--------|-------|------|
| Main loop | `YoghurtPastuerizer.ino` | FSM dispatcher — calls per-state handlers each `loop()` tick |
| State machine | `states_state_machine.*` | State enum, global vars, button debounce, buzzer, emergency stop |
| Sensors | `sensors_temperature.*` | Dual thermistor reads (Steinhart-Hart), 5-sample EMI filtering, fault detection |
| Heater control | `control_heater.*` | Relay switching for heater/agitator/cooling valve; hysteresis logic |
| Display | `display_lcd.*` | U8g2-based 128×64 LCD menu system (11 screen states) |
| Keypad UI | `ui_keypad.*` | 4×3 matrix keypad, menu navigation, parameter entry |
| Memory | `memory_storage.*` | EEPROM settings + run-state checkpoints; RTC-based outage timing |
| Config | `config_settings.h` | All pin assignments and compile-time constants — edit here first |

**Process flow:**
```
SYSTEM_IDLE → HEATING → COOLING → MIXING → HOLDING → COMPLETE
                                                ↓ (fault/emergency stop)
                                           FAULT_ERROR
```

### Key design decisions

- **Hysteresis control** (not true PID): heater cuts out 10°C early to coast on residual heat; cooling valve cuts out 7°C early. The holding phase uses tiered duty cycles (5 s/15 s/25 s on-time) based on deviation bands.
- **Power-loss recovery**: on boot, the firmware checks EEPROM for an interrupted run and automatically resumes from where it left off — no operator confirmation needed. Checkpoints write every 30 s to protect EEPROM endurance.
- **Sensor fault**: 10 consecutive readings of 999.0°C (open circuit) trigger emergency shutdown.
- **Relay safety**: all relay pins are driven HIGH (off) before `pinMode()` to prevent spurious actuation at boot.

### Hardware pin map (from `config_settings.h`)

| Function | Pin(s) |
|----------|--------|
| START / STOP buttons | 2, 3 |
| Buzzer | 6 |
| Heater / Agitator / Cooling relays | 7, 8, 9 |
| Thermistor — jacket / vessel | A0, A1 |
| Keypad rows / cols | 22–25 / 26–28 |
| LCD (CS, SID, SCLK, RST) | 29–32 |
| I2C (SDA/SCL for RTC) | 20, 21 (hardware fixed) |

### Default process parameters (set in `ui_keypad.cpp`)

| Phase | Target temp | Hold duration |
|-------|------------|---------------|
| Heating | 85 °C | — |
| Cooling | 45 °C | — |
| Mixing | — | 2 min |
| Holding | 45 °C | 8 hours |

To tune thermal overshoot/undershoot, adjust the hysteresis offsets and duty-cycle thresholds in `control_heater.cpp`. To tune sensor calibration, adjust the probe offset constants in `config_settings.h`.
