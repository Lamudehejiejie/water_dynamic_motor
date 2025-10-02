# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a PlatformIO project for M5Stack CoreS3 that controls a water dynamic motor system. The application sends motor position commands via serial communication using a custom protocol.

## Build System

This project uses **PlatformIO** for building and uploading firmware.

### Common Commands

```bash
# Build the project
pio run

# Upload to M5Stack CoreS3
pio run --target upload

# Build and upload in one command
pio run -t upload

# Open serial monitor (115200 baud)
pio device monitor

# Clean build files
pio run --target clean

# Upload and monitor
pio run -t upload && pio device monitor
```

## Hardware Configuration

- **Platform**: ESP32 (espressif32)
- **Board**: M5Stack CoreS3
- **Framework**: Arduino
- **Monitor Speed**: 115200 baud

## Dependencies

- `m5stack/M5Unified` - M5Stack unified library for hardware abstraction
- `m5stack/M5GFX` - Graphics library for M5Stack displays

## Code Architecture

### Motor Control Protocol

The system implements a custom motor control protocol that sends 6-byte commands:

```
[0x01, 0x06, 0x00, 0x00, posHigh, posLow]
```

- Position is a 16-bit value (0-1800 range)
- Position is split into high and low bytes
- Commands are sent via M5.Lcd.write() every 50ms

### Motion Pattern

The current implementation creates a cyclic motion:
- 0-2500ms: Position ramps from 0 to 1800
- 2500-5000ms: Position ramps from 1800 to 0
- The pattern repeats continuously

This timing-based approach uses `millis()` and `map()` to calculate positions, with the cycle resetting after 5000ms (note: current implementation doesn't reset `current_time`, which may cause overflow issues).

## Project Structure

- `src/main.cpp` - Main application entry point with setup() and loop()
- `include/` - Header files directory (currently empty)
- `lib/` - Custom libraries directory (currently empty)
- `test/` - Unit tests directory (currently empty)
- `platformio.ini` - PlatformIO configuration file
