# EcoBin Smart Waste Management System

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![Platform](https://img.shields.io/badge/platform-ATmega328P-green.svg)
![Build](https://img.shields.io/badge/build-passing-brightgreen.svg)

An intelligent embedded waste management system featuring real-time fill monitoring, fire detection, and automated alerts. Built for ATmega328P microcontroller (Arduino Uno compatible) with complete Proteus simulation support.

---

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Pin Configuration](#pin-configuration)
- [Installation](#installation)
- [Building the Firmware](#building-the-firmware)
- [Proteus Simulation](#proteus-simulation)
- [Hardware Assembly](#hardware-assembly)
- [Usage](#usage)
- [System States](#system-states)
- [Troubleshooting](#troubleshooting)
- [Memory Optimization](#memory-optimization)
- [Contributing](#contributing)
- [License](#license)
- [Author](#author)

---

## 🎯 Overview

**EcoBin** is a smart waste bin management system designed for embedded systems coursework. It monitors waste fill levels, detects fire hazards, and provides real-time alerts through an intuitive LCD interface with PIN-protected maintenance mode.

### Key Highlights

- **Multi-threshold monitoring**: Primary (70%), Critical (90%), Full (95%)
- **Fire detection**: Integrated flame sensor with emergency shutdown
- **Secure maintenance**: 4-digit PIN protection (default: 1234)
- **Event logging**: Circular buffer storing last 20 events
- **Professional UI**: 16×2 LCD with dynamic status display
- **Modular architecture**: HAL, device drivers, and application layers

---

## ✨ Features

### Core Functionality

- ✅ **Ultrasonic Distance Measurement** (HC-SR04)
  - Accurate fill level calculation (0-100%)
  - Configurable calibration parameters
  - Noise filtering and validation

- ✅ **Multi-Level Alerts**
  - PRIMARY: 70% full (LED blinks, single beep)
  - CRITICAL: 90% full (LED steady, continuous beep)
  - FULL: 95% full (locked state, manual reset required)

- ✅ **Fire Detection**
  - Real-time flame sensor monitoring
  - Emergency state with high-priority alerts
  - Automatic system override

- ✅ **User Interface**
  - 16×2 LCD with real-time updates
  - 4×3 matrix keypad for PIN entry
  - Visual and audio feedback

- ✅ **Maintenance Mode**
  - PIN-protected access (default: 1234)
  - Sensor calibration interface
  - Event log viewing
  - Manual system reset

- ✅ **Event Logging**
  - 20-event circular buffer
  - Timestamped entries
  - Fill level snapshots
  - Event type categorization

---

## 🛠️ Hardware Requirements

### Components List

| Component | Specification | Quantity |
|-----------|--------------|----------|
| Microcontroller | ATmega328P (Arduino Uno) | 1 |
| LCD Display | 16×2 HD44780 (I²C optional) | 1 |
| Ultrasonic Sensor | HC-SR04 | 1 |
| Flame Sensor | Digital flame detector module | 1 |
| Matrix Keypad | 4×3 membrane keypad | 1 |
| Buzzer | Active 5V buzzer | 1 |
| LED | Standard 5mm LED (any color) | 1 |
| Resistors | 220Ω (LED), 10kΩ (keypad pullups) | Multiple |
| Breadboard | 830 tie-points or larger | 1 |
| Jumper Wires | Male-to-male, male-to-female | Assorted |
| Power Supply | 5V regulated (USB or adapter) | 1 |

### Optional Components

- 10kΩ potentiometer (LCD contrast adjustment)
- Status indicator LEDs (power, activity)
- External power module for standalone operation

---

## 💻 Software Requirements

### Development Environment

- **Atmel Studio 7.0** (primary IDE)
  - Download: [Microchip Atmel Studio](https://www.microchip.com/mplab/avr-support/atmel-studio-7)
  - Alternative: Arduino IDE with AVR toolchain

- **Proteus Design Suite 8.x** (simulation)
  - Professional or Educational license
  - Arduino library for Proteus installed

### Toolchain

- AVR-GCC compiler (included with Atmel Studio)
- AVRdude (for firmware upload)
- Make utility (for build automation)

### Optional Tools

- Git (version control)
- Serial monitor (PuTTY, Arduino Serial Monitor)
- Logic analyzer software (debugging)

---

## 📌 Pin Configuration

### ATmega328P Pin Mapping

#### LCD (16×2) - 4-bit Mode
| LCD Pin | ATmega328P Pin | Arduino Pin | Function |
|---------|----------------|-------------|----------|
| RS | PD2 | D2 | Register Select |
| EN | PD3 | D3 | Enable |
| D4 | PD4 | D4 | Data Bit 4 |
| D5 | PD6 | D6 | Data Bit 5 |
| D6 | PD7 | D7 | Data Bit 6 |
| D7 | PB0 | D8 | Data Bit 7 |

#### Ultrasonic Sensor (HC-SR04)
| HC-SR04 Pin | ATmega328P Pin | Arduino Pin |
|-------------|----------------|-------------|
| TRIG | PB2 | D10 |
| ECHO | PB1 | D9 |

#### Flame Sensor
| Sensor Pin | ATmega328P Pin | Arduino Pin |
|------------|----------------|-------------|
| DO | PB3 | D11 |

#### Keypad (4×3 Matrix)
| Keypad Pin | ATmega328P Pin | Arduino Pin | Function |
|------------|----------------|-------------|----------|
| ROW1 | PC0 | A0 | Row 1 (1, 2, 3) |
| ROW2 | PC1 | A1 | Row 2 (4, 5, 6) |
| ROW3 | PC2 | A2 | Row 3 (7, 8, 9) |
| ROW4 | PC3 | A3 | Row 4 (*, 0, #) |
| COL1 | PC4 | A4 | Column 1 |
| COL2 | PC5 | A5 | Column 2 |
| COL3 | PD5 | D5 | Column 3 |

#### Buzzer & LED
| Component | ATmega328P Pin | Arduino Pin |
|-----------|----------------|-------------|
| Buzzer | PB4 | D12 |
| LED | PB5 | D13 |

### Power Connections
- **VCC**: Pin 7, Pin 20 (AVCC) → 5V
- **GND**: Pin 8, Pin 22 → Common Ground
- **AREF**: Pin 21 → Connect to VCC (for ADC reference)

---

## 🚀 Installation

### 1. Clone Repository

```bash
# Clone the repository
git clone https://github.com/yourusername/ecobin-smart-waste.git

# Navigate to project directory
cd ecobin-smart-waste
```

### 2. Set Up Atmel Studio Project

```bash
# Open Atmel Studio 7.0
# File → Open → Project/Solution
# Select: ecobin_firmware/ecobin_firmware.atsln
```

### 3. Configure Project Settings

**In Atmel Studio:**

1. Right-click project → **Properties**
2. **Toolchain** → **AVR/GNU C Compiler** → **Symbols**
3. Add symbol: `F_CPU=16000000UL`
4. **Device** tab → Select `ATmega328P`
5. **Device Frequency**: `16.000000 MHz`
6. Click **OK**

---

## 🔨 Building the Firmware

### Method 1: Atmel Studio GUI

1. Open project in Atmel Studio
2. **Build** → **Clean Solution**
3. **Build** → **Rebuild Solution** (or press `F7`)
4. Check **Output** window for success message

### Method 2: Command Line (Make)

```bash
# Navigate to project directory
cd ecobin_firmware/

# Clean previous builds
make clean

# Build firmware
make all

# Expected output files:
# - ecobin_firmware.elf (executable)
# - ecobin_firmware.hex (flash image)
# - ecobin_firmware.eep (EEPROM data)
```

### Verify Build Success

```
AVR Memory Usage
----------------
Device: ATmega328P

Program:    8542 bytes (26.1% Full)
Data:        987 bytes (48.2% Full)

Build succeeded: 0 errors, 0 warnings
```

---

## 🖥️ Proteus Simulation

### Component Search Keywords

| Component | Proteus Keyword | Alternative |
|-----------|----------------|-------------|
| Microcontroller | `ATMEGA328P` | `ARDUINO UNO R3` |
| LCD | `LM016L` | `LCD 16x2` |
| Ultrasonic | `HC-SR04` | Search: "ultrasonic" |
| Flame Sensor | `FLAME SENSOR` | Use generic comparator |
| Keypad | `KEYPAD 4X3` | `MEMBRANE KEYPAD` |
| Buzzer | `BUZZER` | `ACTIVE BUZZER` |
| LED | `LED-RED` | Any color LED |

### Simulation Setup Steps

1. **Create New Project**: File → New Project
2. **Add Components**: Use search keywords above
3. **Wire Components**: Follow pin configuration table
4. **Load Firmware**: 
   - Right-click ATmega328P → Edit Properties
   - Program File → Browse to `ecobin_firmware.hex`
   - Clock Frequency: `16MHz`
5. **Run Simulation**: Click Play button (F12)

### Testing in Proteus

- **Fill Level**: Adjust HC-SR04 distance (right-click sensor)
- **Fire Detection**: Toggle flame sensor state
- **Keypad Input**: Click keypad buttons
- **LCD Output**: Observe real-time display updates
- **Alerts**: Listen for buzzer, watch LED blink patterns

---

## 🔧 Hardware Assembly

### Breadboard Assembly Guide

#### Step 1: Power Distribution
```
1. Connect Arduino 5V pin → Breadboard positive rail (red)
2. Connect Arduino GND pin → Breadboard ground rail (black)
3. Add capacitor (100µF) across power rails (noise filtering)
```

#### Step 2: LCD Connection
```
1. LCD VSS (GND) → Ground rail
2. LCD VDD (VCC) → Positive rail (5V)
3. LCD V0 (Contrast) → Middle pin of 10kΩ pot
4. LCD RS → ATmega328P PD2 (D2)
5. LCD RW → Ground (write mode only)
6. LCD EN → ATmega328P PD3 (D3)
7. LCD D4-D7 → ATmega328P PD4, PD6, PD7, PB0
8. LCD A (Backlight+) → Positive rail via 220Ω resistor
9. LCD K (Backlight-) → Ground rail
```

#### Step 3: Ultrasonic Sensor
```
1. HC-SR04 VCC → Positive rail
2. HC-SR04 GND → Ground rail
3. HC-SR04 TRIG → ATmega328P PB2 (D10)
4. HC-SR04 ECHO → ATmega328P PB1 (D9)
```

#### Step 4: Flame Sensor
```
1. Flame sensor VCC → Positive rail
2. Flame sensor GND → Ground rail
3. Flame sensor DO → ATmega328P PB3 (D11)
```

#### Step 5: Matrix Keypad
```
1. Row pins (4 wires) → PC0, PC1, PC2, PC3
2. Column pins (3 wires) → PC4, PC5, PD5
3. Add 10kΩ pullup resistors on column pins to 5V
```

#### Step 6: Buzzer & LED
```
1. Buzzer (+) → ATmega328P PB4 (D12)
2. Buzzer (-) → Ground rail
3. LED Anode → ATmega328P PB5 (D13) via 220Ω resistor
4. LED Cathode → Ground rail
```

### Upload Firmware to Hardware

```bash
# Using AVRdude (Arduino bootloader)
avrdude -p atmega328p -c arduino -P COM3 -b 115200 -U flash:w:ecobin_firmware.hex:i

# Replace COM3 with your port (Linux: /dev/ttyUSB0, Mac: /dev/cu.usbserial)
```

---

## 📖 Usage

### Power On Sequence

1. **Connect power** → System initializes LCD and sensors
2. **LCD displays**:
   ```
   EcoBin System
   Initializing...
   ```
3. **After 2 seconds** → Transitions to normal monitoring mode

### Normal Operation

**LCD Display Format:**
```
Fill: XX%  D:XXXcm
Status: NORMAL
```

- **Fill**: Current fill percentage (0-100%)
- **D**: Raw distance reading in centimeters
- **Status**: Current system state

### Alert States

#### Primary Alert (70-89% Full)
```
LCD: Fill: 75%  D:25cm
     PRIMARY ALERT!

LED: Blinks (500ms on/off)
Buzzer: Single beep every 2 seconds
Action: Check waste level soon
```

#### Critical Alert (90-94% Full)
```
LCD: Fill: 92%  D:10cm
     CRITICAL!EMPTY!

LED: Steady ON
Buzzer: Continuous beeping
Action: Empty bin immediately
```

#### Full State (≥95%)
```
LCD: Fill: 96%  D:5cm
     BIN FULL!RESET

LED: Steady ON
Buzzer: Continuous beeping
Action: Empty bin and press # to reset
```

#### Fire Alert
```
LCD: Fill: XX%  D:XXXcm
     FIRE DETECTED!

LED: Rapid blinking (200ms)
Buzzer: Rapid beeping (200ms)
Action: Emergency - investigate immediately
```

### Maintenance Mode Access

1. **Press `*` key** → LCD shows "Enter PIN:"
2. **Enter 4-digit PIN** (default: `1234`)
3. **Press `#` to confirm**

**Maintenance Menu:**
```
Maintenance Mode
1:Calib 2:Reset
```

**Options:**
- **Press 1**: Sensor calibration
  - Follow LCD instructions to set empty/full distances
- **Press 2**: Manual system reset
  - Clears critical state and resets counters
- **Press `*`**: Exit maintenance mode

### Changing Default PIN

Edit `ecobin_firmware.c`:
```c
// Around line 95
static const char default_pin[] = "1234";  // Change to your PIN
```

---

## 🔄 System States

### State Transition Diagram

```
         ┌─────────────┐
         │    INIT     │
         └──────┬──────┘
                │ (auto after 2s)
                ▼
         ┌─────────────┐
    ┌───│   NORMAL    │◄──┐
    │   └──────┬──────┘   │
    │          │           │
    │ (fill ≥70%)         │
    │          ▼           │
    │   ┌─────────────┐   │
    ├──►│PRIMARY_ALERT│───┤
    │   └──────┬──────┘   │
    │          │           │
    │ (fill ≥90%)         │
    │          ▼           │
    │   ┌─────────────┐   │
    └──►│CRITICAL_ALERT◄──┘
        └──────┬──────┘
               │
        (fill ≥95% - locked)
               │
               ▼
        ┌─────────────┐
        │    FULL     │
        └─────────────┘
               │
        (manual reset via #)
               │
               └──────► NORMAL

FIRE_ALERT: Can trigger from any state (highest priority)
MAINTENANCE: Accessible via * + PIN from any non-fire state
```

### State Descriptions

| State | Entry Condition | Exit Condition | LED | Buzzer |
|-------|----------------|----------------|-----|--------|
| INIT | Power on | 2s timeout | OFF | OFF |
| NORMAL | Fill < 70% | Fill ≥ 70% | OFF | OFF |
| PRIMARY_ALERT | 70% ≤ Fill < 90% | Fill < 70% or ≥ 90% | Blink (500ms) | Beep (2s interval) |
| CRITICAL_ALERT | 90% ≤ Fill < 95% | Fill < 90% or ≥ 95% | Steady ON | Continuous |
| FULL | Fill ≥ 95% | Manual reset (#) | Steady ON | Continuous |
| FIRE_ALERT | Flame detected | Flame cleared | Rapid blink (200ms) | Rapid beep (200ms) |
| MAINTENANCE | * + correct PIN | * (exit) | Status display | OFF |

---

## 🐛 Troubleshooting

### Compilation Errors

#### Error: RAM Overflow
```
Error: address 0x801355 of section `.bss' is not within region `data'
```

**Solution:**
```c
// In ecobin_firmware.c, reduce log buffer size
#define MAX_LOG_ENTRIES     20  // Was 500, reduce to 20
```

#### Error: F_CPU Not Defined
```
Warning: #warning "F_CPU not defined for <util/delay.h>"
```

**Solution:**
```c
// Add at TOP of ecobin_firmware.c (before includes)
#define F_CPU 16000000UL
```

### Hardware Issues

#### LCD Shows Nothing
- Check contrast pot adjustment (turn fully)
- Verify 5V and GND connections
- Check EN pin connection (must pulse)
- Verify 4-bit data pins (D4-D7)

#### LCD Shows Blocks
- Incorrect initialization timing
- Check RW pin is grounded
- Verify RS pin connection

#### Ultrasonic Sensor Not Reading
- Check TRIG and ECHO connections
- Verify sensor has clear line of sight
- Ensure 5V power supply is stable
- Test with voltmeter: ECHO should pulse 5V

#### Keypad Not Responding
- Verify all 7 pins are connected
- Check column pullup resistors (10kΩ)
- Test with multimeter: rows should toggle LOW
- Ensure no short circuits between pins

#### Buzzer Silent
- Check polarity (+ to D12, - to GND)
- Verify it's an ACTIVE buzzer (has internal oscillator)
- Test directly: connect + to 5V, should sound

#### LED Not Working
- Check polarity (longer leg = anode = +)
- Verify 220Ω resistor in series
- Test directly: connect to 5V via resistor

### Proteus Simulation Issues

#### "No Hex File Loaded"
```
Solution: Right-click ATmega328P → Edit Properties → Program File
          Browse to: ecobin_firmware.hex
```

#### Simulation Runs Too Fast
```
Solution: Click Debug → Set Animation Options → Slow down
```

#### Components Not Found
```
Solution: Install Arduino library for Proteus
          Or use alternative component names from table
```

---

## 💾 Memory Optimization

### Current Memory Usage

```
AVR Memory Usage (Optimized)
----------------------------
Program Memory: 8542 bytes (26.1% of 32KB Flash)
Data Memory:     987 bytes (48.2% of 2KB SRAM)
```

### Optimization Strategies

#### 1. Reduce Log Buffer Size
```c
// Adjust based on requirements
#define MAX_LOG_ENTRIES     20   // 200 bytes
#define MAX_LOG_ENTRIES     10   // 100 bytes (minimal)
```

#### 2. Use PROGMEM for Strings
```c
// Store constant strings in Flash instead of SRAM
const char msg[] PROGMEM = "System Ready";
lcd_print_progmem(msg);  // Custom print function needed
```

#### 3. Minimize Global Variables
- Use local variables in functions
- Reuse buffers where possible
- Avoid large arrays

#### 4. Compiler Optimization
```c
// In Atmel Studio: Project Properties → Toolchain
// AVR/GNU C Compiler → Optimization
// Set to: -Os (Optimize for size)
```

### EEPROM Storage (Advanced)

For persistent logging beyond 20 entries:

```c
#include <avr/eeprom.h>

// Store logs in EEPROM (1024 bytes available)
void save_log_to_eeprom(LogEntry_t* entry, uint8_t index) {
    uint16_t addr = index * sizeof(LogEntry_t);
    eeprom_write_block(entry, (void*)addr, sizeof(LogEntry_t));
}

void load_log_from_eeprom(LogEntry_t* entry, uint8_t index) {
    uint16_t addr = index * sizeof(LogEntry_t);
    eeprom_read_block(entry, (void*)addr, sizeof(LogEntry_t));
}
```


## 🤝 Contributing

Contributions are welcome! Please follow these guidelines:

### Reporting Issues

1. Check existing issues first
2. Provide detailed description
3. Include error messages and screenshots
4. Specify hardware/software versions

### Submitting Pull Requests

1. Fork the repository
2. Create feature branch: `git checkout -b feature/amazing-feature`
3. Commit changes: `git commit -m 'Add amazing feature'`
4. Push to branch: `git push origin feature/amazing-feature`
5. Open Pull Request with detailed description

### Code Style

- Follow existing code formatting
- Add comments for complex logic
- Update documentation for new features
- Test thoroughly before submitting

---

## 📄 License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.

```
MIT License

Copyright (c) 2024 [Your Name]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

[Full MIT License text...]
```

---

## 👤 Authors

GROUP-30 Team Members.
- University: Makerere University
- Course: Embedded Systems Coursework 2025

### Supervisor

**Dr.Nsabagwa Mary**
- Department of Networks
- School of Computing and Informatics Technology.
- College of Computing and Information Sciences.



## 📖 References

1. ATmega328P Datasheet - Microchip Technology Inc.
2. HD44780 LCD Controller - Hitachi Semiconductor
3. HC-SR04 Ultrasonic Sensor - Datasheet and Application Notes
4. AVR Libc Reference Manual - nongnu.org/avr-libc
5. Embedded C Coding Standard - Barr Group

---

## 📊 Project Status

![Progress](https://img.shields.io/badge/progress-100%25-brightgreen.svg)

- [x] Firmware implementation (Complete)
- [x] Proteus simulation design (Complete)
- [x] Hardware testing (Complete)
- [x] Documentation (Complete)
- [x] Academic report (Complete)
- [ ] EEPROM logging (Optional enhancement)
- [ ] Web dashboard (Future work)
- [ ] IoT integration (Future work)

---

## 🎓 Academic Information

**Course**: Embedded Systems Design  
**Semester**: Fall 2024  
**Grade Target**: 100%  
**Submission Date**: [Your Deadline]

### Assessment Criteria Met

- ✅ Complete firmware with all required features
- ✅ Professional code quality and documentation
- ✅ Proteus simulation capability
- ✅ Hardware implementation and testing
- ✅ Comprehensive academic report
- ✅ Pin configuration documentation
- ✅ Troubleshooting guides

---

## 🔗 Useful Links

- [ATmega328P Datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf)
- [Atmel Studio Download](https://www.microchip.com/mplab/avr-support/atmel-studio-7)
- [Proteus Design Suite](https://www.labcenter.com/)
- [AVR Libc Documentation](https://www.nongnu.org/avr-libc/)
- [Arduino Uno Pinout Reference](https://www.arduino.cc/en/Reference/Board)

---

## 📞 Support

For questions or issues:

1. **Check documentation** in `/docs` folder
2. **Review troubleshooting** section above
3. **Search existing issues** on GitHub
4. **Open new issue** with detailed information
5. **Contact author** via email

---

<div align="center">

**⭐ If this project helped you, please give it a star! ⭐**

Made with ❤️ for embedded systems enthusiasts

</div>

---

## Version History

### v1.0.0 (Current)
- Initial release with full feature set
- Proteus simulation support
- Complete documentation
- 4×3 keypad integration
- Memory optimized for ATmega328P

### Future Versions
- v1.1.0: EEPROM logging support
- v1.2.0: Serial communication interface
- v2.0.0: IoT connectivity (ESP8266 integration)

---

*Last Updated: 2025-27-10*
