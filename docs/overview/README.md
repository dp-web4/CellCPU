# CellCPU System Overview

## Table of Contents

1. [Introduction](#introduction)
2. [System Architecture](#system-architecture)
3. [Key Features](#key-features)
4. [Hardware Platform](#hardware-platform)
5. [Communication Protocol](#communication-protocol)
6. [Integration Context](#integration-context)
7. [Performance Specifications](#performance-specifications)

## Introduction

The CellCPU firmware represents a **production-ready** implementation of ModBatt's hierarchical battery management system, running on ATtiny45 microcontrollers that directly interface with individual lithium-ion cells. Each CellCPU manages a single cell with sophisticated interrupt-driven architecture, providing real-time voltage monitoring, temperature sensing, and active balancing control.

**Implementation Status**: All core modules are fully implemented with professional-grade code including interrupt-driven Virtual UART, bit-banged I2C with timing quantization, and comprehensive error handling.

### System Role

CellCPU serves as the foundation layer in the ModBatt architecture:

```
ModBatt Hierarchical Architecture:
┌─────────────────────────────────────────────────────────────────┐
│                Vehicle Control Unit (VCU)                       │
└─────────────────────────┬───────────────────────────────────────┘
                          │ CAN Bus
┌─────────────────────────┼───────────────────────────────────────┐
│                  Pack Controller                                │
│             (STM32WB55-based firmware)                         │
└─────────────────────────┬───────────────────────────────────────┘
                          │ CAN Bus
        ┌─────────────────┼─────────────────┐
        │                 │                 │
┌───────▼───────┐ ┌───────▼───────┐ ┌───────▼───────┐
│ Module CPU #1 │ │ Module CPU #2 │ │ Module CPU #N │
│ (ATmega64M1)  │ │               │ │  (up to 32)   │
└───────┬───────┘ └───────┬───────┘ └───────┬───────┘
        │                 │                 │
   ┌────▼────┐       ┌────▼────┐       ┌────▼────┐
   │CellCPUs │       │CellCPUs │       │CellCPUs │
   │ 1-94    │       │ 1-94    │       │ 1-94    │ ◄── This Firmware
   │(ATtiny45│       │(ATtiny45│       │(ATtiny45│    (CellCPU)
   │per cell)│       │per cell)│       │per cell)│
   └─────────┘       └─────────┘       └─────────┘
        │                 │                 │
   ┌────▼────┐       ┌────▼────┐       ┌────▼────┐
   │Li-ion   │       │Li-ion   │       │Li-ion   │
   │Cell 1-94│       │Cell 1-94│       │Cell 1-94│
   └─────────┘       └─────────┘       └─────────┘
```

### Core Responsibilities

- **Cell Voltage Monitoring**: High-precision ADC measurement of individual cell voltage
- **Temperature Sensing**: I2C-based temperature monitoring using MCP9843 sensor
- **Active Balancing**: FET-based discharge control for cell balancing
- **Communication Relay**: Serial communication chain between ModuleCPU and all cells
- **Safety Protection**: Local cell-level protection and fault detection

## System Architecture

### Chip-Level Design

```
┌─────────────────────────────────────────────────────────────────┐
│                      CellCPU Architecture                       │
│                      (ATtiny45 Based)                          │
├─────────────────────────────────────────────────────────────────┤
│ Core Microcontroller                                            │
│ ├── ATtiny45: 8-bit AVR @ 8MHz (internal oscillator)          │
│ ├── Memory: 4KB Flash, 256B SRAM, 256B EEPROM                 │
│ ├── ADC: 10-bit, 1.1V internal reference                      │
│ └── Timers: Timer0 for UART bit timing                        │
├─────────────────────────────────────────────────────────────────┤
│ External Components                                             │
│ ├── MCP9843 Temperature Sensor (I2C interface)                │
│ ├── Discharge FET Control (via MCP9843 EVENT pin)             │
│ ├── Voltage Divider (for cell voltage measurement)            │
│ └── Protection Components (TVS, filtering)                     │
├─────────────────────────────────────────────────────────────────┤
│ Communication Interfaces                                        │
│ ├── Virtual UART: Bit-banged serial @ 20kbps                  │
│ ├── I2C: Hardware I2C for temperature sensor                  │
│ └── Daisy Chain: Cell-to-cell serial communication            │
└─────────────────────────────────────────────────────────────────┘
```

### Pin Configuration

```
ATtiny45 Pin Assignment:
├── PB0: cell_up_tx     (Output to upstream cell)
├── PB1: cell_dn_rx     (Input from downstream cell/ModuleCPU)
├── PB2: I2C SDA        (Temperature sensor communication)
├── PB3: I2C SCL/ADC3   (Temperature sensor + cell voltage ADC)
├── PB4: cell_up_rx     (Input from upstream cell)
└── PB5: cell_dn_tx     (Output to downstream cell)

Signal Flow:
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│ Module CPU  │───▶│  CellCPU    │───▶│  CellCPU    │
│             │    │   (Last)    │    │  (Middle)   │
│             │◄───│             │◄───│             │
└─────────────┘    └─────────────┘    └─────────────┘
 cell_dn_rx          cell_up_tx          cell_up_tx
 cell_dn_tx          cell_dn_rx          cell_dn_rx
```

## Key Features

### Voltage Monitoring
- **Resolution**: 10-bit ADC with 1.1V internal reference
- **Range**: 0V to ~5.0V (cell range: 2.5V to 4.2V typical)
- **Accuracy**: ±1% after calibration
- **Update Rate**: On-demand measurement during communication cycles

### Temperature Sensing
- **Sensor**: MCP9843 precision temperature sensor
- **Interface**: I2C communication at 100kHz
- **Range**: -40°C to +125°C
- **Accuracy**: ±0.5°C typical
- **Resolution**: 0.0625°C (12-bit)

### Active Balancing
- **Method**: FET-based discharge balancing
- **Control**: MCP9843 EVENT pin controls discharge FET
- **Trigger**: Configurable voltage threshold from ModuleCPU
- **Current**: Depends on external FET and load resistor design

### Communication Protocol
- **Physical Layer**: Virtual UART at 20kbps (50μs bit time)
- **Topology**: Daisy-chained serial communication
- **Data Format**: 4-byte message per cell (voltage + temperature + status)
- **Bi-directional**: Commands downstream, data upstream

### Power Management
- **Operating Current**: ~1mA active, <100μA idle
- **Supply Voltage**: 3.0V to 5.5V
- **Clock Source**: Internal 8MHz RC oscillator
- **Sleep Modes**: Idle mode during communication gaps

## Hardware Platform

### ATtiny45 Specifications

```
Microcontroller Specifications:
├── Architecture: 8-bit AVR RISC
├── Operating Frequency: 8MHz (internal RC oscillator)
├── Flash Memory: 4KB (program storage)
├── SRAM: 256 bytes (data memory)
├── EEPROM: 256 bytes (non-volatile storage)
├── ADC: 10-bit, 4 channels, 1.1V internal reference
├── Timers: 8-bit Timer/Counter0 with PWM
├── I/O Pins: 6 programmable I/O pins
├── Communication: USI (Universal Serial Interface)
└── Operating Voltage: 2.7V to 5.5V
```

### External Component Integration

#### MCP9843 Temperature Sensor
```
MCP9843 Features:
├── Interface: I2C (address 0x30)
├── Temperature Range: -40°C to +125°C
├── Accuracy: ±0.5°C (typical)
├── Resolution: 0.0625°C (12-bit)
├── Features: Programmable alert pin (used for FET control)
└── Power: 200μA typical operating current
```

#### Voltage Measurement Circuit
```
Voltage Divider Network:
├── Purpose: Scale cell voltage for ADC input
├── Configuration: Resistor divider with filtering
├── Input Range: 0V to 5.0V (covers cell voltage range)
├── ADC Reference: Internal 1.1V reference
└── Protection: TVS diode and RC filtering
```

## Communication Protocol

### Message Structure

The CellCPU implements a simple but robust 4-byte message format:

```c
Cell Data Message (4 bytes):
├── Byte 0: Voltage LSB
├── Byte 1: Voltage MSB + Discharge Status (bit 7)
├── Byte 2: Temperature LSB  
└── Byte 3: Temperature MSB + I2C Status (bit 7)

Command Message (2 bytes):
├── Byte 0: Command Type + Voltage Target MSB
└── Byte 1: Voltage Target LSB
```

### Communication Flow

```
Downstream Command Flow (ModuleCPU → CellCPUs):
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│ Module CPU  │───▶│  CellCPU    │───▶│  CellCPU    │
│             │    │   (Last)    │    │  (First)    │
└─────────────┘    └─────────────┘    └─────────────┘
   Commands           Relay              Relay
   
Upstream Data Flow (CellCPUs → ModuleCPU):
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│ Module CPU  │◄───│  CellCPU    │◄───│  CellCPU    │
│             │    │   (Last)    │    │  (First)    │
└─────────────┘    └─────────────┘    └─────────────┘
  Collect Data     Append Data       Initiate TX
```

### Protocol Features
- **Auto-detection**: Last cell in chain initiates upstream transmission
- **Data Aggregation**: Each cell appends its data to the upstream message
- **Command Relay**: Commands propagate through entire cell chain
- **Error Handling**: I2C and communication error flags in status bits

## Integration Context

### Module-Level Integration

Each CellCPU integrates into a larger module controlled by ModuleCPU:

```
Module Architecture:
┌─────────────────────────────────────────────────────────────────┐
│                    Battery Module                               │
│  ┌─────────────┐                                               │
│  │ ModuleCPU   │ ←→ CAN Bus (to Pack Controller)               │
│  │(ATmega64M1) │                                               │
│  └──────┬──────┘                                               │
│         │ Virtual UART @ 20kbps                                │
│    ┌────▼────┬────────┬────────┬────────┬─────────┐           │
│    │CellCPU  │CellCPU │CellCPU │CellCPU │  ...    │           │
│    │   #1    │   #2   │   #3   │   #4   │  #94    │           │
│    │(ATtiny45│        │        │        │         │           │
│    └────┬────┴────────┴────────┴────────┴─────────┘           │
│         │                                                      │
│    ┌────▼────┬────────┬────────┬────────┬─────────┐           │
│    │Li-ion   │Li-ion  │Li-ion  │Li-ion  │  ...    │           │
│    │Cell #1  │Cell #2 │Cell #3 │Cell #4 │ Cell#94 │           │
│    └─────────┴────────┴────────┴────────┴─────────┘           │
└─────────────────────────────────────────────────────────────────┘
```

### System-Level Context

The CellCPU provides the foundational data that flows up through the entire system:

```
Data Flow Hierarchy:
Cell Data (CellCPU) 
    ↓
Module Aggregation (ModuleCPU)
    ↓  
Pack Management (Pack Controller)
    ↓
Vehicle Interface (VCU)
    ↓
Fleet Management (Cloud/Telematics)
```

## Performance Specifications

### Timing Characteristics

```
Real-Time Performance:
├── ADC Conversion Time: ~100μs (13 cycles @ 125kHz ADC clock)
├── I2C Transaction Time: ~1ms (temperature reading)
├── Communication Bit Time: 50μs (20kbps virtual UART)
├── Message Transmission: ~1.6ms (4 bytes × 10 bits × 50μs)
├── Total Cell Response: <5ms typical
└── Power-on Initialization: ~10ms
```

### Resource Utilization

```
Memory Usage:
├── Flash Program Memory: ~2.5KB / 4KB available (62%)
├── SRAM Data Memory: ~150 bytes / 256 bytes (58%)
├── EEPROM: Not used in current implementation
└── I/O Pins: 6/6 pins used (100% utilization)

Power Consumption:
├── Active Operation: ~1mA @ 5V
├── I2C Communication: +200μA during transactions
├── Idle/Sleep Current: <100μA
└── Total Module Impact: <100mA for 94 cells
```

### Environmental Specifications

```
Operating Conditions:
├── Supply Voltage: 3.0V to 5.5V
├── Operating Temperature: -40°C to +85°C
├── Humidity: 0% to 95% RH (non-condensing)
├── Vibration: Automotive grade (varies by implementation)
└── EMI/EMC: Automotive compliance requirements
```

The CellCPU represents a highly optimized, minimal-resource solution for individual cell monitoring and control within ModBatt's scalable battery management architecture. Despite its small footprint, it provides comprehensive functionality essential for safe and efficient battery operation.