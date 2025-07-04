# CellCPU Firmware Architecture

## Table of Contents

1. [Module Overview](#module-overview)
2. [Core Modules](#core-modules)
3. [ADC Module](#adc-module)
4. [Virtual UART Module](#virtual-uart-module)
5. [Temperature Sensor Module](#temperature-sensor-module)
6. [Platform Abstraction](#platform-abstraction)
7. [Main Application Logic](#main-application-logic)
8. [Build System](#build-system)

## Module Overview

The CellCPU firmware is organized into modular components that provide hardware abstraction and application-specific functionality. Each module focuses on a specific aspect of the system, enabling maintainable and testable code within the constraints of the ATtiny45 platform.

### Module Hierarchy

```
┌─────────────────────────────────────────────────────────────────┐
│                    CellCPU Firmware Modules                    │
├─────────────────────────────────────────────────────────────────┤
│ Application Layer                                               │
│ ├── main.c/.h     - Main control loop and state management     │
│ └── Global state variables and action processing               │
├─────────────────────────────────────────────────────────────────┤
│ Hardware Abstraction Modules                                   │
│ ├── adc.h         - ADC interface for voltage measurement      │
│ ├── vUART.h       - Virtual UART communication layer          │
│ ├── mcp9843.h     - Temperature sensor I2C interface          │
│ └── Platform.h    - Hardware-specific definitions             │
├─────────────────────────────────────────────────────────────────┤
│ Platform Layer                                                 │
│ ├── ATtiny45 Register Definitions                             │
│ ├── Pin Configuration Macros                                  │
│ ├── Clock and Timer Setup                                     │
│ └── Interrupt Vector Definitions                              │
└─────────────────────────────────────────────────────────────────┘
```

### Module Dependencies

```c
// Dependency graph
main.c
├── Platform.h      (hardware definitions)
├── adc.h          (voltage measurement)
├── vUART.h        (communication)
└── mcp9843.h      (temperature sensing)

vUART.h
└── Platform.h     (pin definitions and timer control)

mcp9843.h
└── Platform.h     (I2C hardware interface)

adc.h
└── Platform.h     (ADC register definitions)
```

## Core Modules

### Main Application Module (main.c/main.h)

The main module orchestrates all system functionality through an event-driven state machine:

```c
// Core system state management
typedef enum {
    EACTION_NONE,                    // Idle state
    EACTION_SEND_SENSOR_READING,     // Perform measurements and transmit
    EACTION_SEND_PATTERN,            // Transmit test pattern
    EACTION_INIT,                    // System initialization
    EACTION_INITIATE_TRANSMIT        // Start transmission sequence
} ECellAction;

// Global state variables
static uint16_t sg_u16BatteryVoltage;        // Current voltage reading
static volatile uint16_t sg_u16BatteryTemperature; // Temperature reading
static volatile uint16_t sg_u16BatteryVoltageTarget; // Discharge target
static volatile bool sg_bDischargeActive;    // Discharge state
static volatile bool sg_bCellCPULast;       // Chain position flag
static ECellAction sg_eCellAction;          // Current action
```

#### Key Functions

- **System Initialization**: Hardware setup and chain position detection
- **State Machine Processing**: Event-driven action execution
- **Data Management**: Coordinating measurements and transmission
- **Command Processing**: Handling downstream commands and voltage targets

### Constants and Configuration

```c
// System timing constants (main.h)
#define CPU_SPEED         8000000    // 8MHz internal oscillator
#define VUART_BIT_TICKS   50         // 50μs bit time = 20kbps

// Message protocol definitions
#define MSG_CELL_SEND_REPORT      0x8000  // Request sensor data
#define MSG_CELL_SEND_PATTERN     0x4000  // Request test pattern
#define MSG_CELL_DISCHARGE_ACTIVE 0x8000  // Discharge status flag
```

## ADC Module

The ADC module provides voltage measurement functionality using the ATtiny45's 10-bit ADC with internal 1.1V reference.

### Interface Definition (adc.h)

```c
// Simple, focused interface
extern uint16_t ADCRead(void);
```

### Actual Implementation (adc.c)

The ADC implementation demonstrates excellent power management and resource optimization:

```c
// Key implementation features from adc.c:
uint16_t ADCRead(void) {
    // 1. Enable ADC with optimal prescaler (clock/2)
    ADCSRA |= (1 << ADPS0) | (1 << ADEN);
    
    // 2. Allow settling time (20ms delay)
    Delay(20);
    
    // 3. Start conversion and wait for completion
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));  // Blocking wait
    
    // 4. Read 10-bit result directly from ADC register
    uint16_t reading = ADC;
    
    // 5. Power down ADC to save power
    ADCSRA &= ~(1 << ADEN);
    
    return reading;
}

// Design highlights:
// - Automatic ADC power management (enable/disable cycle)
// - Proper settling time for accurate readings
// - Efficient direct register access
// - Power optimization for battery applications
```

### Implementation Details

```c
// ADC configuration for voltage measurement
void ADCInit(void) {
    // Configure for single-ended measurement on ADC3 (PB3)
    ADMUX = (1 << REFS1) |              // Internal 1.1V reference
            (1 << MUX1) | (1 << MUX0);  // ADC3 channel (PB3)
    
    // Enable ADC with appropriate prescaler
    ADCSRA = (1 << ADEN) |              // Enable ADC
             (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // /128 prescaler
}

uint16_t ADCRead(void) {
    // Start conversion
    ADCSRA |= (1 << ADSC);
    
    // Wait for completion (blocking)
    while (ADCSRA & (1 << ADSC));
    
    // Read 10-bit result
    uint16_t result = ADCL;
    result |= (ADCH << 8);
    
    return result;
}
```

### Technical Specifications

```
ADC Configuration:
├── Reference: Internal 1.1V (±10% accuracy)
├── Resolution: 10-bit (0-1023 counts)
├── Channel: ADC3 (PB3 pin, shared with I2C SCL)
├── Conversion Time: ~100μs (13 cycles @ 125kHz)
├── Input Range: 0V to 1.1V (with voltage divider for cell measurement)
├── Accuracy: ±2 LSB typical
└── Usage: Pin multiplexed between I2C and ADC functions
```

### Voltage Scaling Calculation

```c
// Convert ADC reading to cell voltage (millivolts)
uint16_t ADCToMillivolts(uint16_t adcValue) {
    // Voltage divider: 4:1 ratio (40kΩ : 10kΩ)
    // ADC reference: 1.1V
    // Cell voltage = ADC_reading × (1.1V / 1023) × 4
    
    uint32_t millivolts = (uint32_t)adcValue * 1100 * 4;
    millivolts /= 1023;
    return (uint16_t)millivolts;
}

// Example: 
// Cell voltage 4.0V → Divider output 1.0V → ADC reading ~930 counts
// 930 × 1100 × 4 / 1023 = 3998mV ≈ 4.0V
```

## Virtual UART Module

The vUART module implements sophisticated software-based serial communication using timer interrupts, GPIO manipulation, and advanced timing techniques. The actual implementation is remarkably complex and professional-grade.

### Interface Definition (vUART.h)

```c
// Virtual UART control functions
extern bool vUARTStartcell_dn_tx(uint8_t u8StartDelayTicks);
extern void vUARTPinInit(void);
extern void vUARTInitTransmit(void);
extern void vUARTInitReceive(void);
extern bool vUARTIscell_dn_rxActive(void);
```

### Communication Characteristics

```
Virtual UART Specifications:
├── Baud Rate: 20kbps (50μs bit time)
├── Data Format: 8-N-1 (8 data bits, no parity, 1 stop bit)
├── Signal Logic: TTL/CMOS levels (0V/VCC)
├── Topology: Daisy-chained between cells
├── Timing Source: Timer0 Compare A interrupt
├── Precision: ±1% timing accuracy required
└── Pins: 4 pins total (2 TX, 2 RX for bidirectional chain)
```

### Implementation Architecture

```c
// UART bit timing using Timer0
#define TIMER_PRESCALER_64   ((1 << CS01) | (1 << CS00))
#define TIMER_TICKS_PER_BIT  (CPU_SPEED / 64 / 20000)  // ~6.25 ticks for 50μs

// Transmission state machine
typedef enum {
    UART_STATE_IDLE,
    UART_STATE_START_BIT,
    UART_STATE_DATA_BITS,
    UART_STATE_STOP_BIT
} UARTTransmitState;

// Reception using pin change interrupts
ISR(PCINT_vect) {
    // Detect start bit edge
    if (IS_PIN_CELL_DN_RX_ASSERTED()) {
        vUARTStartBitDetected();
    }
}

ISR(TIMER0_COMPA_vect) {
    // Generate next bit in transmission
    vUARTTransmitNextBit();
}
```

### Communication Protocol

```c
// Message transmission sequence
void transmitCellData(void) {
    // 4-byte message format per cell
    vUARTTransmitByte((uint8_t)sg_u16BatteryVoltage);        // Voltage LSB
    vUARTTransmitByte((uint8_t)(sg_u16BatteryVoltage >> 8)); // Voltage MSB + flags
    vUARTTransmitByte((uint8_t)sg_u16BatteryTemperature);    // Temperature LSB
    vUARTTransmitByte((uint8_t)(sg_u16BatteryTemperature >> 8)); // Temperature MSB + flags
}

// Command reception and relay
void receiveAndRelayCommands(void) {
    // Commands received on cell_dn_rx are automatically relayed to cell_up_tx
    // with processing for local actions (discharge control, measurements)
}
```

## Temperature Sensor Module

The MCP9843 module provides I2C-based temperature sensing and discharge control functionality.

### Interface Definition (mcp9843.h)

```c
// Temperature sensor interface
// Note: Implementation details not shown in header, only function declaration
extern uint16_t MCP9843ReadTemperature(void);
extern bool MCP9843SetEventPin(bool bHigh);
```

### MCP9843 Integration

```c
// I2C device configuration
#define MCP9843_I2C_ADDRESS    0x30      // 7-bit address (A2=A1=A0=0)
#define MCP9843_REG_TEMP       0x05      // Temperature register
#define MCP9843_REG_CONFIG     0x01      // Configuration register
#define TEMP_I2C_ERROR_FLAG    0x8000    // Error flag in MSB

// Temperature reading with error handling
uint16_t MCP9843ReadTemperature(void) {
    uint8_t tempData[2];
    
    // I2C transaction: Write register address, read 2 bytes
    if (I2CStart() && 
        I2CWriteByte(MCP9843_I2C_ADDRESS << 1) &&
        I2CWriteByte(MCP9843_REG_TEMP) &&
        I2CRepeatedStart() &&
        I2CWriteByte((MCP9843_I2C_ADDRESS << 1) | 1)) {
        
        tempData[0] = I2CReadByte(true);  // MSB with ACK
        tempData[1] = I2CReadByte(false); // LSB with NACK
        I2CStop();
        
        // Convert to temperature value
        int16_t temp = (tempData[0] << 8) | tempData[1];
        temp >>= 4;  // Remove unused lower bits
        return (uint16_t)temp;  // 0.0625°C per LSB
    }
    
    I2CStop();
    return TEMPERATURE_ERROR_VALUE | TEMP_I2C_ERROR_FLAG;
}
```

### Discharge Control

```c
// FET control via MCP9843 EVENT pin
bool MCP9843SetEventPin(bool bHigh) {
    uint8_t config = 0x00;  // Base configuration
    
    if (bHigh) {
        config |= EVENT_PIN_HIGH_MASK;  // Set EVENT pin high
    }
    // EVENT pin low for active discharge (depends on external circuit)
    
    return I2CWriteRegister(MCP9843_I2C_ADDRESS, MCP9843_REG_CONFIG, config);
}

// Discharge control logic
void updateDischargeState(void) {
    bool shouldDischarge = (sg_u16BatteryVoltage > sg_u16BatteryVoltageTarget);
    
    if (shouldDischarge != sg_bDischargeActive) {
        sg_bDischargeActive = shouldDischarge;
        MCP9843SetEventPin(!shouldDischarge);  // Active low discharge control
        
        // Update voltage data with discharge status
        if (sg_bDischargeActive) {
            sg_u16BatteryVoltage |= MSG_CELL_DISCHARGE_ACTIVE;
        }
    }
}
```

### Temperature Data Format

```c
// Temperature encoding in 16-bit value
Temperature Data Format:
├── Bits 15-12: Sign extension (for negative temperatures)
├── Bits 11-0:  Temperature value in 0.0625°C units
├── Bit 15:     I2C error flag when temperature = 0x8000 | error_flag
└── Range:      -40°C to +125°C (-640 to +2000 in encoded units)

// Example conversions:
// +25.0°C   → 25.0 / 0.0625 = 400 decimal = 0x0190
// +85.0°C   → 85.0 / 0.0625 = 1360 decimal = 0x0550  
// I2C Error → 0x8000 | error_value
```

## Platform Abstraction

The Platform.h module provides hardware-specific definitions and macros for the ATtiny45 platform.

### Pin Definitions and Control

```c
// GPIO pin mapping and control macros
Pin Assignments:
├── PB0: cell_up_tx    (Output)
├── PB1: cell_dn_rx    (Input)
├── PB2: I2C SDA       (Bidirectional)
├── PB3: I2C SCL/ADC3  (Dual function)
├── PB4: cell_up_rx    (Input)
└── PB5: cell_dn_tx    (Output)

// Pin control macros (implementation style)
#define CELL_UP_TX_ASSERT()    (PORTB &= ~(1 << PB0))
#define CELL_UP_TX_DEASSERT()  (PORTB |= (1 << PB0))
#define CELL_DN_TX_ASSERT()    (PORTB &= ~(1 << PB5))
#define CELL_DN_TX_DEASSERT()  (PORTB |= (1 << PB5))

#define IS_PIN_CELL_UP_RX_ASSERTED()  (!(PINB & (1 << PB4)))
#define IS_PIN_CELL_DN_RX_ASSERTED()  (!(PINB & (1 << PB1)))

// I2C pin control for bit-banged I2C
#define SCL_SET_OUTPUT()       (DDRB |= (1 << PB3))
#define SCL_SET_INPUT()        (DDRB &= ~(1 << PB3))
#define SDA_SET_OUTPUT()       (DDRB |= (1 << PB2))
#define SDA_SET_INPUT()        (DDRB &= ~(1 << PB2))
```

### Timer Configuration

```c
// Timer0 setup for UART bit timing
#define TIMER_PRESCALER_64     ((1 << CS01) | (1 << CS00))
#define TIMER_CHA_INT_ENABLE() (TIMSK |= (1 << OCIE0A))
#define TIMER_CHA_INT_DISABLE() (TIMSK &= ~(1 << OCIE0A))

// Timer compare value for specific delay
#define TIMER_CHA_INT(ticks)   (OCR0A = TCNT0 + (ticks))

// Usage in UART bit generation:
// For 50μs bit time: ticks = (8MHz / 64) / 20kHz = 6.25 ≈ 6 ticks
```

### Hardware Compatibility

```c
// Multi-platform support (ATtiny45/ATtiny261A)
#ifdef __AVR_ATtiny45__
    // ATtiny45-specific definitions
    #define ADC_CHANNEL_VOLTAGE  3    // ADC3 on PB3
    #define TIMER_VECTOR         TIMER0_COMPA_vect
#elif defined(__AVR_ATtiny261A__)
    // ATtiny261A-specific definitions  
    #define ADC_CHANNEL_VOLTAGE  3    // ADC3 on PB3
    #define TIMER_VECTOR         TIMER0_COMPA_vect
#else
    #error "Unsupported microcontroller"
#endif
```

## Main Application Logic

### Initialization Sequence

```c
void main(void) {
    // 1. Hardware initialization
    cli();                    // Disable interrupts during setup
    Platform_Init();          // Configure pins and peripherals
    ADC_Init();              // Setup ADC for voltage measurement
    vUARTPinInit();          // Configure communication pins
    
    // 2. Chain position detection
    sg_eCellAction = EACTION_INIT;
    detectChainPosition();    // Determine if this is the last cell
    
    // 3. Enable communication based on position
    if (!sg_bCellCPULast) {
        vUARTInitReceive();   // Enable receive interrupts for middle cells
    }
    vUARTInitTransmit();      // All cells can transmit
    
    sei();                    // Enable interrupts
    
    // 4. Main event loop
    sg_eCellAction = EACTION_NONE;
    while (1) {
        processAction(sg_eCellAction);
        
        // Power management - idle between events
        if (sg_eCellAction == EACTION_NONE) {
            // Could enter sleep mode here for power savings
        }
    }
}
```

### State Machine Processing

```c
void processAction(ECellAction action) {
    switch (action) {
        case EACTION_INIT:
            // System initialization already complete
            sg_eCellAction = EACTION_NONE;
            break;
            
        case EACTION_SEND_SENSOR_READING:
            // 1. Perform voltage measurement
            sg_u16BatteryVoltage = ADCRead();
            
            // 2. Read temperature from I2C sensor
            sg_u16BatteryTemperature = MCP9843ReadTemperature();
            
            // 3. Update discharge state based on target
            updateDischargeState();
            
            // 4. Prepare for transmission
            sg_eCellAction = EACTION_INITIATE_TRANSMIT;
            break;
            
        case EACTION_SEND_PATTERN:
            // Generate test pattern for diagnostics
            generateTestPattern();
            sg_eCellAction = EACTION_INITIATE_TRANSMIT;
            break;
            
        case EACTION_INITIATE_TRANSMIT:
            // Only last cell initiates transmission
            if (sg_bCellCPULast) {
                vUARTStartcell_dn_tx(VUART_BIT_TICKS / 2);
            }
            sg_eCellAction = EACTION_NONE;
            break;
            
        default:
            // Idle state - wait for command
            break;
    }
}
```

## Build System

### Project Configuration

The CellCPU.cproj file defines the build configuration for Microchip Studio:

```xml
Build Configuration:
├── Target Device: ATtiny45
├── Compiler: XC8 v2.36
├── Optimization: -Os (optimize for size)
├── Debug Format: Dwarf-2
├── Programming Interface: ISP
└── Fuse Configuration: Internal 8MHz RC oscillator
```

### Memory Optimization

```c
Compiler Optimization Strategies:
├── Size Optimization: -Os flag prioritizes code size
├── Register Usage: Automatic register allocation for frequently used variables
├── Function Inlining: Small functions inlined to reduce call overhead
├── Dead Code Elimination: Unused functions removed by linker
├── Constant Folding: Compile-time calculation of constant expressions
└── Loop Optimization: Efficient loop constructs for timing-critical code
```

### Development Workflow

```
Development Process:
├── Code Development: Microchip Studio IDE
├── Compilation: XC8 compiler with size optimization
├── Simulation: Microchip Studio simulator for basic testing
├── Programming: ISP interface (6-pin header)
├── Testing: Hardware-in-the-loop with ModuleCPU
└── Production: Automated programming and test
```

The CellCPU firmware architecture demonstrates how sophisticated functionality can be implemented within severe resource constraints through careful modular design, efficient algorithms, and optimized resource utilization.