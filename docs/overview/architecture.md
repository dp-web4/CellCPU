# CellCPU Software Architecture

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Software Design Patterns](#software-design-patterns)
3. [State Machine Implementation](#state-machine-implementation)
4. [Communication Architecture](#communication-architecture)
5. [Data Flow Design](#data-flow-design)
6. [Memory Management](#memory-management)
7. [Interrupt Architecture](#interrupt-architecture)
8. [Error Handling Strategy](#error-handling-strategy)

## Architecture Overview

The CellCPU firmware implements a minimalist, event-driven architecture optimized for the resource constraints of the ATtiny45 microcontroller. The design prioritizes reliability, power efficiency, and deterministic real-time behavior.

### High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    CellCPU Software Stack                       │
├─────────────────────────────────────────────────────────────────┤
│ Application Layer                                               │
│ ├── Main Control Loop                                          │
│ ├── Cell State Management                                      │
│ ├── Data Processing & Formatting                               │
│ └── Safety & Protection Logic                                  │
├─────────────────────────────────────────────────────────────────┤
│ Communication Layer                                             │
│ ├── Virtual UART Protocol                                      │
│ ├── Message Parsing & Generation                               │
│ ├── Command Processing                                          │
│ └── Data Aggregation                                           │
├─────────────────────────────────────────────────────────────────┤
│ Hardware Abstraction Layer                                      │
│ ├── ADC Interface (Voltage Measurement)                        │
│ ├── I2C Interface (Temperature & Control)                      │
│ ├── GPIO Control (Communication Pins)                          │
│ └── Timer Management (UART Bit Timing)                         │
├─────────────────────────────────────────────────────────────────┤
│ Platform Layer                                                  │
│ ├── ATtiny45 Hardware Registers                                │
│ ├── Interrupt Vector Table                                     │
│ ├── Clock & Power Management                                   │
│ └── Pin Configuration & Control                                │
└─────────────────────────────────────────────────────────────────┘
```

### Core Design Principles

1. **Minimal Resource Usage**: Optimized for 4KB flash and 256B RAM
2. **Event-Driven**: Responds to communication events and measurement requests
3. **Stateless Operation**: No persistent state beyond current measurements
4. **Fail-Safe Design**: Default to safe states on errors or communication loss
5. **Deterministic Timing**: Predictable response times for real-time operation

## Software Design Patterns

### Event-Driven State Machine

The firmware implements a simple state machine driven by communication events:

```c
typedef enum {
    EACTION_NONE,                    // Idle state, waiting for events
    EACTION_SEND_SENSOR_READING,     // Perform measurement and transmit
    EACTION_SEND_PATTERN,            // Transmit test pattern
    EACTION_INIT,                    // Initial setup and configuration
    EACTION_INITIATE_TRANSMIT        // Start data transmission sequence
} ECellAction;

// State transition logic
void processAction(ECellAction action) {
    switch (action) {
        case EACTION_INIT:
            initializeCell();
            detectChainPosition();
            configureInterrupts();
            sg_eCellAction = EACTION_NONE;
            break;
            
        case EACTION_SEND_SENSOR_READING:
            performMeasurements();
            updateDischargeState();
            sg_eCellAction = EACTION_INITIATE_TRANSMIT;
            break;
            
        case EACTION_SEND_PATTERN:
            generateTestPattern();
            sg_eCellAction = EACTION_INITIATE_TRANSMIT;
            break;
            
        case EACTION_INITIATE_TRANSMIT:
            if (isLastCellInChain()) {
                startDataTransmission();
            }
            sg_eCellAction = EACTION_NONE;
            break;
            
        default:
            // Idle state - wait for next event
            break;
    }
}
```

### Producer-Consumer Pattern

The communication system implements a producer-consumer pattern for data transmission:

```c
// Data production (measurement and formatting)
void produceData(void) {
    // Producer: Generate cell data
    sg_u16BatteryVoltage = ADCRead();
    sg_u16BatteryTemperature = MCP9843ReadTemperature();
    
    // Format data with status flags
    if (sg_bDischargeActive) {
        sg_u16BatteryVoltage |= MSG_CELL_DISCHARGE_ACTIVE;
    }
}

// Data consumption (transmission protocol)
uint8_t Celldn_txDataGet(void) {
    // Consumer: Serialize data for transmission
    switch (sg_u8TransmitOffset) {
        case 0: return (uint8_t)sg_u16BatteryVoltage;        // LSB
        case 1: return (uint8_t)(sg_u16BatteryVoltage >> 8); // MSB
        case 2: return (uint8_t)sg_u16BatteryTemperature;    // LSB
        case 3: return (uint8_t)(sg_u16BatteryTemperature >> 8); // MSB
    }
    sg_u8TransmitOffset++;
    return 0xFF; // Error case
}
```

### Chain of Responsibility Pattern

Command processing follows a chain-of-responsibility pattern where each cell processes commands and passes them along:

```c
void Celldn_rxDataBit(uint8_t u8DataBit) {
    // Build received command
    sg_u16BatteryVoltageMsg <<= 1;
    sg_u16BatteryVoltageMsg |= u8DataBit;
    sg_u8dn_rxBitCount++;
    
    // Early command detection for fast response
    if (2 == sg_u8dn_rxBitCount) {
        if (sg_u16BatteryVoltageMsg & (MSG_CELL_SEND_REPORT >> 14)) {
            // Process command immediately
            if (sg_u16BatteryVoltageMsg & (MSG_CELL_SEND_PATTERN >> 14)) {
                sg_eCellAction = EACTION_SEND_PATTERN;
            } else {
                sg_eCellAction = EACTION_SEND_SENSOR_READING;
            }
        }
    }
    
    // Complete command processing
    if (16 == sg_u8dn_rxBitCount) {
        if (!(sg_u16BatteryVoltageMsg & MSG_CELL_SEND_REPORT)) {
            // This is a voltage target command
            sg_u16BatteryVoltageTarget = sg_u16BatteryVoltageMsg;
        }
        sg_u16BatteryVoltageMsg = 0;
    }
}
```

## State Machine Implementation

### Cell State Transitions

```
Cell State Machine:
┌─────────────┐    Power On     ┌─────────────┐
│   RESET     │─────────────────▶│    INIT     │
└─────────────┘                  └──────┬──────┘
                                        │ Hardware Setup
                                        │ Chain Detection
                                        ▼
                                 ┌─────────────┐
                                 │    IDLE     │◄─┐
                                 └──────┬──────┘  │
                                        │         │
                                Command │         │ Complete
                                Received│         │
                                        ▼         │
                                 ┌─────────────┐  │
                         ┌──────▶│  MEASURE    │  │
                         │       └──────┬──────┘  │
                   Test  │              │         │
                 Pattern │              │ Sensor  │
                         │              │ Reading │
                         │              ▼         │
                 ┌─────────────┐ ┌─────────────┐  │
                 │   PATTERN   │ │  TRANSMIT   │──┘
                 └──────┬──────┘ └──────┬──────┘
                        │               │
                        └───────────────┘
                           Transmit
```

### Chain Position Detection

Critical for determining communication behavior:

```c
void detectChainPosition(void) {
    // Initially assume we're the last cell
    sg_bCellCPULast = true;
    
    // Assert cell_dn_tx to signal our presence to downstream cells
    CELL_DN_TX_ASSERT();
    
    // Sample multiple times to ensure stable detection
    uint8_t sample_count = 10;
    while (sample_count--) {
        if (IS_PIN_CELL_UP_RX_ASSERTED()) {
            // An upstream cell is present
            sg_bCellCPULast = false;
        }
        Delay(1000); // 1ms between samples
    }
    
    // Configure interrupts based on position
    if (!sg_bCellCPULast) {
        // Middle/first cell: enable receive interrupts
        vUARTInitReceive();
    }
    
    // Signal end of detection window
    CELL_DN_TX_DEASSERT();
    
    // All cells enable transmit capability
    vUARTInitTransmit();
}
```

## Communication Architecture

### Virtual UART Implementation

The communication system implements a software-based UART using timer interrupts:

```c
// UART timing constants
#define VUART_BIT_TICKS    50        // 50μs bit time = 20kbps
#define CPU_SPEED          8000000   // 8MHz system clock

// Bit-level transmission state machine
typedef enum {
    UART_IDLE,
    UART_START_BIT,
    UART_DATA_BITS,
    UART_STOP_BIT
} UARTState;

// Timer-based bit generation
ISR(TIMER_COMPA_VECTOR) {
    static UARTState txState = UART_IDLE;
    static uint8_t bitCount = 0;
    static uint8_t currentByte = 0;
    
    switch (txState) {
        case UART_START_BIT:
            // Transmit start bit (low)
            if (transmittingUpstream) {
                CELL_UP_TX_ASSERT();
            } else {
                CELL_DN_TX_ASSERT();
            }
            txState = UART_DATA_BITS;
            bitCount = 0;
            break;
            
        case UART_DATA_BITS:
            // Transmit data bits LSB first
            if (currentByte & (1 << bitCount)) {
                if (transmittingUpstream) {
                    CELL_UP_TX_DEASSERT();  // Logic 1
                } else {
                    CELL_DN_TX_DEASSERT();
                }
            } else {
                if (transmittingUpstream) {
                    CELL_UP_TX_ASSERT();    // Logic 0
                } else {
                    CELL_DN_TX_ASSERT();
                }
            }
            
            bitCount++;
            if (bitCount >= 8) {
                txState = UART_STOP_BIT;
            }
            break;
            
        case UART_STOP_BIT:
            // Transmit stop bit (high)
            if (transmittingUpstream) {
                CELL_UP_TX_DEASSERT();
            } else {
                CELL_DN_TX_DEASSERT();
            }
            
            // Check for more data
            if (Celldn_txDataAvailable()) {
                currentByte = Celldn_txDataGet();
                txState = UART_START_BIT;
            } else {
                txState = UART_IDLE;
                // Disable timer interrupt
                TIMER_CHA_INT_DISABLE();
            }
            break;
    }
    
    // Schedule next bit
    if (txState != UART_IDLE) {
        TIMER_CHA_INT(VUART_BIT_TICKS);
    }
}
```

### Message Protocol Structure

```c
// Command message format (ModuleCPU → CellCPUs)
#define MSG_CELL_SEND_REPORT     0x8000    // Bit 15: Request data
#define MSG_CELL_SEND_PATTERN    0x4000    // Bit 14: Send test pattern
#define MSG_CELL_DISCHARGE_ACTIVE 0x8000   // Bit 15: Discharge status

// Data message format (CellCPUs → ModuleCPU)
typedef struct {
    uint16_t voltage;      // Raw ADC reading or scaled millivolts
    uint16_t temperature;  // Temperature in sensor-specific format
} CellDataMessage;

// Status encoding
#define TEMP_I2C_ERROR_FLAG     0x8000    // Bit 15 of temperature
#define VOLTAGE_DISCHARGE_FLAG  0x8000    // Bit 15 of voltage
```

## Data Flow Design

### Measurement Pipeline

```c
void performMeasurements(void) {
    // 1. Voltage measurement (ADC)
    uint16_t voltage = ADCRead();
    sg_u16BatteryVoltage = voltage;
    
    // 2. Temperature measurement (I2C)
    uint16_t temperature = MCP9843ReadTemperature();
    
    // 3. Discharge state evaluation
    bool dischargeNeeded = (voltage > sg_u16BatteryVoltageTarget);
    
    // 4. Update discharge state if changed
    if (dischargeNeeded != sg_bDischargeActive) {
        sg_bDischargeActive = dischargeNeeded;
        MCP9843SetEventPin(!sg_bDischargeActive); // Active low
    }
    
    // 5. Format data with status flags
    if (sg_bDischargeActive) {
        voltage |= MSG_CELL_DISCHARGE_ACTIVE;
    }
    
    // Check temperature I2C status
    if (temperature == TEMPERATURE_ERROR_VALUE) {
        temperature |= TEMP_I2C_ERROR_FLAG;
    }
    
    // 6. Store formatted data
    sg_u16BatteryVoltage = voltage;
    sg_u16BatteryTemperature = temperature;
}
```

### Communication Flow Control

```c
// Upstream data flow (cells → module)
void initiateUpstreamTransmission(void) {
    // Only the last cell in chain initiates transmission
    if (sg_bCellCPULast) {
        // Wait for any downstream activity to complete
        while (vUARTIscell_dn_rxActive()) {
            // Busy wait with timeout
        }
        
        // Start transmission with small delay for signal settling
        vUARTStartcell_dn_tx(VUART_BIT_TICKS/2);
    }
    // Other cells will automatically relay when they receive data
}

// Downstream command flow (module → cells)
void relayDownstreamCommand(void) {
    // All cells automatically relay received commands
    // This is handled by the UART receive interrupt
    // and transmission state machine
}
```

## Memory Management

### Stack and Heap Usage

```c
// Memory allocation strategy for ATtiny45 (256 bytes SRAM)
Memory Layout:
├── Stack: ~50 bytes (grows downward from 0x15F)
├── Global Variables: ~100 bytes (static allocation)
├── Function Locals: ~50 bytes (automatic variables)
├── ISR Context: ~20 bytes (interrupt stack usage)
└── Available: ~36 bytes (safety margin)

// Critical global variables
static uint16_t sg_u16BatteryVoltage;        // 2 bytes
static volatile uint16_t sg_u16BatteryTemperature; // 2 bytes
static volatile uint16_t sg_u16BatteryVoltageTarget; // 2 bytes
static volatile bool sg_bDischargeActive;    // 1 byte
static volatile bool sg_bCellCPULast;       // 1 byte
static ECellAction sg_eCellAction;          // 1 byte
static uint16_t sg_u16BatteryVoltageMsg;    // 2 bytes
static uint8_t sg_u8TransmitOffset;         // 1 byte
static uint8_t sg_u8dn_rxBitCount;         // 1 byte
// Total: ~13 bytes for core state
```

### Flash Memory Organization

```c
Flash Memory Usage (4KB total):
├── Interrupt Vectors: 26 bytes (0x0000-0x0019)
├── Program Code: ~2.5KB (application logic)
├── Constant Data: ~100 bytes (lookup tables, strings)
├── Unused Space: ~1.4KB (available for features)
└── Bootloader: Not used (direct programming)

// Code optimization strategies
- Use uint8_t instead of int where possible
- Minimize function call depth
- Use register variables for frequently accessed data
- Optimize ISR code for minimal stack usage
```

## Interrupt Architecture

### Interrupt Priority and Handling

```c
// Interrupt vector priorities (hardware defined)
Interrupt Priority Order:
├── 1. RESET (highest)
├── 2. INT0 (external interrupt 0)
├── 3. PCINT0 (pin change interrupt) ← Used for UART RX
├── 4. TIMER1_COMPA (timer 1 compare A)
├── 5. TIMER1_OVF (timer 1 overflow)  
├── 6. TIMER0_OVF (timer 0 overflow)
├── 7. EE_RDY (EEPROM ready)
├── 8. ANA_COMP (analog comparator)
├── 9. ADC (ADC conversion complete)
├── 10. TIMER1_COMPB (timer 1 compare B)
├── 11. TIMER0_COMPA (timer 0 compare A) ← Used for UART TX
├── 12. TIMER0_COMPB (timer 0 compare B)
├── 13. WDT (watchdog timer)
└── 14. USI_START (USI start condition)

// UART receive interrupt (highest priority communication)
ISR(PCINT_VECTOR) {
    // Process pin change for UART receive
    vUARTReceiveISR();
}

// UART transmit interrupt (timer-based bit generation)
ISR(TIMER_COMPA_VECTOR) {
    // Generate next UART bit
    vUARTTransmitISR();
}

// Unused interrupt handlers (for debugging)
ISR(INT0_vect, ISR_BLOCK) { while(1); }      // Trap unexpected interrupts
ISR(TIMER1_OVF_vect, ISR_BLOCK) { while(1); }
// ... (additional trap handlers)
```

### Interrupt Synchronization

```c
// Critical section protection
#define ENTER_CRITICAL_SECTION() \
    do { \
        uint8_t sreg_save = SREG; \
        cli();

#define EXIT_CRITICAL_SECTION() \
        SREG = sreg_save; \
    } while(0)

// Example usage in shared data access
void updateDischargeState(bool newState) {
    ENTER_CRITICAL_SECTION();
    if (sg_bDischargeActive != newState) {
        sg_bDischargeActive = newState;
        // Update hardware state
        MCP9843SetEventPin(!newState);
    }
    EXIT_CRITICAL_SECTION();
}
```

## Error Handling Strategy

### Fault Detection and Response

```c
// Error categories and responses
typedef enum {
    ERROR_NONE = 0,
    ERROR_I2C_TIMEOUT,          // Temperature sensor communication
    ERROR_ADC_INVALID,          // Voltage measurement failure
    ERROR_COMMUNICATION_LOSS,   // Loss of ModuleCPU communication
    ERROR_VOLTAGE_OUT_OF_RANGE, // Cell voltage safety limits
} ErrorType;

// Error handling strategy
void handleError(ErrorType error) {
    switch (error) {
        case ERROR_I2C_TIMEOUT:
            // Flag temperature as invalid
            sg_u16BatteryTemperature |= TEMP_I2C_ERROR_FLAG;
            // Continue operation with voltage monitoring only
            break;
            
        case ERROR_ADC_INVALID:
            // Use last known good voltage reading
            // Report error in next communication cycle
            break;
            
        case ERROR_COMMUNICATION_LOSS:
            // Disable discharge for safety
            sg_bDischargeActive = false;
            MCP9843SetEventPin(true); // Discharge off
            break;
            
        case ERROR_VOLTAGE_OUT_OF_RANGE:
            // Emergency discharge disable
            sg_bDischargeActive = false;
            MCP9843SetEventPin(true);
            // Continue monitoring
            break;
    }
}

// Watchdog timer backup (if enabled)
void initWatchdog(void) {
    // Configure for ~1 second timeout
    WDTCR = (1 << WDCE) | (1 << WDE);
    WDTCR = (1 << WDE) | (1 << WDP2) | (1 << WDP1);
}

void resetWatchdog(void) {
    wdt_reset(); // Reset watchdog timer
}
```

### Graceful Degradation

```c
// System continues operation with reduced functionality on errors
void performDegradedOperation(void) {
    // Always attempt voltage measurement
    uint16_t voltage = ADCRead();
    if (voltage > 0 && voltage < 1024) {
        sg_u16BatteryVoltage = voltage;
    }
    
    // Attempt temperature reading with timeout
    uint16_t temperature = MCP9843ReadTemperatureWithTimeout(100); // 100ms max
    if (temperature != TEMPERATURE_TIMEOUT_VALUE) {
        sg_u16BatteryTemperature = temperature;
    } else {
        // Use error flag
        sg_u16BatteryTemperature = TEMPERATURE_INVALID | TEMP_I2C_ERROR_FLAG;
    }
    
    // Conservative discharge management
    if (voltage > (sg_u16BatteryVoltageTarget + SAFETY_MARGIN)) {
        // Only enable discharge if well above target
        sg_bDischargeActive = true;
    } else {
        // Disable discharge on any uncertainty
        sg_bDischargeActive = false;
    }
    
    // Update hardware state
    MCP9843SetEventPin(!sg_bDischargeActive);
}
```

The CellCPU architecture demonstrates how sophisticated functionality can be achieved within severe resource constraints through careful design choices, optimized algorithms, and robust error handling strategies.