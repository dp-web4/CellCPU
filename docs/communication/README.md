# CellCPU Communication with ModuleCPU

## Table of Contents

1. [Communication Overview](#communication-overview)
2. [Physical Layer Implementation](#physical-layer-implementation)
3. [Protocol Architecture](#protocol-architecture)
4. [Message Format and Structure](#message-format-and-structure)
5. [Daisy Chain Topology](#daisy-chain-topology)
6. [Virtual UART Implementation](#virtual-uart-implementation)
7. [Data Flow and Timing](#data-flow-and-timing)
8. [Error Handling and Recovery](#error-handling-and-recovery)
9. [Performance Characteristics](#performance-characteristics)

## Communication Overview

The CellCPU communication system implements a custom serial protocol that enables the ModuleCPU to communicate with up to 94 individual CellCPUs in a daisy-chain configuration. This design provides scalable, reliable communication while minimizing wiring complexity and cost.

### System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                Communication System Architecture               │
├─────────────────────────────────────────────────────────────────┤
│ ModuleCPU (ATmega64M1)                                         │
│ ├── Serial Interface (Hardware UART)                          │
│ ├── Command Generation and Data Collection                    │
│ ├── Error Detection and Recovery                              │
│ └── CAN Bus Interface (to Pack Controller)                    │
├─────────────────────────────────────────────────────────────────┤
│ CellCPU Chain (1 to 94 cells)                                 │
│ ├── Virtual UART Implementation                               │
│ ├── Daisy-Chain Serial Communication                          │
│ ├── Command Relay and Data Aggregation                        │
│ └── Individual Cell Monitoring and Control                     │
├─────────────────────────────────────────────────────────────────┤
│ Physical Layer                                                 │
│ ├── TTL/CMOS Logic Levels (0V/VCC)                           │
│ ├── 20kbps Bit Rate (50μs bit time)                          │
│ ├── 8-N-1 Serial Format                                       │
│ └── 4-Wire Interface (2 TX, 2 RX)                            │
└─────────────────────────────────────────────────────────────────┘
```

### Key Communication Features

```
Communication Specifications:
├── Protocol: Custom serial protocol over Virtual UART
├── Bit Rate: 20kbps (50μs bit time)
├── Data Format: 8 data bits, no parity, 1 stop bit
├── Topology: Daisy-chain (linear chain of devices)
├── Maximum Cells: 94 CellCPUs per ModuleCPU
├── Message Size: 4 bytes per cell (376 bytes total for 94 cells)
├── Update Rate: Configurable (typically 1-10Hz)
├── Error Detection: Built-in timeout and validation
├── Power Consumption: <1mA per CellCPU during communication
└── Cable Length: Up to several meters (depends on signal conditioning)
```

## Physical Layer Implementation

### Pin Configuration and Signal Routing

```
CellCPU Communication Pins:
├── PB0 (cell_up_tx):   Output to upstream cell/ModuleCPU
├── PB1 (cell_dn_rx):   Input from downstream cell/ModuleCPU  
├── PB4 (cell_up_rx):   Input from upstream cell
├── PB5 (cell_dn_tx):   Output to downstream cell
└── Pin States:         TTL/CMOS logic levels (0V = assert, VCC = deassert)

Signal Chain Configuration:
┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│ ModuleCPU   │    │ CellCPU #1  │    │ CellCPU #2  │    │ CellCPU #94 │
│             │───▶│ (Last)      │───▶│ (Middle)    │───▶│ (First)     │
│ cell_dn_tx  │    │cell_dn_rx   │    │cell_dn_rx   │    │cell_dn_rx   │
│             │    │cell_up_tx   │    │cell_up_tx   │    │cell_up_tx   │
│ cell_dn_rx  │◄───│             │◄───│             │◄───│             │
└─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘
   Commands            Relay             Relay             End of Chain
   Data Collection     Append Data       Append Data       Initiate TX
```

### Electrical Characteristics

```c
// Signal conditioning and protection
Signal Characteristics:
├── Logic Levels:
│   ├── Logic 0 (Assert):    0V to 0.3×VCC
│   ├── Logic 1 (Deassert):  0.7×VCC to VCC
│   ├── Input Threshold:     0.5×VCC (hysteresis)
│   └── Output Drive:        20mA maximum
├── Signal Integrity:
│   ├── Rise/Fall Time:      <1μs (much faster than 50μs bit time)
│   ├── Series Resistance:   100Ω (damping and current limiting)
│   ├── Input Filtering:     RC filter (1kΩ + 100pF)
│   └── ESD Protection:      TVS diodes on all communication pins
├── Timing Accuracy:
│   ├── Bit Time Tolerance:  ±1% (±0.5μs on 50μs bit time)
│   ├── Clock Source:        Internal 8MHz RC oscillator (±10%)
│   ├── Timer Precision:     Timer0 with prescaler for bit timing
│   └── Jitter:             <0.1μs (negligible compared to bit time)
└── Loading:
    ├── Input Impedance:     >100kΩ (minimal loading)
    ├── Input Capacitance:   ~10pF per input
    ├── Chain Loading:       ~940pF total for 94 cells
    └── Drive Capability:    Adequate for intended chain length
```

## Protocol Architecture

### Communication Model

The CellCPU communication protocol implements a master-slave model with command broadcast and data aggregation:

```c
// Communication protocol layers
Protocol Stack:
├── Application Layer:
│   ├── Cell data reporting (voltage, temperature, status)
│   ├── Discharge control commands  
│   ├── Test pattern generation
│   └── Diagnostic information
├── Message Layer:
│   ├── 4-byte cell data format
│   ├── 2-byte command format
│   ├── Status flag encoding
│   └── Error indication methods
├── Transport Layer:
│   ├── Data aggregation (multiple cells)
│   ├── Command relay and processing
│   ├── Timeout and retry mechanisms
│   └── Chain position management
├── Data Link Layer:
│   ├── Byte-level transmission
│   ├── Start/stop bit generation
│   ├── Bit timing control
│   └── Basic error detection
└── Physical Layer:
    ├── Virtual UART implementation
    ├── GPIO pin manipulation
    ├── Timer-based bit generation
    └── Interrupt-driven reception
```

### Command Processing Flow

```c
// Command processing state machine
typedef enum {
    CMD_STATE_IDLE,           // Waiting for command
    CMD_STATE_RECEIVING,      // Actively receiving command
    CMD_STATE_PROCESSING,     // Processing received command
    CMD_STATE_RELAYING,       // Relaying command to next cell
    CMD_STATE_RESPONDING      // Generating response data
} CommandState;

// Command processing workflow
void processIncomingCommand(void) {
    // 1. Command Reception (from ModuleCPU or previous cell)
    //    - Receive on cell_dn_rx pin
    //    - Parse command type and parameters
    //    - Early detection for fast response
    
    // 2. Local Command Processing
    //    - Determine if command applies to this cell
    //    - Execute measurement or control actions
    //    - Prepare response data
    
    // 3. Command Relay (if not last cell)
    //    - Forward command to next cell via cell_up_tx
    //    - Maintain command integrity and timing
    //    - Continue chain propagation
    
    // 4. Response Generation
    //    - Wait for upstream cells to complete
    //    - Append local data to response stream
    //    - Transmit combined data to ModuleCPU
}
```

## Message Format and Structure

### Command Messages (Downstream: ModuleCPU → CellCPUs)

```c
// Command message format (16-bit commands)
#define MSG_CELL_SEND_REPORT     0x8000    // Bit 15: Request sensor data
#define MSG_CELL_SEND_PATTERN    0x4000    // Bit 14: Request test pattern
#define MSG_VOLTAGE_TARGET_MASK  0x3FFF    // Bits 13-0: Voltage target value

// Command message structure
typedef union {
    uint16_t raw;
    struct {
        uint16_t voltageTarget : 14;    // Voltage target (0-16383)
        uint16_t sendPattern   : 1;     // Test pattern request
        uint16_t sendReport    : 1;     // Data report request
    } fields;
} CommandMessage;

// Command processing implementation
void Celldn_rxDataBit(uint8_t u8DataBit) {
    // Build 16-bit command word
    sg_u16BatteryVoltageMsg <<= 1;
    sg_u16BatteryVoltageMsg |= u8DataBit;
    sg_u8dn_rxBitCount++;
    
    // Early command detection for immediate response
    if (2 == sg_u8dn_rxBitCount) {
        // Check command type from first 2 bits
        if (sg_u16BatteryVoltageMsg & (MSG_CELL_SEND_REPORT >> 14)) {
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
        // Reset for next command
        sg_u16BatteryVoltageMsg = 0;
        sg_u8dn_rxBitCount = 0;
    }
}
```

### Data Messages (Upstream: CellCPUs → ModuleCPU)

```c
// Cell data message format (4 bytes per cell)
Cell Data Message Structure:
├── Byte 0: Voltage LSB (bits 7-0)
├── Byte 1: Voltage MSB (bits 15-8) + Discharge Status (bit 15)
├── Byte 2: Temperature LSB (bits 7-0) 
└── Byte 3: Temperature MSB (bits 15-8) + I2C Status (bit 15)

// Status bit definitions
#define MSG_CELL_DISCHARGE_ACTIVE 0x8000    // Bit 15 of voltage word
#define TEMP_I2C_ERROR_FLAG      0x8000    // Bit 15 of temperature word

// Data transmission implementation
uint8_t Celldn_txDataGet(void) {
    // Return next byte of cell data
    switch (sg_u8TransmitOffset) {
        case 0: 
            return (uint8_t)sg_u16BatteryVoltage;        // Voltage LSB
        case 1: 
            return (uint8_t)(sg_u16BatteryVoltage >> 8); // Voltage MSB + flags
        case 2: 
            return (uint8_t)sg_u16BatteryTemperature;    // Temperature LSB
        case 3: 
            return (uint8_t)(sg_u16BatteryTemperature >> 8); // Temperature MSB + flags
        default:
            return 0xFF;  // Error case
    }
}

// Example data encoding
void encodeCell Data(uint16_t voltage, uint16_t temperature, bool discharging, bool i2cError) {
    // Voltage with discharge status
    if (discharging) {
        voltage |= MSG_CELL_DISCHARGE_ACTIVE;
    }
    sg_u16BatteryVoltage = voltage;
    
    // Temperature with I2C error status  
    if (i2cError) {
        temperature |= TEMP_I2C_ERROR_FLAG;
    }
    sg_u16BatteryTemperature = temperature;
}
```

### Aggregate Message Format

```c
// Complete message for 94 cells (376 bytes total)
Complete Message Structure:
├── Cell #1 Data:    4 bytes (voltage + temperature + status)
├── Cell #2 Data:    4 bytes 
├── ...
├── Cell #94 Data:   4 bytes
└── Total Size:      376 bytes

// Message timing calculation
Transmission Timing:
├── Bit Time:        50μs
├── Bits per Byte:   10 (8 data + 1 start + 1 stop)
├── Bytes per Cell:  4
├── Total Bits:      94 cells × 4 bytes × 10 bits = 3,760 bits
├── Total Time:      3,760 × 50μs = 188ms
└── Effective Rate:  376 bytes / 188ms = 2.0 kB/s
```

## Daisy Chain Topology

### Chain Position Detection

CellCPUs must automatically detect their position in the chain to determine communication behavior:

```c
// Chain position detection algorithm
void detectChainPosition(void) {
    // Initially assume this is the last cell in chain
    sg_bCellCPULast = true;
    
    // Assert downstream TX to signal presence
    CELL_DN_TX_ASSERT();
    
    // Sample upstream RX multiple times for stability
    uint8_t sampleCount = 10;
    uint8_t assertedSamples = 0;
    
    while (sampleCount--) {
        if (IS_PIN_CELL_UP_RX_ASSERTED()) {
            assertedSamples++;
        }
        Delay(1000);  // 1ms between samples
    }
    
    // If upstream cell detected, we're not the last
    if (assertedSamples > 5) {  // Majority voting
        sg_bCellCPULast = false;
    }
    
    // Release detection signal
    CELL_DN_TX_DEASSERT();
    
    // Configure communication based on position
    if (!sg_bCellCPULast) {
        // Middle cells: enable command reception
        vUARTInitReceive();
    }
    
    // All cells: enable data transmission capability
    vUARTInitTransmit();
}

// Chain communication roles
Cell Position Roles:
├── Last Cell (closest to ModuleCPU):
│   ├── Receives commands from ModuleCPU
│   ├── Initiates upstream data transmission
│   ├── Relays commands to upstream cells
│   └── Appends own data to upstream messages
├── Middle Cells:
│   ├── Receive commands from downstream cell
│   ├── Relay commands to upstream cells
│   ├── Append data to upstream transmission
│   └── Maintain signal integrity through chain
└── First Cell (end of chain):
    ├── Receives commands from downstream cell
    ├── Terminates command propagation
    ├── Initiates data response sequence
    └── Begins upstream data aggregation
```

### Data Aggregation Process

```c
// Data aggregation in daisy-chain
void aggregateChainData(void) {
    // Phase 1: Command propagation (downstream)
    // - Commands flow from ModuleCPU through last cell to first cell
    // - Each cell processes commands and triggers local measurements
    // - Command relay maintains timing and signal integrity
    
    // Phase 2: Data collection preparation
    // - All cells perform measurements simultaneously  
    // - Data formatting and status flag preparation
    // - Wait for transmission initiation signal
    
    // Phase 3: Data transmission (upstream)
    // - First cell initiates transmission sequence
    // - Each cell appends its 4-byte data to the stream
    // - Data flows from first cell back to ModuleCPU
    // - ModuleCPU collects complete 376-byte message
}

// Data transmission state machine
typedef enum {
    TX_STATE_IDLE,               // No transmission activity
    TX_STATE_WAITING_INITIATE,   // Waiting for transmission start
    TX_STATE_TRANSMITTING,       // Actively transmitting data
    TX_STATE_RELAYING,          // Relaying upstream data
    TX_STATE_COMPLETE           // Transmission finished
} TransmissionState;

TransmissionState handleDataTransmission(void) {
    static TransmissionState state = TX_STATE_IDLE;
    
    switch (state) {
        case TX_STATE_IDLE:
            if (sg_eCellAction == EACTION_INITIATE_TRANSMIT) {
                if (sg_bCellCPULast) {
                    // Last cell initiates transmission
                    vUARTStartcell_dn_tx(VUART_BIT_TICKS/2);
                    state = TX_STATE_TRANSMITTING;
                } else {
                    // Other cells wait for upstream activity
                    state = TX_STATE_WAITING_INITIATE;
                }
            }
            break;
            
        case TX_STATE_WAITING_INITIATE:
            if (vUARTIscell_dn_rxActive()) {
                // Upstream transmission detected
                state = TX_STATE_RELAYING;
            }
            break;
            
        case TX_STATE_TRANSMITTING:
            if (!vUARTTransmissionActive()) {
                state = TX_STATE_COMPLETE;
            }
            break;
            
        case TX_STATE_RELAYING:
            if (!vUARTIscell_dn_rxActive()) {
                state = TX_STATE_COMPLETE;
            }
            break;
            
        case TX_STATE_COMPLETE:
            state = TX_STATE_IDLE;
            sg_eCellAction = EACTION_NONE;
            break;
    }
    
    return state;
}
```

## Virtual UART Implementation

### Timer-Based Bit Generation

```c
// Virtual UART timing and control
#define VUART_BIT_TICKS    50        // 50μs bit time
#define CPU_SPEED          8000000   // 8MHz internal oscillator

// Timer configuration for UART bit timing
void configureUARTTimer(void) {
    // Timer0 setup for 50μs intervals
    // Prescaler calculation: 8MHz / 64 = 125kHz
    // Timer ticks for 50μs: 125kHz × 50μs = 6.25 ≈ 6 ticks
    
    TCCR0 = (1 << CS01) | (1 << CS00);  // Prescaler /64
    TIMSK |= (1 << OCIE0A);             // Enable compare match interrupt
}

// UART transmission interrupt service routine
ISR(TIMER0_COMPA_vect) {
    static uint8_t bitPosition = 0;
    static uint8_t currentByte = 0;
    static UARTState txState = UART_IDLE;
    
    switch (txState) {
        case UART_START_BIT:
            // Transmit start bit (logic 0)
            if (transmittingUpstream) {
                CELL_UP_TX_ASSERT();
            } else {
                CELL_DN_TX_ASSERT();
            }
            txState = UART_DATA_BITS;
            bitPosition = 0;
            break;
            
        case UART_DATA_BITS:
            // Transmit data bits (LSB first)
            if (currentByte & (1 << bitPosition)) {
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
            
            bitPosition++;
            if (bitPosition >= 8) {
                txState = UART_STOP_BIT;
            }
            break;
            
        case UART_STOP_BIT:
            // Transmit stop bit (logic 1)
            if (transmittingUpstream) {
                CELL_UP_TX_DEASSERT();
            } else {
                CELL_DN_TX_DEASSERT();
            }
            
            // Check for more data to transmit
            if (moreDataAvailable()) {
                currentByte = getNextDataByte();
                txState = UART_START_BIT;
            } else {
                txState = UART_IDLE;
                TIMER_CHA_INT_DISABLE();  // Stop timer interrupt
            }
            break;
    }
    
    // Schedule next bit time if still transmitting
    if (txState != UART_IDLE) {
        TIMER_CHA_INT(VUART_BIT_TICKS);
    }
}
```

### Reception and Interrupt Handling

```c
// UART reception using pin change interrupts
ISR(PCINT_vect) {
    // Pin change interrupt for start bit detection
    static uint8_t lastPinState = 0xFF;
    uint8_t currentPinState = PINB;
    
    // Check for start bit on cell_dn_rx (PB1)
    if ((lastPinState & (1 << PB1)) && !(currentPinState & (1 << PB1))) {
        // Falling edge detected - start bit
        vUARTStartBitDetected();
    }
    
    // Check for start bit on cell_up_rx (PB4) if middle cell
    if (!sg_bCellCPULast) {
        if ((lastPinState & (1 << PB4)) && !(currentPinState & (1 << PB4))) {
            vUARTUpstreamStartBitDetected();
        }
    }
    
    lastPinState = currentPinState;
}

// Start bit detection and bit sampling setup
void vUARTStartBitDetected(void) {
    // Setup timer for data bit sampling
    // Sample in middle of bit time (25μs after start bit edge)
    TIMER_CHA_INT(VUART_BIT_TICKS / 2);
    TIMER_CHA_INT_ENABLE();
    
    // Initialize reception state
    sg_u8RxBitCount = 0;
    sg_u8RxDataByte = 0;
    sg_bReceiving = true;
}

// Bit sampling for reception
void sampleDataBit(void) {
    if (IS_PIN_CELL_DN_RX_ASSERTED()) {
        // Logic 0 received
        sg_u8RxDataByte &= ~(1 << sg_u8RxBitCount);
    } else {
        // Logic 1 received  
        sg_u8RxDataByte |= (1 << sg_u8RxBitCount);
    }
    
    sg_u8RxBitCount++;
    
    if (sg_u8RxBitCount >= 8) {
        // Complete byte received
        processReceivedByte(sg_u8RxDataByte);
        sg_bReceiving = false;
        TIMER_CHA_INT_DISABLE();
    } else {
        // Schedule next bit sample
        TIMER_CHA_INT(VUART_BIT_TICKS);
    }
}
```

## Data Flow and Timing

### Communication Cycle Timing

```c
// Complete communication cycle analysis
Communication Cycle Breakdown:
├── Command Phase (Downstream):
│   ├── Command transmission: 20ms (2 bytes × 10 bits × 50μs)
│   ├── Command propagation: 94 × 0.1ms = 9.4ms relay delay
│   ├── Processing time: 94 × 1ms = 94ms measurement time
│   └── Subtotal: ~123ms
├── Response Phase (Upstream):
│   ├── Data transmission: 188ms (376 bytes × 10 bits × 50μs)
│   ├── Signal propagation: 94 × 0.1ms = 9.4ms relay delay
│   ├── Processing overhead: ~10ms
│   └── Subtotal: ~207ms
├── Total Cycle Time: ~330ms
├── Maximum Update Rate: ~3Hz
└── Typical Update Rate: 1-2Hz (with margin)

// Timing critical sections
void maintainTimingAccuracy(void) {
    // Critical timing requirements:
    // 1. Bit time accuracy: ±1% (±0.5μs on 50μs)
    // 2. Start bit detection: <5μs response time
    // 3. Command relay delay: <100μs between cells
    // 4. Data aggregation: No gaps in transmission stream
    
    // Implementation strategies:
    // - Use timer interrupts for precise bit timing
    // - Minimize interrupt latency with optimized ISRs
    // - Pre-calculate timing values to avoid runtime computation
    // - Use hardware timers for critical timing functions
}
```

### Flow Control and Synchronization

```c
// Flow control mechanisms
void manageDataFlow(void) {
    // No explicit flow control - system relies on:
    // 1. Fixed timing and synchronous operation
    // 2. Chain position detection for role assignment
    // 3. Command completion before data transmission
    // 4. Timeout mechanisms for error recovery
    
    // Synchronization points:
    // - Command reception triggers immediate local processing
    // - Data transmission initiated only by last cell
    // - All cells participate in data aggregation phase
    // - ModuleCPU manages overall communication timing
}

// Collision avoidance
void avoidCommunicationCollisions(void) {
    // System design prevents collisions through:
    // 1. Master-slave protocol (only ModuleCPU initiates)
    // 2. Strict command/response phases
    // 3. Unidirectional data flow in each phase
    // 4. Fixed chain topology with known propagation delays
    
    // No CSMA/CD required due to deterministic protocol
}
```

## Error Handling and Recovery

### Communication Error Detection

```c
// Error detection mechanisms
typedef enum {
    COMM_ERROR_NONE = 0,
    COMM_ERROR_TIMEOUT,          // Communication timeout
    COMM_ERROR_FRAMING,          // Invalid start/stop bits
    COMM_ERROR_OVERRUN,          // Data reception overrun
    COMM_ERROR_CHAIN_BREAK,      // Chain continuity lost
    COMM_ERROR_DATA_CORRUPTION   // Invalid data received
} CommunicationError;

// Timeout detection
#define COMMUNICATION_TIMEOUT_MS    1000    // 1 second timeout
static uint16_t sg_u16CommTimeoutCounter = 0;

void checkCommunicationTimeout(void) {
    sg_u16CommTimeoutCounter++;
    
    if (sg_u16CommTimeoutCounter > COMMUNICATION_TIMEOUT_MS) {
        // Communication timeout detected
        handleCommunicationError(COMM_ERROR_TIMEOUT);
        sg_u16CommTimeoutCounter = 0;
    }
}

void resetCommunicationTimeout(void) {
    // Called when valid communication received
    sg_u16CommTimeoutCounter = 0;
}

// Error handling strategy
void handleCommunicationError(CommunicationError error) {
    switch (error) {
        case COMM_ERROR_TIMEOUT:
            // Disable discharge for safety
            sg_bDischargeActive = false;
            MCP9843SetEventPin(true);  // Ensure discharge off
            
            // Reset communication interfaces
            vUARTInitReceive();
            vUARTInitTransmit();
            break;
            
        case COMM_ERROR_FRAMING:
            // Reset UART state machine
            sg_bReceiving = false;
            sg_u8RxBitCount = 0;
            TIMER_CHA_INT_DISABLE();
            break;
            
        case COMM_ERROR_CHAIN_BREAK:
            // Re-detect chain position
            detectChainPosition();
            break;
            
        default:
            // General recovery - reset communication
            initializeCommunication();
            break;
    }
    
    // Log error for diagnostics
    sg_u8CommunicationErrorCount++;
}
```

### Fault Tolerance

```c
// Graceful degradation on communication failures
void maintainSafeOperation(void) {
    // Safety measures during communication failures:
    
    // 1. Disable cell balancing
    if (sg_u16CommTimeoutCounter > (COMMUNICATION_TIMEOUT_MS / 2)) {
        sg_bDischargeActive = false;
        MCP9843SetEventPin(true);  // Disable discharge
    }
    
    // 2. Continue local monitoring
    // - Voltage and temperature measurements continue
    // - Safety limits still enforced locally
    // - Critical protection functions remain active
    
    // 3. Automatic recovery attempts
    static uint8_t recoveryAttempts = 0;
    if (sg_u16CommTimeoutCounter > COMMUNICATION_TIMEOUT_MS) {
        if (recoveryAttempts < MAX_RECOVERY_ATTEMPTS) {
            initializeCommunication();
            recoveryAttempts++;
        }
        // Reset counter to prevent overflow
        sg_u16CommTimeoutCounter = COMMUNICATION_TIMEOUT_MS / 2;
    }
    
    // 4. Reset recovery counter on successful communication
    if (sg_u16CommTimeoutCounter == 0) {
        recoveryAttempts = 0;
    }
}

// Chain integrity monitoring
void monitorChainIntegrity(void) {
    // Monitor for chain breaks or communication issues
    static uint8_t missedMessages = 0;
    static uint16_t lastMessageTime = 0;
    
    uint16_t currentTime = getSystemTime();
    
    if ((currentTime - lastMessageTime) > EXPECTED_MESSAGE_INTERVAL) {
        missedMessages++;
        
        if (missedMessages > MAX_MISSED_MESSAGES) {
            // Assume chain break or ModuleCPU failure
            handleCommunicationError(COMM_ERROR_CHAIN_BREAK);
            missedMessages = 0;
        }
    } else {
        // Reset missed message counter on successful communication
        missedMessages = 0;
        lastMessageTime = currentTime;
    }
}
```

## Performance Characteristics

### Bandwidth and Throughput

```c
// Communication performance analysis
Performance Metrics:
├── Raw Bit Rate: 20kbps
├── Effective Data Rate: ~2.0 kB/s (including protocol overhead)
├── Protocol Efficiency: 75% (376 data bytes / 500 total bits transmitted)
├── Latency: 330ms for complete 94-cell update
├── Update Rate: 3Hz maximum, 1-2Hz typical
├── Scalability: Linear degradation with cell count
├── Power Efficiency: <1mA per cell during communication
└── Reliability: >99.9% message success rate under normal conditions

// Bandwidth utilization
void optimizeBandwidthUsage(void) {
    // Current protocol optimizations:
    // 1. Minimal message overhead (4 bytes per cell)
    // 2. No redundant data transmission
    // 3. Efficient bit encoding (no escape sequences)
    // 4. Parallel data collection (all cells measure simultaneously)
    
    // Potential improvements:
    // - Data compression for repeated values
    // - Selective updates (only changed cells)
    // - Higher baud rates (limited by chain length and noise)
    // - Protocol optimizations for specific use cases
}

// Scalability considerations
void analyzescalability(void) {
    // Performance vs. chain length:
    // - Linear increase in message length
    // - Linear increase in propagation delay
    // - Quadratic increase in total cycle time
    // - Maximum practical limit: ~100 cells per chain
    
    // Alternative approaches for larger systems:
    // - Multiple shorter chains
    // - Higher-speed communication protocols
    // - Dedicated communication controllers
    // - Hierarchical communication architectures
}
```

The CellCPU communication system provides reliable, scalable communication between the ModuleCPU and up to 94 individual cell monitoring units. The custom protocol balances simplicity, reliability, and performance while maintaining the low cost and power consumption required for battery management applications.