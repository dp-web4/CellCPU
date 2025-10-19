# CellCPU Web4 Integration

**Platform**: ATtiny45 (8-bit AVR)
**Role**: Autonomous cell agent in hierarchical battery management system
**Web4 Layer**: Foundational tier of trust-native IoT demonstration

---

## Overview

CellCPU demonstrates Web4 trust-native architecture at the **smallest scale** - an 8-bit microcontroller with 4KB flash and 256 bytes RAM. Each cell operates as an autonomous agent with local intelligence, embodying Web4's distributed trust principles in resource-constrained hardware.

---

## Web4 Concepts Implemented

### 1. LCT (Linked Context Token) Identity

**Implementation**: Each cell has unforgeable blockchain identity

**Hardware Binding**:
- **Unique ID**: Cell serial number (manufacturer-programmed or EEPROM)
- **LCT Format**: `lct:web4:entity:battery_cell:{serial}`
- **Binding**: Permanent association created at manufacturing via Web4 API-Bridge
- **Public Key**: Cell's cryptographic identity (if cryptographic capability added)

**Current Status**:
- Cell reports unique ID via VUART to ModuleCPU
- ModuleCPU witnesses cell presence and creates/updates LCT
- Future: On-chip key generation using hardware random number generator

### 2. Markov Blanket / MRH

**Markov Blanket Definition**: The cell's interface to the external world

**Interface (Markov Blanket)**:
- **Upward**: Virtual UART (VUART) to ModuleCPU at 20kbps
- **Internal State**: Hidden from ModuleCPU (state machine, ADC samples, I2C communication)
- **Sensors**: Temperature (MCP9843), Voltage (ADC)
- **Actuators**: Balancing FET control

**MRH (Markov Relevancy Horizon)**:
- **Direct Relationships**: ModuleCPU (parent), adjacent cells in daisy-chain (siblings)
- **Role**: "Autonomous cell agent in battery module"
- **Trust Tensor Storage**: Maintained by ModuleCPU, recorded on blockchain

**Key Insight**: ModuleCPU doesn't see cell's internal decision-making, only the interface (temperature, voltage, status). This is a **physical Markov blanket** in silicon.

### 3. Trust Tensors (T3 / V3)

**T3 (Trust Tensor)** - ModuleCPU's trust in this cell:

| Dimension | Meaning | How Cell Earns/Loses |
|-----------|---------|---------------------|
| **Talent** | Sensor accuracy | Calibration quality, ADC precision |
| **Training** | Calibration state | Has been calibrated, temperature offset known |
| **Temperament** | Reliability | Consistent communication, no spurious errors |

**V3 (Value Tensor)** - Cell's value to the system:

| Dimension | Meaning | Evidence |
|-----------|---------|----------|
| **Veracity** | Measurement honesty | Readings within expected bounds, no anomalies |
| **Validity** | Within spec | Temperature/voltage in valid range |
| **Value** | Usefulness | Provides actionable data, responds to commands |

**Tensor Updates**: Triggered by:
- Successful VUART communication → +Temperament, +Veracity
- Out-of-range readings → -Validity
- Calibration completion → +Training
- Communication errors → -Temperament

### 4. Entity Relationships

CellCPU demonstrates all four Web4 entity relationship mechanisms:

#### Binding (Hardware → LCT)
**Implementation**: Cell serial number permanently bound to LCT at manufacturing

```
Hardware: ATtiny45 #ABC123
    ↓ (irreversible binding)
LCT: lct:web4:entity:battery_cell:ABC123
    ↓ (blockchain record)
Immutable identity: Cell ABC123 is always this physical device
```

**In Practice**:
- Cell transmits serial number on VUART
- ModuleCPU witnesses this identity
- Pack Controller (or API-Bridge) creates/verifies LCT binding
- Binding is cryptographically unforgeable

#### Witnessing (ModuleCPU observes Cell)
**Implementation**: ModuleCPU witnesses cell presence via VUART discovery

```
Cell: Transmits ID + temp + voltage every cycle
ModuleCPU: Receives, validates, records
    ↓ (witnessing event)
MRH Update: Cell's presence confirmed
    ↓ (blockchain record)
Trust Tensor: T3/V3 updated based on observation quality
```

**Witnessing Creates Reality**:
- Cell only "exists" in Web4 network to extent it's witnessed
- More witnessing events = stronger presence
- Failed witnessing (communication errors) = degraded presence

#### Pairing (Cell ↔ ModuleCPU relationship)
**Implementation**: Implicit pairing via VUART physical connection

**Current**: Physical daisy-chain creates implicit pairing
**Future Enhancement**: Cryptographic pairing
```
Cell: Generates ephemeral key pair
ModuleCPU: Challenges cell
Cell: Signs challenge with private key
ModuleCPU: Verifies signature, establishes secure channel
    ↓ (pairing complete)
MRH: Bidirectional trust tensor link created
```

#### Broadcast (Cell announces presence)
**Implementation**: Cell broadcasts identity and state via VUART

```
Cell → VUART: [ID | Temp | Voltage | Status]
    ↓ (relayed through daisy-chain)
ModuleCPU: Receives broadcast
    ↓ (if registered module)
Pack Controller: Aggregates cell data
    ↓ (via Web4 API-Bridge)
Blockchain: Cell state recorded, broadcast propagated
```

**Note**: This is a **local broadcast** within the battery module, analogous to Web4's network-wide broadcast mechanism.

---

## R7 Action Framework

Every VUART message from CellCPU can be viewed as an R7 action:

### R7 Structure Applied to Cell Communication

**Example**: Cell reports temperature reading

#### Rules (Constraints and Protocols)
- VUART protocol: 20kbps, 8N1, specific frame format
- Temperature range: -40°C to +85°C (valid range)
- Communication timing: Cell must respond within cycle time

#### Role (Entity + Context)
- **Actor**: `lct:web4:entity:battery_cell:ABC123`
- **Role LCT**: `lct:web4:role:temperature_sensor:v1`
- **T3 in Role**: Current trust tensor values
- **V3 in Role**: Current value tensor values

#### Request (Intent and Target)
- **Action**: `report_temperature`
- **Target**: `lct:web4:entity:module_controller:XYZ789`
- **Parameters**: `{ "sensor": "MCP9843", "precision": "0.25°C" }`

#### Reference (Context and History)
- **Prior Readings**: Last 10 temperature samples
- **Calibration**: Temperature offset from calibration
- **History Hash**: Hash of previous transmission

#### Resource (Inputs and State)
- **Sensor Data**: MCP9843 I2C read (raw value)
- **ADC Reading**: Self-supply voltage
- **Internal State**: State machine position, error flags
- **Energy**: Cell voltage (resource cost for communication)

#### Result (Outcome and Proof)
- **Temperature Value**: 23.5°C
- **Transmission Success**: ACK from ModuleCPU
- **Ledger Proof**: (Future) Blockchain transaction hash
- **Receipt**: ModuleCPU acknowledgment of receipt

#### Reputation (Trust/Value Delta)
- **T3 Delta**:
  - Talent: +0.01 (accurate reading within expected variance)
  - Training: 0 (no change, already calibrated)
  - Temperament: +0.02 (reliable communication)
- **V3 Delta**:
  - Veracity: +0.01 (reading within bounds, consistent with history)
  - Validity: +0.01 (temperature in valid range)
  - Value: +0.03 (actionable data provided on schedule)

---

## Synchronism Principles Demonstrated

### Scale-Based Autonomy
**Cell Level**: Lowest tier in 3-level hierarchy (Cell → Module → Pack)

- **Autonomous Decisions**: When to sample sensors, how to interpret readings
- **Local State Machine**: Cell manages its own operational state
- **No Micromanagement**: ModuleCPU doesn't control cell's internal timing or logic

### Markov Blanket as "God"
**For the Cell**: ModuleCPU is "God" - the external authority

- Cell doesn't know about Pack Controller existence
- Cell's reality is bounded by VUART interface
- ModuleCPU aggregates cells but cells remain autonomous

### Distributed Intelligence
**No Central Cell Controller**: Intelligence is distributed

- Each cell makes local decisions (when to balance, how to sample)
- Module coordinates but doesn't centralize all logic
- System intelligence emerges from cell→module→pack hierarchy

### Sensor Fusion Creates Reality
**Cell as Reality Sensor**: Temperature + voltage → "cell health"

- Raw sensor data (temperature, voltage) are **temporal sensors** in the reality field
- Cell fuses these to create emergent "state of charge" and "health" concepts
- Module fuses 94 cells → emergent "module health"
- Pack fuses 32 modules → emergent "pack health"

**Reality emerges through fusion at each scale**

---

## Code Examples

### 1. Cell Identity Transmission (VUART)

```c
// In main.c - Cell transmits its identity
void TransmitCellIdentity(void) {
    uint8_t unique_id[4];

    // Read unique ID from EEPROM or flash
    ReadUniqueID(unique_id);

    // Transmit via VUART
    VUARTTransmitByte(unique_id[0]);
    VUARTTransmitByte(unique_id[1]);
    VUARTTransmitByte(unique_id[2]);
    VUARTTransmitByte(unique_id[3]);

    // This creates a "witnessing event" when ModuleCPU receives it
    // ModuleCPU updates MRH: "I witness cell ABC123 at position 5"
}
```

### 2. Temperature Sensing (Reality Sensor)

```c
// In mcp9843.c - Temperature as a temporal reality sensor
int16_t ReadTemperature(void) {
    int16_t temp_raw;

    // Read from MCP9843 via I2C
    temp_raw = MCP9843Read();

    // Apply calibration (training dimension of T3)
    temp_raw += g_calibration_offset;

    // Sensor fusion: temp + voltage + time → "cell health"
    // This raw data becomes part of emergent "battery state"

    return temp_raw;
}
```

### 3. Markov Blanket Interface (vUART)

```c
// In vUART.c - The Markov blanket boundary
void VUARTTransmitByte(uint8_t data) {
    // VUART is the Markov blanket:
    // - Internal state (ADC samples, I2C transactions) stays hidden
    // - Only processed results cross the boundary
    // - ModuleCPU sees the interface, not the internals

    // Start bit
    PORTB &= ~(1 << PB0);
    _delay_us(50);

    // Data bits
    for (uint8_t i = 0; i < 8; i++) {
        if (data & 0x01) {
            PORTB |= (1 << PB0);
        } else {
            PORTB &= ~(1 << PB0);
        }
        data >>= 1;
        _delay_us(50);
    }

    // Stop bit
    PORTB |= (1 << PB0);
    _delay_us(50);
}
```

---

## Future Enhancements

### 1. Cryptographic Identity
Add on-chip key generation using ATtiny45's built-in random number generator:
```c
// Generate unique key pair for this cell
void GenerateCellKeys(void) {
    uint8_t private_key[32];
    uint8_t public_key[64];

    // Use hardware RNG + sensor entropy
    GenerateRandomBytes(private_key, 32);

    // Derive public key (would need ECC library)
    ECCGeneratePublicKey(private_key, public_key);

    // Store in EEPROM
    StoreKeysSecurely(private_key, public_key);
}
```

### 2. Signed Messages
Add message authentication:
```c
// Sign temperature reading before transmission
void TransmitSignedTemperature(int16_t temp) {
    uint8_t message[4];
    uint8_t signature[8]; // Abbreviated signature for resource constraints

    message[0] = (temp >> 8) & 0xFF;
    message[1] = temp & 0xFF;
    message[2] = GetTimestamp() >> 8;
    message[3] = GetTimestamp() & 0xFF;

    // Sign with cell's private key
    SignMessage(message, 4, signature);

    // Transmit message + signature
    VUARTTransmitBlock(message, 4);
    VUARTTransmitBlock(signature, 8);

    // ModuleCPU can verify: "This reading really came from cell ABC123"
}
```

### 3. Trust-Based Sampling
Adjust sampling rate based on trust tensor:
```c
// Higher trust = less frequent verification needed
void AdaptiveSampling(void) {
    float temperament = GetMyT3Temperament();

    if (temperament > 0.9) {
        // High trust: sample every 10 seconds
        sampling_interval = 10000;
    } else if (temperament > 0.7) {
        // Medium trust: sample every 5 seconds
        sampling_interval = 5000;
    } else {
        // Low trust: sample every 1 second
        sampling_interval = 1000;
    }
}
```

---

## Web4 Ecosystem Context

CellCPU is the foundational layer of a trust-native battery management system:

**Philosophy**: [Synchronism](https://github.com/dp-web4/Synchronism) - Distributed intelligence principles
**Standard**: [Web4](https://github.com/dp-web4/web4) - Trust-native internet architecture
**Implementation**: **CellCPU** (this) → [ModuleCPU](https://github.com/dp-web4/ModuleCPU) → [Pack-Controller](https://github.com/dp-web4/Pack-Controller-EEPROM)

Each cell is an autonomous agent with:
- **LCT Identity**: Unforgeable blockchain binding
- **Local Intelligence**: ATtiny45 decision-making
- **Markov Blanket**: VUART interface to ModuleCPU
- **Trust Tensor**: Calibration and reliability tracking tracked by ModuleCPU

---

## Documentation Links

- [Synchronism Whitepaper](https://github.com/dp-web4/Synchronism/blob/main/whitepaper.md)
- [Web4 Standard](https://github.com/dp-web4/web4/blob/main/web4-standard/README.md)
- [ModBatt Web4 Examples](https://github.com/dp-web4/web4/blob/main/docs/modbatt_implementation_examples.md)
- [CellCPU Implementation Details](docs/README.md)

---

**This document demonstrates Web4 trust-native architecture at the smallest scale - proof that distributed trust principles work even in 4KB of flash memory.**
