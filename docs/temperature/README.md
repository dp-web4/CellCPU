# CellCPU Temperature Sensing System

## Table of Contents

1. [Temperature System Overview](#temperature-system-overview)
2. [MCP9843 Sensor Integration](#mcp9843-sensor-integration)
3. [I2C Communication Interface](#i2c-communication-interface)
4. [Temperature Data Processing](#temperature-data-processing)
5. [Thermal Management and Control](#thermal-management-and-control)
6. [Calibration and Accuracy](#calibration-and-accuracy)
7. [Error Handling and Diagnostics](#error-handling-and-diagnostics)
8. [Performance Optimization](#performance-optimization)

## Temperature System Overview

The CellCPU temperature sensing system provides precision temperature monitoring for individual lithium-ion cells using the MCP9843 digital temperature sensor. This system enables thermal protection, thermal balancing optimization, and comprehensive battery health monitoring.

### System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                Temperature Sensing Architecture                 │
├─────────────────────────────────────────────────────────────────┤
│ Physical Temperature Interface                                  │
│ ├── MCP9843 Sensor (8-pin DFN package)                        │
│ ├── Thermal Coupling to Cell                                   │
│ ├── I2C Communication Interface                                │
│ └── EVENT Pin for Discharge Control                            │
├─────────────────────────────────────────────────────────────────┤
│ I2C Communication Layer                                         │
│ ├── 2-Wire Interface (SDA, SCL)                               │
│ ├── 100kHz Standard Mode Operation                             │
│ ├── Pin Multiplexing with ADC Function                        │
│ └── Software I2C Implementation                                │
├─────────────────────────────────────────────────────────────────┤
│ Data Processing and Control                                     │
│ ├── 12-bit Temperature Resolution                              │
│ ├── Digital Filtering and Validation                          │
│ ├── Thermal Protection Logic                                   │
│ └── Discharge Control Integration                               │
├─────────────────────────────────────────────────────────────────┤
│ ATtiny45 Integration                                           │
│ ├── PB2: I2C SDA (bidirectional)                             │
│ ├── PB3: I2C SCL (multiplexed with ADC)                      │
│ ├── USI Hardware for I2C Support                              │
│ └── Timer-based I2C Bit Timing                                │
└─────────────────────────────────────────────────────────────────┘
```

### Key Temperature System Features

```
Temperature System Specifications:
├── Sensor: MCP9843 Digital Temperature Sensor
├── Accuracy: ±0.5°C (typical), ±1°C (maximum)
├── Resolution: 0.0625°C (12-bit, 1/16th degree)
├── Range: -40°C to +125°C (industrial grade)
├── Interface: I2C at 100kHz standard mode
├── Response Time: 250ms maximum conversion time
├── Power: 200μA typical operating current
├── Update Rate: Configurable, typically 1-10Hz
├── Thermal Coupling: Direct contact with cell or thermal interface
└── Control Output: EVENT pin for discharge FET control
```

## MCP9843 Sensor Integration

### Hardware Configuration

The MCP9843 is a precision digital temperature sensor specifically chosen for its accuracy, low power consumption, and integrated control features.

```c
// MCP9843 sensor specifications and capabilities
MCP9843 Technical Specifications:
├── Package: 8-pin DFN (2mm × 3mm × 0.9mm)
├── Supply Voltage: 2.7V to 5.5V
├── Supply Current: 200μA typical, 1μA standby
├── Temperature Range: -40°C to +125°C
├── Accuracy: ±0.5°C (-20°C to +100°C), ±1°C (full range)
├── Resolution: User-selectable (9-bit to 12-bit)
├── Conversion Time: 30ms (9-bit) to 250ms (12-bit)
├── I2C Address: 0x30 (with A2=A1=A0=0)
├── Alert Function: Programmable temperature limits
├── EVENT Pin: Open-drain output for external control
└── Features: Non-volatile configuration, EEPROM settings
```

### Physical Integration and Thermal Design

```c
// Thermal interface design considerations
Thermal Integration Strategy:
├── Sensor Placement:
│   ├── Direct contact with cell surface (preferred)
│   ├── Thermal interface material for good coupling
│   ├── Minimize thermal resistance to cell
│   └── Shield from ambient temperature variations
├── Thermal Time Constants:
│   ├── Cell thermal mass: ~10-30 seconds
│   ├── Sensor response: <10 seconds to 90% final value
│   ├── System response: Limited by cell thermal mass
│   └── Update rate: 1Hz adequate for thermal monitoring
├── Environmental Considerations:
│   ├── Ambient temperature compensation
│   ├── Air flow effects minimization
│   ├── Thermal isolation from other components
│   └── Mechanical stability over temperature cycles
└── Accuracy Factors:
    ├── Thermal gradient across cell: ±1-2°C typical
    ├── Sensor self-heating: <0.1°C at 200μA
    ├── PCB thermal effects: <0.5°C with proper design
    └── Calibration: ±0.5°C achievable with system cal
```

### MCP9843 Register Configuration

```c
// MCP9843 register map and configuration
#define MCP9843_REG_CONFIG      0x01    // Configuration register
#define MCP9843_REG_UPPER_TEMP  0x02    // Upper temperature limit
#define MCP9843_REG_LOWER_TEMP  0x03    // Lower temperature limit  
#define MCP9843_REG_CRIT_TEMP   0x04    // Critical temperature limit
#define MCP9843_REG_TEMP        0x05    // Temperature register
#define MCP9843_REG_MANUF_ID    0x06    // Manufacturer ID (0x54)
#define MCP9843_REG_DEVICE_ID   0x07    // Device ID (0x84)
#define MCP9843_REG_RESOLUTION  0x08    // Resolution register

// Configuration register bit definitions
#define MCP9843_CONFIG_EVENT_MODE     0x01    // EVENT pin mode
#define MCP9843_CONFIG_EVENT_POLARITY 0x02    // EVENT pin polarity
#define MCP9843_CONFIG_EVENT_SELECT   0x04    // EVENT pin output select
#define MCP9843_CONFIG_ALERT_ENABLE   0x08    // Alert function enable
#define MCP9843_CONFIG_UPPER_WIN      0x10    // Upper window flag
#define MCP9843_CONFIG_LOWER_WIN      0x20    // Lower window flag
#define MCP9843_CONFIG_CRIT_LOCK      0x40    // Critical temp lock
#define MCP9843_CONFIG_WIN_LOCK       0x80    // Window temp lock

// Initialize MCP9843 for CellCPU application
bool MCP9843Initialize(void) {
    uint16_t deviceID;
    
    // Verify device presence and identity
    if (!I2C_ReadRegister16(MCP9843_ADDRESS, MCP9843_REG_DEVICE_ID, &deviceID)) {
        return false;
    }
    
    if (deviceID != 0x0084) {  // Expected MCP9843 device ID
        return false;
    }
    
    // Configure for maximum resolution (12-bit)
    if (!I2C_WriteRegister(MCP9843_ADDRESS, MCP9843_REG_RESOLUTION, 0x03)) {
        return false;
    }
    
    // Configure basic operation
    uint8_t config = MCP9843_CONFIG_EVENT_MODE |      // EVENT pin enabled
                     MCP9843_CONFIG_EVENT_POLARITY;   // Active low output
    
    if (!I2C_WriteRegister(MCP9843_ADDRESS, MCP9843_REG_CONFIG, config)) {
        return false;
    }
    
    // Set temperature limits for discharge control (optional)
    // Upper limit: 45°C (too hot for discharge)
    // Lower limit: 0°C (too cold for discharge)
    I2C_WriteRegister16(MCP9843_ADDRESS, MCP9843_REG_UPPER_TEMP, encodeTemperature(45.0));
    I2C_WriteRegister16(MCP9843_ADDRESS, MCP9843_REG_LOWER_TEMP, encodeTemperature(0.0));
    
    return true;
}
```

## I2C Communication Interface

### Software I2C Implementation

The CellCPU implements software-based I2C communication using the ATtiny45's USI peripheral and GPIO control.

```c
// I2C interface implementation using USI and GPIO
I2C Configuration:
├── Clock Frequency: 100kHz (standard mode)
├── Address Mode: 7-bit addressing
├── Data Rate: ~10kbps effective (including protocol overhead)
├── SDA Pin: PB2 (dedicated I2C data)
├── SCL Pin: PB3 (multiplexed with ADC3)
├── Pull-ups: External 4.7kΩ resistors to VCC
├── Voltage Levels: I2C specification compliant
└── Bus Arbitration: Not required (single master)

// Pin control macros for I2C operation
#define I2C_SDA_HIGH()    do { DDRB &= ~(1<<PB2); PORTB |= (1<<PB2); } while(0)
#define I2C_SDA_LOW()     do { PORTB &= ~(1<<PB2); DDRB |= (1<<PB2); } while(0)
#define I2C_SCL_HIGH()    do { DDRB &= ~(1<<PB3); PORTB |= (1<<PB3); } while(0)
#define I2C_SCL_LOW()     do { PORTB &= ~(1<<PB3); DDRB |= (1<<PB3); } while(0)
#define I2C_SDA_READ()    (PINB & (1<<PB2))
#define I2C_SCL_READ()    (PINB & (1<<PB3))

// I2C timing constants for 100kHz operation
#define I2C_DELAY_US      5     // 5μs delay for 100kHz clock
#define I2C_TIMEOUT_MS    10    // 10ms timeout for I2C operations

// Basic I2C protocol implementation
bool I2C_Start(void) {
    // Ensure bus is idle
    I2C_SDA_HIGH();
    I2C_SCL_HIGH();
    _delay_us(I2C_DELAY_US);
    
    // Generate start condition: SDA high-to-low while SCL high
    I2C_SDA_LOW();
    _delay_us(I2C_DELAY_US);
    I2C_SCL_LOW();
    _delay_us(I2C_DELAY_US);
    
    return true;
}

bool I2C_Stop(void) {
    // Generate stop condition: SDA low-to-high while SCL high
    I2C_SDA_LOW();
    _delay_us(I2C_DELAY_US);
    I2C_SCL_HIGH();
    _delay_us(I2C_DELAY_US);
    I2C_SDA_HIGH();
    _delay_us(I2C_DELAY_US);
    
    return true;
}

bool I2C_WriteByte(uint8_t data) {
    // Send 8 bits, MSB first
    for (uint8_t i = 0; i < 8; i++) {
        if (data & 0x80) {
            I2C_SDA_HIGH();
        } else {
            I2C_SDA_LOW();
        }
        
        _delay_us(I2C_DELAY_US);
        I2C_SCL_HIGH();
        _delay_us(I2C_DELAY_US);
        I2C_SCL_LOW();
        
        data <<= 1;
    }
    
    // Check for ACK
    I2C_SDA_HIGH();  // Release SDA for slave ACK
    _delay_us(I2C_DELAY_US);
    I2C_SCL_HIGH();
    _delay_us(I2C_DELAY_US);
    
    bool ack = !I2C_SDA_READ();  // ACK is low
    
    I2C_SCL_LOW();
    _delay_us(I2C_DELAY_US);
    
    return ack;
}

uint8_t I2C_ReadByte(bool sendAck) {
    uint8_t data = 0;
    
    I2C_SDA_HIGH();  // Release SDA for slave to drive
    
    // Read 8 bits, MSB first
    for (uint8_t i = 0; i < 8; i++) {
        _delay_us(I2C_DELAY_US);
        I2C_SCL_HIGH();
        _delay_us(I2C_DELAY_US);
        
        data <<= 1;
        if (I2C_SDA_READ()) {
            data |= 1;
        }
        
        I2C_SCL_LOW();
    }
    
    // Send ACK/NACK
    if (sendAck) {
        I2C_SDA_LOW();   // ACK
    } else {
        I2C_SDA_HIGH();  // NACK
    }
    
    _delay_us(I2C_DELAY_US);
    I2C_SCL_HIGH();
    _delay_us(I2C_DELAY_US);
    I2C_SCL_LOW();
    _delay_us(I2C_DELAY_US);
    
    return data;
}
```

### Pin Multiplexing Management

```c
// Pin multiplexing between I2C and ADC functions
void configurePinsForI2C(void) {
    // Configure PB3 as I2C SCL output (open-drain)
    DDRB &= ~(1 << PB3);  // Set as input initially
    PORTB |= (1 << PB3);  // Enable pull-up
    
    // Configure PB2 as I2C SDA output (open-drain)
    DDRB &= ~(1 << PB2);  // Set as input initially
    PORTB |= (1 << PB2);  // Enable pull-up
    
    // Disable ADC to prevent conflict
    ADCSRA &= ~(1 << ADEN);
    
    // Brief settling time
    _delay_us(10);
}

void configurePinForADC(void) {
    // Configure PB3 as ADC input
    DDRB &= ~(1 << PB3);  // Set as input
    PORTB &= ~(1 << PB3); // Disable pull-up
    
    // Re-enable ADC
    ADCSRA |= (1 << ADEN);
    
    // Set ADC multiplexer to ADC3
    ADMUX = (1 << REFS1) |              // Internal 1.1V reference
            (1 << MUX1) | (1 << MUX0);  // ADC3 channel
    
    // Allow ADC to settle
    _delay_us(100);
}

// Measurement coordination with pin switching
void performCoordinatedMeasurement(void) {
    uint16_t voltage, temperature;
    
    // 1. Measure voltage first (ADC mode)
    configurePinForADC();
    voltage = ADCRead();
    
    // 2. Switch to I2C and measure temperature
    configurePinsForI2C();
    temperature = MCP9843ReadTemperature();
    
    // 3. Return to ADC mode for next cycle
    configurePinForADC();
    
    // 4. Process and store results
    processMeasurements(voltage, temperature);
}
```

## Temperature Data Processing

### Temperature Reading and Conversion

```c
// MCP9843 temperature data format and conversion
Temperature Data Format:
├── Register Size: 16-bit (2 bytes)
├── Resolution: 12-bit (configurable 9-12 bit)
├── Format: Two's complement signed integer
├── Scale Factor: 0.0625°C per LSB (1/16th degree)
├── Range: -40°C to +125°C (-640 to +2000 in raw units)
├── Unused Bits: Lower 4 bits (always read as 0)
└── Sign Extension: Required for negative temperatures

// Read temperature with comprehensive error handling
typedef enum {
    TEMP_READ_SUCCESS = 0,
    TEMP_READ_I2C_ERROR,
    TEMP_READ_TIMEOUT,
    TEMP_READ_INVALID_DATA,
    TEMP_READ_RANGE_ERROR
} TemperatureReadResult;

TemperatureReadResult MCP9843ReadTemperatureWithStatus(int16_t* temperature) {
    uint8_t tempBytes[2];
    uint8_t retryCount = 3;
    
    while (retryCount--) {
        // Ensure I2C pins are configured
        configurePinsForI2C();
        
        // Start I2C transaction
        if (!I2C_Start()) {
            continue;  // Retry on start failure
        }
        
        // Write device address
        if (!I2C_WriteByte(MCP9843_ADDRESS << 1)) {
            I2C_Stop();
            continue;
        }
        
        // Write register address
        if (!I2C_WriteByte(MCP9843_REG_TEMP)) {
            I2C_Stop();
            continue;
        }
        
        // Repeated start for read
        if (!I2C_Start()) {
            I2C_Stop();
            continue;
        }
        
        // Write device address with read bit
        if (!I2C_WriteByte((MCP9843_ADDRESS << 1) | 1)) {
            I2C_Stop();
            continue;
        }
        
        // Read temperature data
        tempBytes[0] = I2C_ReadByte(true);   // MSB with ACK
        tempBytes[1] = I2C_ReadByte(false);  // LSB with NACK
        I2C_Stop();
        
        // Convert to 16-bit signed integer
        int16_t rawTemp = (tempBytes[0] << 8) | tempBytes[1];
        
        // Remove unused lower 4 bits
        rawTemp >>= 4;
        
        // Range validation
        if (rawTemp < -640 || rawTemp > 2000) {  // -40°C to +125°C
            continue;  // Invalid range, retry
        }
        
        *temperature = rawTemp;
        return TEMP_READ_SUCCESS;
    }
    
    return TEMP_READ_I2C_ERROR;
}

// Convert raw temperature to various formats
int16_t convertToTenthsDegreeC(int16_t rawTemp) {
    // Convert from 1/16th degree units to 1/10th degree units
    // rawTemp is in 0.0625°C units, result in 0.1°C units
    return (rawTemp * 10) / 16;
}

int16_t convertToDegreeC(int16_t rawTemp) {
    // Convert to whole degrees Celsius
    return (rawTemp + 8) / 16;  // Add 8 for rounding
}

uint16_t convertToEncodedFormat(int16_t rawTemp) {
    // Convert to format suitable for transmission
    // Include sign extension and status bits
    uint16_t encoded = (uint16_t)rawTemp;
    
    // Clear status bits (will be set separately if needed)
    encoded &= 0x7FFF;
    
    return encoded;
}
```

### Digital Filtering and Validation

```c
// Temperature filtering and validation
typedef struct {
    int16_t filterBuffer[TEMP_FILTER_SIZE];    // Circular buffer
    uint8_t filterIndex;                       // Current buffer position
    int16_t lastValidTemp;                     // Last known good reading
    uint8_t consecutiveErrors;                 // Error count
    uint8_t filterInitialized;                 // Filter initialization flag
} TemperatureFilter;

#define TEMP_FILTER_SIZE        4       // Simple 4-sample moving average
#define MAX_TEMP_CHANGE_PER_S   10      // Maximum °C change per second
#define MAX_CONSECUTIVE_ERRORS  5       // Max errors before fault

int16_t processTemperatureReading(int16_t rawReading, TemperatureFilter* filter) {
    int16_t temperature = convertToTenthsDegreeC(rawReading);
    
    // Range validation
    if (temperature < -400 || temperature > 1250) {  // -40.0°C to +125.0°C
        filter->consecutiveErrors++;
        if (filter->consecutiveErrors >= MAX_CONSECUTIVE_ERRORS) {
            return TEMPERATURE_SENSOR_FAULT;
        }
        return filter->lastValidTemp;  // Use last valid reading
    }
    
    // Rate of change validation (if filter initialized)
    if (filter->filterInitialized) {
        int16_t tempChange = abs(temperature - filter->lastValidTemp);
        if (tempChange > MAX_TEMP_CHANGE_PER_S) {
            // Unrealistic temperature change - likely noise
            filter->consecutiveErrors++;
            return filter->lastValidTemp;
        }
    }
    
    // Apply digital filter
    filter->filterBuffer[filter->filterIndex] = temperature;
    filter->filterIndex = (filter->filterIndex + 1) % TEMP_FILTER_SIZE;
    
    // Calculate filtered temperature
    int32_t sum = 0;
    uint8_t validSamples = 0;
    
    for (uint8_t i = 0; i < TEMP_FILTER_SIZE; i++) {
        if (filter->filterBuffer[i] != TEMPERATURE_INVALID) {
            sum += filter->filterBuffer[i];
            validSamples++;
        }
    }
    
    if (validSamples > 0) {
        int16_t filteredTemp = (int16_t)(sum / validSamples);
        filter->lastValidTemp = filteredTemp;
        filter->consecutiveErrors = 0;
        filter->filterInitialized = 1;
        return filteredTemp;
    }
    
    return TEMPERATURE_SENSOR_FAULT;
}

// Initialize temperature filter
void initializeTemperatureFilter(TemperatureFilter* filter) {
    for (uint8_t i = 0; i < TEMP_FILTER_SIZE; i++) {
        filter->filterBuffer[i] = TEMPERATURE_INVALID;
    }
    filter->filterIndex = 0;
    filter->lastValidTemp = 250;  // 25.0°C default
    filter->consecutiveErrors = 0;
    filter->filterInitialized = 0;
}
```

## Thermal Management and Control

### Discharge Control Based on Temperature

```c
// Temperature-based discharge control logic
typedef struct {
    int16_t maxDischargeTemp;      // Maximum safe discharge temperature
    int16_t minDischargeTemp;      // Minimum safe discharge temperature
    int16_t hysteresis;            // Temperature hysteresis
    bool thermalProtectionActive; // Thermal protection state
    uint8_t overTempCount;        // Over-temperature event counter
} ThermalProtection;

void updateThermalProtection(ThermalProtection* protection, int16_t temperature) {
    // Temperature limits for lithium-ion cells
    // Typical limits: 0°C to 45°C for discharge, 0°C to 60°C for charge
    
    bool overTemp = false;
    bool underTemp = false;
    
    // Check over-temperature condition
    if (temperature > protection->maxDischargeTemp) {
        overTemp = true;
        protection->overTempCount++;
    } else if (temperature < (protection->maxDischargeTemp - protection->hysteresis)) {
        // Temperature dropped below threshold minus hysteresis
        overTemp = false;
    }
    
    // Check under-temperature condition
    if (temperature < protection->minDischargeTemp) {
        underTemp = true;
    } else if (temperature > (protection->minDischargeTemp + protection->hysteresis)) {
        underTemp = false;
    }
    
    // Update thermal protection state
    if (overTemp || underTemp) {
        protection->thermalProtectionActive = true;
        
        // Force disable discharge
        sg_bDischargeActive = false;
        MCP9843SetEventPin(true);  // Disable discharge (active low)
        
        // Set status flags
        if (overTemp) {
            sg_u8ThermalStatus |= THERMAL_STATUS_OVER_TEMP;
        }
        if (underTemp) {
            sg_u8ThermalStatus |= THERMAL_STATUS_UNDER_TEMP;
        }
    } else {
        // Clear thermal protection
        protection->thermalProtectionActive = false;
        sg_u8ThermalStatus &= ~(THERMAL_STATUS_OVER_TEMP | THERMAL_STATUS_UNDER_TEMP);
    }
}

// Thermal derating for discharge current
uint8_t calculateThermalDerating(int16_t temperature) {
    // Reduce discharge current as temperature increases
    // Returns percentage of full discharge current (0-100%)
    
    if (temperature < 100) {        // Below 10°C
        return 50;                  // 50% of full current
    } else if (temperature < 250) { // 10°C to 25°C
        return 75;                  // 75% of full current
    } else if (temperature < 400) { // 25°C to 40°C
        return 100;                 // Full current allowed
    } else if (temperature < 450) { // 40°C to 45°C
        return 50;                  // Reduced current
    } else {                        // Above 45°C
        return 0;                   // No discharge allowed
    }
}
```

### EVENT Pin Control for Thermal Management

```c
// MCP9843 EVENT pin control for discharge management
bool MCP9843SetEventPin(bool pinHigh) {
    uint8_t config;
    
    // Read current configuration
    if (!I2C_ReadRegister(MCP9843_ADDRESS, MCP9843_REG_CONFIG, &config)) {
        return false;
    }
    
    // Configure EVENT pin mode and polarity
    config |= MCP9843_CONFIG_EVENT_MODE;      // Enable EVENT pin output
    config |= MCP9843_CONFIG_EVENT_POLARITY;  // Active low polarity
    
    // Set or clear EVENT pin based on desired state
    if (pinHigh) {
        config |= MCP9843_CONFIG_EVENT_SELECT;   // Set EVENT pin high (discharge off)
    } else {
        config &= ~MCP9843_CONFIG_EVENT_SELECT;  // Set EVENT pin low (discharge on)
    }
    
    // Write updated configuration
    return I2C_WriteRegister(MCP9843_ADDRESS, MCP9843_REG_CONFIG, config);
}

// Temperature-based EVENT pin control
void updateEventPinControl(int16_t temperature, bool dischargeRequested) {
    static bool lastEventState = true;  // Track last EVENT pin state
    bool eventPinHigh = true;           // Default: discharge disabled
    
    // Thermal protection logic
    if (temperature > DISCHARGE_THERMAL_LIMIT_HIGH) {
        eventPinHigh = true;   // Disable discharge - too hot
    } else if (temperature < DISCHARGE_THERMAL_LIMIT_LOW) {
        eventPinHigh = true;   // Disable discharge - too cold
    } else if (dischargeRequested && !sg_thermalProtection.thermalProtectionActive) {
        eventPinHigh = false;  // Enable discharge - temperature OK
    } else {
        eventPinHigh = true;   // Disable discharge - not requested or protection active
    }
    
    // Update EVENT pin only if state changed
    if (eventPinHigh != lastEventState) {
        MCP9843SetEventPin(eventPinHigh);
        lastEventState = eventPinHigh;
        
        // Update global discharge state
        sg_bDischargeActive = !eventPinHigh;
    }
}
```

## Calibration and Accuracy

### Temperature Calibration

```c
// Temperature calibration for improved accuracy
typedef struct {
    int16_t offsetCalibration;     // Offset correction in 0.1°C units
    uint16_t gainCalibration;      // Gain correction (16.16 fixed point)
    uint8_t calibrationValid;      // Calibration data validity flag
    uint16_t serialNumber;         // Device serial number
} TemperatureCalibration;

// Apply calibration to raw temperature reading
int16_t applyTemperatureCalibration(int16_t rawTemp, TemperatureCalibration* cal) {
    if (!cal->calibrationValid) {
        return convertToTenthsDegreeC(rawTemp);  // Use uncalibrated conversion
    }
    
    // Convert to tenths of degrees
    int16_t tempTenths = convertToTenthsDegreeC(rawTemp);
    
    // Apply offset correction
    tempTenths += cal->offsetCalibration;
    
    // Apply gain correction (if not unity)
    if (cal->gainCalibration != 0x10000) {  // Not unity gain
        int32_t corrected = (int32_t)tempTenths * cal->gainCalibration;
        tempTenths = (int16_t)(corrected >> 16);
    }
    
    return tempTenths;
}

// Factory calibration process
bool performFactoryTemperatureCalibration(TemperatureCalibration* cal) {
    // Calibration requires known reference temperatures
    // Typically performed at 0°C and 50°C reference points
    
    int16_t ref0C_reading, ref50C_reading;
    int16_t ref0C_actual = 0;     // 0.0°C in 0.1°C units
    int16_t ref50C_actual = 500;  // 50.0°C in 0.1°C units
    
    // Take readings at reference temperatures
    // (This would be done in a calibrated temperature chamber)
    ref0C_reading = convertToTenthsDegreeC(MCP9843ReadTemperature());
    // ... (wait for temperature stabilization)
    ref50C_reading = convertToTenthsDegreeC(MCP9843ReadTemperature());
    
    // Calculate offset correction (based on 0°C reading)
    cal->offsetCalibration = ref0C_actual - ref0C_reading;
    
    // Calculate gain correction (based on span)
    int16_t measuredSpan = ref50C_reading - ref0C_reading;
    int16_t actualSpan = ref50C_actual - ref0C_actual;
    
    if (measuredSpan > 0) {
        cal->gainCalibration = ((int32_t)actualSpan << 16) / measuredSpan;
    } else {
        cal->gainCalibration = 0x10000;  // Unity gain
    }
    
    // Validate calibration data
    if (abs(cal->offsetCalibration) < 50 &&  // Within ±5°C offset
        cal->gainCalibration > 0x8000 &&     // Gain > 0.5
        cal->gainCalibration < 0x18000) {    // Gain < 1.5
        cal->calibrationValid = 1;
        return true;
    }
    
    return false;
}
```

### Accuracy Optimization

```c
// Temperature accuracy optimization techniques
void optimizeTemperatureAccuracy(void) {
    // Multiple measurement averaging
    // - Take multiple readings and average
    // - Reduces random noise effects
    // - Improves effective resolution
    
    // Environmental compensation
    // - Account for PCB self-heating
    // - Compensate for ambient temperature gradients
    // - Consider thermal time constants
    
    // Sensor placement optimization
    // - Minimize thermal resistance to cell
    // - Reduce influence of ambient conditions
    // - Ensure good thermal coupling
    
    // Calibration maintenance
    // - Periodic calibration verification
    // - Temperature-dependent correction factors
    // - Aging compensation (if required)
}

// Multi-sample temperature measurement for improved accuracy
int16_t measureTemperatureHighAccuracy(void) {
    const uint8_t sampleCount = 8;
    int32_t accumulator = 0;
    uint8_t validSamples = 0;
    
    for (uint8_t i = 0; i < sampleCount; i++) {
        int16_t temperature;
        TemperatureReadResult result = MCP9843ReadTemperatureWithStatus(&temperature);
        
        if (result == TEMP_READ_SUCCESS) {
            accumulator += convertToTenthsDegreeC(temperature);
            validSamples++;
        }
        
        _delay_ms(50);  // Allow time between samples
    }
    
    if (validSamples >= (sampleCount / 2)) {
        return (int16_t)(accumulator / validSamples);
    }
    
    return TEMPERATURE_SENSOR_FAULT;
}
```

## Error Handling and Diagnostics

### Temperature Sensor Diagnostics

```c
// Comprehensive temperature sensor diagnostics
typedef struct {
    uint8_t i2cErrorCount;           // I2C communication errors
    uint8_t rangeErrorCount;         // Out-of-range readings
    uint8_t timeoutErrorCount;       // Communication timeouts
    uint8_t consecutiveFailures;     // Consecutive read failures
    uint16_t totalReadAttempts;      // Total read attempts
    uint16_t successfulReads;        // Successful reads
    uint8_t sensorHealth;            // Overall sensor health status
} TemperatureDiagnostics;

// Sensor health assessment
typedef enum {
    SENSOR_HEALTH_GOOD = 0,      // Normal operation
    SENSOR_HEALTH_DEGRADED,      // Reduced performance
    SENSOR_HEALTH_POOR,          // Significant issues
    SENSOR_HEALTH_FAILED         // Sensor failure
} SensorHealthStatus;

SensorHealthStatus assessSensorHealth(TemperatureDiagnostics* diag) {
    // Calculate success rate
    uint16_t successRate = 0;
    if (diag->totalReadAttempts > 0) {
        successRate = (diag->successfulReads * 100) / diag->totalReadAttempts;
    }
    
    // Health assessment based on error rates and success
    if (successRate >= 95 && diag->consecutiveFailures == 0) {
        return SENSOR_HEALTH_GOOD;
    } else if (successRate >= 80 && diag->consecutiveFailures < 3) {
        return SENSOR_HEALTH_DEGRADED;
    } else if (successRate >= 50 && diag->consecutiveFailures < 10) {
        return SENSOR_HEALTH_POOR;
    } else {
        return SENSOR_HEALTH_FAILED;
    }
}

// Self-test functionality
bool performTemperatureSensorSelfTest(void) {
    uint16_t manufacturerID, deviceID;
    
    // Test 1: Read manufacturer ID
    if (!I2C_ReadRegister16(MCP9843_ADDRESS, MCP9843_REG_MANUF_ID, &manufacturerID)) {
        return false;
    }
    
    if (manufacturerID != 0x0054) {  // Expected Microchip manufacturer ID
        return false;
    }
    
    // Test 2: Read device ID  
    if (!I2C_ReadRegister16(MCP9843_ADDRESS, MCP9843_REG_DEVICE_ID, &deviceID)) {
        return false;
    }
    
    if (deviceID != 0x0084) {  // Expected MCP9843 device ID
        return false;
    }
    
    // Test 3: Read temperature and verify reasonable range
    int16_t temperature;
    TemperatureReadResult result = MCP9843ReadTemperatureWithStatus(&temperature);
    
    if (result != TEMP_READ_SUCCESS) {
        return false;
    }
    
    int16_t tempC = convertToTenthsDegreeC(temperature);
    if (tempC < -400 || tempC > 800) {  // -40°C to +80°C reasonable range
        return false;
    }
    
    // Test 4: Verify EVENT pin control
    bool originalState = sg_bDischargeActive;
    
    if (!MCP9843SetEventPin(true)) {   // Test high
        return false;
    }
    
    if (!MCP9843SetEventPin(false)) {  // Test low
        return false;
    }
    
    MCP9843SetEventPin(!originalState);  // Restore original state
    
    return true;  // All tests passed
}
```

## Performance Optimization

### Communication Optimization

```c
// I2C communication performance optimization
void optimizeI2CPerformance(void) {
    // Timing optimization
    // - Use minimum required delays for target clock speed
    // - Optimize delay loops for processor speed
    // - Consider using hardware timers for precise timing
    
    // Error recovery optimization
    // - Implement fast error detection
    // - Use timeout mechanisms to prevent hangs
    // - Graceful degradation on communication errors
    
    // Pin switching optimization
    // - Minimize time spent switching between I2C and ADC modes
    // - Cache pin configurations to reduce setup time
    // - Coordinate measurements to minimize mode switches
}

// Optimized measurement sequence
void performOptimizedMeasurementCycle(void) {
    static uint8_t measurementCounter = 0;
    
    // Optimize measurement frequency based on thermal dynamics
    // Temperature changes slowly compared to voltage
    
    measurementCounter++;
    
    // Always measure voltage (fast ADC reading)
    sg_u16BatteryVoltage = ADCRead();
    
    // Measure temperature less frequently (slow I2C operation)
    if ((measurementCounter % 4) == 0) {  // Every 4th cycle
        configurePinsForI2C();
        sg_u16BatteryTemperature = MCP9843ReadTemperature();
        configurePinForADC();
    }
    
    // Update discharge control based on latest data
    updateThermalProtection(&sg_thermalProtection, 
                           convertToTenthsDegreeC(sg_u16BatteryTemperature));
}

// Power optimization for temperature sensing
void optimizeTemperaturePowerConsumption(void) {
    // Sensor power management
    // - Use sensor's low power modes when possible
    // - Reduce measurement frequency during idle periods
    // - Turn off sensor during extended sleep periods
    
    // I2C bus power optimization
    // - Use minimal pull-up current
    // - Disable I2C pins when not in use
    // - Coordinate with other I2C activities
}
```

The CellCPU temperature sensing system provides accurate, reliable temperature monitoring essential for safe lithium-ion battery operation. The integration of the MCP9843 sensor with comprehensive software support enables precise thermal protection and optimal battery performance across the full operating temperature range.