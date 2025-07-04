# CellCPU Cell Interface and Monitoring

## Table of Contents

1. [Interface Overview](#interface-overview)
2. [Voltage Monitoring System](#voltage-monitoring-system)
3. [Temperature Monitoring System](#temperature-monitoring-system)
4. [Cell Balancing Control](#cell-balancing-control)
5. [Data Acquisition Pipeline](#data-acquisition-pipeline)
6. [Safety and Protection](#safety-and-protection)
7. [Calibration and Accuracy](#calibration-and-accuracy)
8. [Diagnostic Features](#diagnostic-features)

## Interface Overview

The CellCPU provides comprehensive monitoring and control for individual lithium-ion cells through multiple sensor interfaces and control outputs. Each CellCPU directly interfaces with one cell, providing real-time voltage monitoring, temperature sensing, and active balancing control.

### Cell Interface Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Cell Interface System                       │
├─────────────────────────────────────────────────────────────────┤
│ Physical Cell Interface                                         │
│ ├── Cell Voltage Input (0V to 5V range)                       │
│ ├── Temperature Sensor Placement                               │
│ ├── Discharge Control Output                                   │
│ └── Ground Reference Connection                                 │
├─────────────────────────────────────────────────────────────────┤
│ Signal Conditioning                                             │
│ ├── Voltage Divider Network (4:1 ratio)                       │
│ ├── Input Protection (TVS diodes, RC filtering)               │
│ ├── Temperature Sensor I2C Interface                          │
│ └── FET Driver for Discharge Control                          │
├─────────────────────────────────────────────────────────────────┤
│ ATtiny45 Interface                                             │
│ ├── ADC3 (PB3): Scaled voltage measurement                    │
│ ├── I2C (PB2/PB3): Temperature sensor communication           │
│ ├── MCP9843 EVENT pin: Discharge FET control                  │
│ └── Reference voltage: Internal 1.1V                          │
└─────────────────────────────────────────────────────────────────┘
```

### Key Specifications

```
Cell Interface Specifications:
├── Voltage Range: 2.0V to 5.0V (covers Li-ion operating range)
├── Voltage Resolution: ~4.3mV per ADC count
├── Voltage Accuracy: ±1% after calibration
├── Temperature Range: -40°C to +125°C
├── Temperature Resolution: 0.0625°C (12-bit)
├── Temperature Accuracy: ±0.5°C typical
├── Update Rate: On-demand (typically 1-10Hz)
├── Response Time: <5ms for complete measurement cycle
└── Power Consumption: <1mA continuous operation
```

## Voltage Monitoring System

### Hardware Implementation

The voltage monitoring system uses a precision voltage divider network to scale cell voltage for the ATtiny45's ADC input range.

```c
// Voltage measurement circuit design
Voltage Divider Network:
├── R1 (High side): 30kΩ ±1% precision resistor
├── R2 (Low side): 10kΩ ±1% precision resistor  
├── Divider ratio: 10k/(30k+10k) = 0.25 (4:1 scaling)
├── Input impedance: 40kΩ (minimal cell loading)
├── Input protection: 5.1V TVS diode
├── Anti-alias filter: 100Ω + 1nF RC filter
└── ADC reference: Internal 1.1V (±10% accuracy)

// Voltage scaling calculation:
// Cell voltage 4.2V → Divider output 1.05V → ADC reading ~975 counts
// ADC resolution: 1.1V / 1023 counts = 1.075mV per count
// Cell resolution: 1.075mV / 0.25 = 4.3mV per count
```

### ADC Configuration and Operation

```c
// ADC setup for optimal accuracy
void configureVoltageADC(void) {
    // Configure ADC for voltage measurement
    ADMUX = (1 << REFS1) |              // Internal 1.1V reference
            (1 << MUX1) | (1 << MUX0);  // ADC3 channel (PB3)
    
    // ADC prescaler for optimal accuracy vs. speed
    ADCSRA = (1 << ADEN) |              // Enable ADC
             (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // /128 prescaler
    
    // ADC clock = 8MHz / 128 = 62.5kHz (optimal for accuracy)
    // Conversion time = 13 cycles = 208μs
}

// Voltage measurement with multiple samples for accuracy
uint16_t measureCellVoltage(void) {
    uint32_t accumulator = 0;
    uint8_t samples = 8;  // Oversampling for noise reduction
    
    // Configure pin for ADC (switch from I2C mode)
    SCL_SET_INPUT();     // Disable SCL output
    SCL_LOW();           // Disable pull-up
    
    // Allow ADC input to settle
    _delay_us(100);
    
    // Take multiple samples
    for (uint8_t i = 0; i < samples; i++) {
        ADCSRA |= (1 << ADSC);              // Start conversion
        while (ADCSRA & (1 << ADSC));       // Wait for completion
        
        uint16_t reading = ADCL;
        reading |= (ADCH << 8);
        accumulator += reading;
        
        _delay_us(50);  // Small delay between samples
    }
    
    // Return averaged result
    return (uint16_t)(accumulator / samples);
}

// Convert ADC reading to millivolts
uint16_t adcToMillivolts(uint16_t adcValue) {
    // Using integer math for efficiency on 8-bit processor
    // Formula: mV = (ADC_value × 1100mV × 4) / 1023
    uint32_t millivolts = (uint32_t)adcValue * 1100UL * 4UL;
    millivolts /= 1023UL;
    return (uint16_t)millivolts;
}
```

### Voltage Measurement Accuracy

```c
// Calibration and accuracy considerations
Voltage Accuracy Factors:
├── ADC Reference Stability: ±10% (1.1V internal reference)
├── Voltage Divider Precision: ±1% (precision resistors)
├── Temperature Coefficient: ~50ppm/°C (resistor drift)
├── ADC Linearity Error: ±1 LSB (INL specification)
├── Quantization Error: ±0.5 LSB (inherent ADC limitation)
├── Noise and Filtering: <1 LSB RMS (with proper filtering)
└── Total System Error: <±2% worst case, ±1% typical

// Calibration approach for production units
typedef struct {
    uint16_t offsetCorrection;    // Zero offset correction
    uint16_t gainCorrection;      // Full-scale gain correction
    uint8_t  validationFlag;      // Calibration data validity
} VoltageCalibration;

// Apply calibration to raw ADC reading
uint16_t applyCalibratedVoltage(uint16_t rawADC, VoltageCalibration* cal) {
    // Apply offset correction
    int32_t corrected = (int32_t)rawADC - cal->offsetCorrection;
    
    // Apply gain correction (using 16.16 fixed point)
    corrected = (corrected * (int32_t)cal->gainCorrection) >> 16;
    
    // Clamp to valid range
    if (corrected < 0) corrected = 0;
    if (corrected > 1023) corrected = 1023;
    
    return (uint16_t)corrected;
}
```

## Temperature Monitoring System

### MCP9843 Sensor Integration

The temperature monitoring system uses the MCP9843 precision temperature sensor connected via I2C interface.

```c
// MCP9843 sensor specifications and interface
MCP9843 Temperature Sensor:
├── Interface: I2C at 100kHz (standard mode)
├── I2C Address: 0x30 (A2=A1=A0=0, factory default)
├── Resolution: 12-bit (0.0625°C per LSB)
├── Accuracy: ±0.5°C (typical), ±1°C (maximum)
├── Conversion Time: 250ms maximum (default configuration)
├── Temperature Range: -40°C to +125°C
├── Supply Current: 200μA typical during conversion
├── Alert/Event Pin: Programmable output for discharge control
└── Configuration: User-programmable limits and hysteresis
```

### I2C Communication Implementation

```c
// I2C interface for temperature sensor
#define MCP9843_ADDRESS         0x30
#define MCP9843_REG_CONFIG      0x01
#define MCP9843_REG_UPPER_TEMP  0x02
#define MCP9843_REG_LOWER_TEMP  0x03
#define MCP9843_REG_CRIT_TEMP   0x04
#define MCP9843_REG_TEMP        0x05
#define MCP9843_REG_MANUF_ID    0x06
#define MCP9843_REG_DEVICE_ID   0x07

// Temperature reading with comprehensive error handling
typedef enum {
    TEMP_OK = 0,
    TEMP_I2C_START_ERROR,
    TEMP_I2C_ADDRESS_NACK,
    TEMP_I2C_REGISTER_NACK,
    TEMP_I2C_REPEATED_START_ERROR,
    TEMP_I2C_READ_ERROR,
    TEMP_VALUE_OUT_OF_RANGE
} TemperatureError;

TemperatureError readTemperatureRaw(uint16_t* temperature) {
    uint8_t tempBytes[2];
    
    // Start I2C transaction
    if (!I2C_Start()) {
        return TEMP_I2C_START_ERROR;
    }
    
    // Send device address for write
    if (!I2C_WriteByte(MCP9843_ADDRESS << 1)) {
        I2C_Stop();
        return TEMP_I2C_ADDRESS_NACK;
    }
    
    // Send register address (temperature register)
    if (!I2C_WriteByte(MCP9843_REG_TEMP)) {
        I2C_Stop();
        return TEMP_I2C_REGISTER_NACK;
    }
    
    // Repeated start for read operation
    if (!I2C_RepeatedStart()) {
        I2C_Stop();
        return TEMP_I2C_REPEATED_START_ERROR;
    }
    
    // Send device address for read
    if (!I2C_WriteByte((MCP9843_ADDRESS << 1) | 1)) {
        I2C_Stop();
        return TEMP_I2C_READ_ERROR;
    }
    
    // Read temperature data (MSB first)
    tempBytes[0] = I2C_ReadByte(true);   // MSB with ACK
    tempBytes[1] = I2C_ReadByte(false);  // LSB with NACK
    I2C_Stop();
    
    // Combine bytes and extract temperature
    uint16_t rawTemp = (tempBytes[0] << 8) | tempBytes[1];
    rawTemp >>= 4;  // Remove 4 unused LSBs
    
    // Range check (basic sanity check)
    if (rawTemp > 4095) {  // Beyond 12-bit range
        return TEMP_VALUE_OUT_OF_RANGE;
    }
    
    *temperature = rawTemp;
    return TEMP_OK;
}

// High-level temperature reading with retry logic
uint16_t MCP9843ReadTemperature(void) {
    uint16_t temperature;
    TemperatureError error;
    uint8_t retries = 3;
    
    // Configure pins for I2C operation
    configurePinsForI2C();
    
    // Attempt reading with retries
    while (retries--) {
        error = readTemperatureRaw(&temperature);
        if (error == TEMP_OK) {
            return temperature;  // Success
        }
        
        // Brief delay before retry
        _delay_ms(10);
        
        // Reset I2C bus on error
        I2C_Reset();
    }
    
    // All retries failed - return error indication
    return TEMPERATURE_ERROR_VALUE | TEMP_I2C_ERROR_FLAG;
}
```

### Temperature Data Processing

```c
// Temperature conversion and processing
int16_t convertRawToTemperature(uint16_t rawValue) {
    // MCP9843 uses 12-bit two's complement format
    // Resolution: 0.0625°C per LSB
    // Range: -40°C to +125°C
    
    int16_t temperature = (int16_t)rawValue;
    
    // Handle negative temperatures (two's complement)
    if (temperature & 0x0800) {  // Sign bit set
        temperature |= 0xF000;   // Sign extend to 16 bits
    }
    
    // Convert to actual temperature (keep in 1/16th degree units)
    // temperature = temperature * 0.0625 = temperature / 16
    // Return in 0.1°C units for integer processing
    return (temperature * 10) / 16;
}

// Temperature processing with filtering
typedef struct {
    int16_t filterBuffer[4];     // Simple moving average filter
    uint8_t bufferIndex;
    int16_t lastValidTemp;       // Last known good temperature
    uint8_t errorCount;          // Consecutive error count
} TemperatureFilter;

int16_t processTemperatureReading(uint16_t rawReading, TemperatureFilter* filter) {
    // Check for I2C error flag
    if (rawReading & TEMP_I2C_ERROR_FLAG) {
        filter->errorCount++;
        
        // Use last valid temperature if recent errors
        if (filter->errorCount < 5) {
            return filter->lastValidTemp;
        } else {
            // Return error indication after persistent failures
            return TEMPERATURE_SENSOR_FAILED;
        }
    }
    
    // Convert raw reading to temperature
    int16_t temperature = convertRawToTemperature(rawReading & 0x7FFF);
    
    // Range validation
    if (temperature < -400 || temperature > 1250) {  // -40.0°C to +125.0°C
        filter->errorCount++;
        return filter->lastValidTemp;
    }
    
    // Apply simple moving average filter
    filter->filterBuffer[filter->bufferIndex] = temperature;
    filter->bufferIndex = (filter->bufferIndex + 1) % 4;
    
    int32_t sum = 0;
    for (uint8_t i = 0; i < 4; i++) {
        sum += filter->filterBuffer[i];
    }
    
    temperature = (int16_t)(sum / 4);
    filter->lastValidTemp = temperature;
    filter->errorCount = 0;
    
    return temperature;
}
```

## Cell Balancing Control

### Discharge Control System

The CellCPU controls cell balancing through the MCP9843's EVENT pin, which drives an external FET for active discharge balancing.

```c
// Discharge control implementation
Cell Balancing System:
├── Control Method: Active discharge via external FET
├── Control Signal: MCP9843 EVENT pin (open-drain output)
├── FET Type: N-channel MOSFET (external to CellCPU)
├── Load Resistor: External precision resistor (application-specific)
├── Discharge Current: Typically 100mA to 1A (depends on external components)
├── Control Logic: Voltage threshold-based with hysteresis
├── Safety: Current limiting via external resistor sizing
└── Thermal: Heat dissipation managed externally
```

### Discharge Control Logic

```c
// Discharge control state machine
typedef enum {
    DISCHARGE_DISABLED,      // No discharge activity
    DISCHARGE_ENABLED,       // Active discharge in progress  
    DISCHARGE_ERROR,         // Error state (sensor failure, etc.)
    DISCHARGE_INHIBITED      // Temporarily disabled (safety)
} DischargeState;

typedef struct {
    uint16_t targetVoltage;      // Desired cell voltage (ADC units)
    uint16_t hysteresisVolts;    // Hysteresis to prevent oscillation
    DischargeState state;        // Current discharge state
    uint16_t enableCounter;      // Time discharge has been enabled
    uint16_t disableCounter;     // Time since discharge disabled
    bool forceDisable;           // Emergency disable flag
} DischargeControl;

// Update discharge state based on voltage and target
void updateDischargeControl(DischargeControl* ctrl, uint16_t currentVoltage) {
    bool shouldDischarge = false;
    
    // Safety check - disable on any error conditions
    if (ctrl->forceDisable || 
        (sg_u16BatteryTemperature & TEMP_I2C_ERROR_FLAG) ||
        currentVoltage == 0) {
        ctrl->state = DISCHARGE_INHIBITED;
        MCP9843SetEventPin(true);  // Disable discharge (active low)
        return;
    }
    
    // Determine if discharge should be active
    switch (ctrl->state) {
        case DISCHARGE_DISABLED:
            // Enable discharge if voltage significantly above target
            if (currentVoltage > (ctrl->targetVoltage + ctrl->hysteresisVolts)) {
                shouldDischarge = true;
                ctrl->state = DISCHARGE_ENABLED;
                ctrl->enableCounter = 0;
            }
            break;
            
        case DISCHARGE_ENABLED:
            // Continue discharge until voltage drops below target
            if (currentVoltage <= ctrl->targetVoltage) {
                shouldDischarge = false;
                ctrl->state = DISCHARGE_DISABLED;
                ctrl->disableCounter = 0;
            } else {
                shouldDischarge = true;
                ctrl->enableCounter++;
                
                // Safety timeout - disable after extended operation
                if (ctrl->enableCounter > DISCHARGE_TIMEOUT_CYCLES) {
                    ctrl->state = DISCHARGE_ERROR;
                    shouldDischarge = false;
                }
            }
            break;
            
        case DISCHARGE_ERROR:
            // Remain in error state until manual reset
            shouldDischarge = false;
            break;
            
        case DISCHARGE_INHIBITED:
            // Check if inhibit condition has cleared
            if (!ctrl->forceDisable) {
                ctrl->state = DISCHARGE_DISABLED;
            }
            shouldDischarge = false;
            break;
    }
    
    // Update hardware control
    MCP9843SetEventPin(!shouldDischarge);  // EVENT pin active low for discharge
    
    // Update global discharge status for reporting
    sg_bDischargeActive = shouldDischarge;
    
    // Set status bit in voltage data
    if (shouldDischarge) {
        sg_u16BatteryVoltage |= MSG_CELL_DISCHARGE_ACTIVE;
    } else {
        sg_u16BatteryVoltage &= ~MSG_CELL_DISCHARGE_ACTIVE;
    }
}
```

### Discharge Safety Features

```c
// Safety mechanisms for discharge control
void enforceDischargeSeafetyLimits(DischargeControl* ctrl, 
                                   uint16_t voltage, 
                                   int16_t temperature) {
    // Voltage-based safety limits
    if (voltage < MINIMUM_SAFE_VOLTAGE) {  // e.g., 2.5V
        ctrl->forceDisable = true;
        logSafetyEvent(SAFETY_LOW_VOLTAGE);
    }
    
    if (voltage > MAXIMUM_SAFE_VOLTAGE) {  // e.g., 4.5V  
        ctrl->forceDisable = true;
        logSafetyEvent(SAFETY_HIGH_VOLTAGE);
    }
    
    // Temperature-based safety limits
    if (temperature > MAXIMUM_DISCHARGE_TEMP) {  // e.g., 60°C
        ctrl->forceDisable = true;
        logSafetyEvent(SAFETY_HIGH_TEMPERATURE);
    }
    
    if (temperature < MINIMUM_DISCHARGE_TEMP) {  // e.g., -10°C
        ctrl->forceDisable = true;
        logSafetyEvent(SAFETY_LOW_TEMPERATURE);
    }
    
    // Communication loss safety
    if (sg_u8CommunicationTimeoutCounter > MAX_COMM_TIMEOUT) {
        ctrl->forceDisable = true;
        logSafetyEvent(SAFETY_COMMUNICATION_LOSS);
    }
    
    // Clear force disable if all conditions are acceptable
    if (voltage >= MINIMUM_SAFE_VOLTAGE && 
        voltage <= MAXIMUM_SAFE_VOLTAGE &&
        temperature <= MAXIMUM_DISCHARGE_TEMP &&
        temperature >= MINIMUM_DISCHARGE_TEMP &&
        sg_u8CommunicationTimeoutCounter < MAX_COMM_TIMEOUT) {
        ctrl->forceDisable = false;
    }
}
```

## Data Acquisition Pipeline

### Measurement Coordination

The CellCPU coordinates voltage and temperature measurements while managing pin multiplexing between ADC and I2C functions.

```c
// Complete measurement cycle
typedef struct {
    uint16_t voltage;          // Raw ADC voltage reading
    uint16_t temperature;      // Raw temperature reading  
    uint8_t measurementFlags;  // Status and error flags
    uint32_t timestamp;        // Measurement timestamp (if needed)
} CellMeasurement;

// Flags for measurement status
#define MEASUREMENT_VOLTAGE_VALID    0x01
#define MEASUREMENT_TEMP_VALID       0x02
#define MEASUREMENT_I2C_ERROR        0x04
#define MEASUREMENT_ADC_ERROR        0x08
#define MEASUREMENT_RANGE_ERROR      0x10

CellMeasurement performCompleteMeasurement(void) {
    CellMeasurement result = {0};
    
    // 1. Voltage measurement (ADC)
    configurePinForADC();  // Switch PB3 to ADC mode
    _delay_us(100);        // Allow settling time
    
    result.voltage = measureCellVoltage();
    if (result.voltage > 0 && result.voltage < 1024) {
        result.measurementFlags |= MEASUREMENT_VOLTAGE_VALID;
    } else {
        result.measurementFlags |= MEASUREMENT_ADC_ERROR;
    }
    
    // 2. Temperature measurement (I2C)
    configurePinsForI2C(); // Switch PB3 to I2C SCL mode
    _delay_us(100);        // Allow settling time
    
    result.temperature = MCP9843ReadTemperature();
    if (!(result.temperature & TEMP_I2C_ERROR_FLAG)) {
        result.measurementFlags |= MEASUREMENT_TEMP_VALID;
        
        // Range check
        int16_t tempInTenthDegrees = convertRawToTemperature(result.temperature);
        if (tempInTenthDegrees < -400 || tempInTenthDegrees > 1250) {
            result.measurementFlags |= MEASUREMENT_RANGE_ERROR;
        }
    } else {
        result.measurementFlags |= MEASUREMENT_I2C_ERROR;
    }
    
    // 3. Return to ADC mode for next cycle
    configurePinForADC();
    
    return result;
}

// Process measurement results and update global state
void processMeasurementResults(CellMeasurement* measurement) {
    // Update voltage data
    if (measurement->measurementFlags & MEASUREMENT_VOLTAGE_VALID) {
        sg_u16BatteryVoltage = measurement->voltage;
    }
    // Keep last valid voltage if current measurement failed
    
    // Update temperature data  
    if (measurement->measurementFlags & MEASUREMENT_TEMP_VALID) {
        sg_u16BatteryTemperature = measurement->temperature;
    } else {
        // Flag temperature as invalid
        sg_u16BatteryTemperature = measurement->temperature | TEMP_I2C_ERROR_FLAG;
    }
    
    // Update discharge control based on latest measurements
    if (measurement->measurementFlags & MEASUREMENT_VOLTAGE_VALID) {
        updateDischargeControl(&sg_dischargeControl, measurement->voltage);
    }
}
```

## Safety and Protection

### Cell-Level Protection Features

```c
// Comprehensive safety monitoring
typedef struct {
    uint16_t minVoltageLimit;      // Minimum safe voltage (ADC units)
    uint16_t maxVoltageLimit;      // Maximum safe voltage (ADC units)
    int16_t minTempLimit;          // Minimum safe temperature (0.1°C)
    int16_t maxTempLimit;          // Maximum safe temperature (0.1°C)
    uint8_t voltageViolationCount; // Consecutive voltage violations
    uint8_t tempViolationCount;    // Consecutive temperature violations
    bool safetyShutdown;           // Emergency shutdown flag
} SafetyLimits;

void checkSafetyLimits(SafetyLimits* limits, 
                       uint16_t voltage, 
                       int16_t temperature) {
    bool violation = false;
    
    // Voltage limit checking
    if (voltage < limits->minVoltageLimit || 
        voltage > limits->maxVoltageLimit) {
        limits->voltageViolationCount++;
        violation = true;
        
        if (limits->voltageViolationCount >= SAFETY_VIOLATION_THRESHOLD) {
            limits->safetyShutdown = true;
            // Immediately disable discharge
            sg_dischargeControl.forceDisable = true;
        }
    } else {
        limits->voltageViolationCount = 0;  // Reset on valid reading
    }
    
    // Temperature limit checking
    if (temperature < limits->minTempLimit || 
        temperature > limits->maxTempLimit) {
        limits->tempViolationCount++;
        violation = true;
        
        if (limits->tempViolationCount >= SAFETY_VIOLATION_THRESHOLD) {
            limits->safetyShutdown = true;
            sg_dischargeControl.forceDisable = true;
        }
    } else {
        limits->tempViolationCount = 0;     // Reset on valid reading
    }
    
    // Report safety violations in status
    if (violation) {
        sg_u8SafetyStatus |= SAFETY_LIMIT_VIOLATION;
    } else {
        sg_u8SafetyStatus &= ~SAFETY_LIMIT_VIOLATION;
    }
}
```

## Calibration and Accuracy

### Factory Calibration Process

```c
// Calibration data structure stored in EEPROM
typedef struct {
    uint16_t voltageOffsetCal;   // ADC offset correction
    uint16_t voltageGainCal;     // ADC gain correction  
    int16_t tempOffsetCal;       // Temperature offset correction
    uint8_t calValidationCode;   // Calibration validity marker
    uint16_t serialNumber;       // Unique device identifier
    uint16_t checksum;           // Data integrity check
} CalibrationData;

// Apply calibration to raw measurements
uint16_t applyCalibratedVoltage(uint16_t rawADC) {
    CalibrationData cal;
    
    // Read calibration from EEPROM
    if (!readCalibrationData(&cal)) {
        return rawADC;  // Use uncalibrated if cal data invalid
    }
    
    // Apply offset and gain corrections
    int32_t corrected = (int32_t)rawADC - cal.voltageOffsetCal;
    corrected = (corrected * (int32_t)cal.voltageGainCal) >> 16;
    
    // Clamp to valid ADC range
    if (corrected < 0) corrected = 0;
    if (corrected > 1023) corrected = 1023;
    
    return (uint16_t)corrected;
}

int16_t applyCalibratedTemperature(uint16_t rawTemp) {
    CalibrationData cal;
    
    if (!readCalibrationData(&cal)) {
        return convertRawToTemperature(rawTemp);
    }
    
    int16_t temperature = convertRawToTemperature(rawTemp);
    return temperature + cal.tempOffsetCal;
}
```

## Diagnostic Features

### Built-in Self Test

```c
// Comprehensive self-test functionality
typedef enum {
    SELF_TEST_PASS = 0,
    SELF_TEST_ADC_FAIL,
    SELF_TEST_I2C_FAIL,
    SELF_TEST_TEMP_SENSOR_FAIL,
    SELF_TEST_VOLTAGE_RANGE_FAIL,
    SELF_TEST_DISCHARGE_CONTROL_FAIL
} SelfTestResult;

SelfTestResult performSelfTest(void) {
    // Test 1: ADC functionality
    configurePinForADC();
    uint16_t adcTest = ADCRead();
    if (adcTest == 0 || adcTest >= 1023) {
        return SELF_TEST_ADC_FAIL;
    }
    
    // Test 2: I2C communication
    configurePinsForI2C();
    uint16_t deviceID = MCP9843ReadDeviceID();
    if (deviceID != EXPECTED_MCP9843_DEVICE_ID) {
        return SELF_TEST_I2C_FAIL;
    }
    
    // Test 3: Temperature sensor reading
    uint16_t tempReading = MCP9843ReadTemperature();
    if (tempReading & TEMP_I2C_ERROR_FLAG) {
        return SELF_TEST_TEMP_SENSOR_FAIL;
    }
    
    // Test 4: Voltage range check
    uint16_t voltage = measureCellVoltage();
    uint16_t millivolts = adcToMillivolts(voltage);
    if (millivolts < 1000 || millivolts > 5000) {  // 1V to 5V expected range
        return SELF_TEST_VOLTAGE_RANGE_FAIL;
    }
    
    // Test 5: Discharge control
    bool originalDischargeState = sg_bDischargeActive;
    MCP9843SetEventPin(true);   // Test disable
    _delay_ms(10);
    MCP9843SetEventPin(false);  // Test enable
    _delay_ms(10);
    MCP9843SetEventPin(!originalDischargeState);  // Restore original state
    
    return SELF_TEST_PASS;
}

// Diagnostic data reporting
typedef struct {
    uint8_t selfTestResult;      // Last self-test result
    uint8_t i2cErrorCount;       // I2C communication errors
    uint8_t adcErrorCount;       // ADC measurement errors
    uint8_t safetyViolationCount;// Safety limit violations
    uint16_t operationHours;     // Operating time counter
    uint8_t temperatureSensorStatus; // Temperature sensor health
} DiagnosticData;
```

The CellCPU cell interface provides comprehensive monitoring and control capabilities while maintaining the simplicity and reliability required for safety-critical battery management applications. The combination of precision measurement, robust safety features, and diagnostic capabilities ensures reliable operation across the full range of environmental and operational conditions.