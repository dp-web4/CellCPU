# Proposed Edge Correction for CellCPU VUART Reception

## Problem
Temperature data showing -55.35°C (0x0000) indicates corrupted data during VUART reception. Without edge correction, timing drift during byte reception causes bit sampling errors.

## Solution
Implement edge-triggered timing correction similar to ModuleCPU's approach.

## Implementation Details

### 1. Add Edge Correction Variables
```c
// Edge correction tracking variables
static volatile int8_t sg_minTimingError_up;      // Min timing error on upstream
static volatile int8_t sg_maxTimingError_up;      // Max timing error on upstream  
static volatile uint16_t sg_edgeCorrections_up;   // Count of corrections on upstream
static volatile uint8_t sg_lastEdgeTimer_up;      // Timer value at last upstream edge

static volatile int8_t sg_minTimingError_dn;      // Min timing error on downstream
static volatile int8_t sg_maxTimingError_dn;      // Max timing error on downstream
static volatile uint16_t sg_edgeCorrections_dn;   // Count of corrections on downstream
static volatile uint8_t sg_lastEdgeTimer_dn;      // Timer value at last downstream edge

// Configuration
#define TIMING_TOLERANCE 3      // Only correct if error > 3 timer ticks
#define MAX_CORRECTION 5        // Maximum correction per edge
```

### 2. Modify PCINT ISR for Edge Detection Control
- After start bit detection, keep PCINT enabled for edge detection during byte
- Switch between monitoring specific pin vs any edge

### 3. Add Edge Correction Logic in PCINT ISR
During byte reception (ESTATE_RX_DATA state):
- Calculate timing error from expected mid-bit position
- If error > TIMING_TOLERANCE, resync timer to mid-bit
- Track min/max errors for diagnostics

### 4. Timer Tick Offset
- Define VUART_BIT_TICK_OFFSET (likely 6, same as current)
- Use for both initial sync and edge corrections

## Key Differences from ModuleCPU
- CellCPU uses PCINT for both pins (not INT1 like ModuleCPU)
- Must handle two independent receive channels (upstream and downstream)
- ATtiny45 has fewer resources - keep overhead minimal

## Testing Plan
1. Verify edge correction doesn't break basic VUART function
2. Check temperature data validity improves
3. Monitor correction statistics to verify it's working

## Risk Assessment
- **Low Risk**: Similar approach proven in ModuleCPU
- **Mitigation**: Can disable with #define if issues arise