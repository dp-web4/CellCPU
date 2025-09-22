# Edge Sync Problem Analysis - CellCPU VUART

## Problem Statement
The edge sync implementation breaks VUART relay timing, causing corrupted communication between cells. While edge sync successfully improves local reception, it inadvertently distorts the timing of relayed bits to neighboring cells.

## How VUART Relay Works

### Architecture
- Each cell has two VUART channels:
  - **Upstream (cell_up_rx → cell_dn_tx)**: Receives from cell below, relays + appends data to cell above
  - **Downstream (cell_dn_rx → cell_up_tx)**: Receives from cell above, relays to cell below

### Real-Time Relay Mechanism
Both channels use 1-bit delayed relay without buffering:
1. Timer fires every bit period
2. Outputs the PREVIOUS bit to the relay pin
3. Samples the CURRENT bit from the receive pin
4. Creates a 1-bit pipeline delay

### Critical Point: No Buffering
- Data is NOT stored and retransmitted
- Each bit is relayed in real-time as it arrives
- Timing of output bits is directly tied to timer interrupts

## Original Implementation (Working)

### Start Bit Detection (PCINT ISR)
```c
// Falling edge detected
if (bCellUpRXAsserted && ...) {
    TIMER_CHA_INT(VUART_BIT_TICKS);  // Set timer for 1 bit period
    INT_CELL_UP_RX_DISABLE();         // CRITICAL: Disable interrupts
    sg_bcell_up_rxPriorState = true;  // Start bit value
    sg_u8Cell_up_rxBitCount = 0;
}
```

### Relay Timing (Timer ISR)
```c
// Fixed timing, no adjustments possible
TIMER_CHA_INT(VUART_BIT_TICKS-6);  // Next interrupt in 1 bit period
// Output previous bit
if (sg_bcell_up_rxPriorState)
    CELL_DN_TX_ASSERT();
else
    CELL_DN_TX_DEASSERT();
// Sample current bit
sg_bcell_up_rxPriorState = IS_PIN_CELL_UP_RX_ASSERTED();
```

### Result
- Start bit width on output: ~1 bit period (consistent)
- Data bit widths: 1 bit period each (consistent)
- No timing variations possible after start bit

## Edge Sync Implementation (Broken)

### Start Bit Detection (PCINT ISR)
```c
if (bCellUpRXAsserted && ...) {
    // Set timer for 1.5 bit periods (middle of first data bit)
    TIMER_CHA_INT(VUART_BIT_TICKS + (VUART_BIT_TICKS/2) - VUART_BIT_TICK_OFFSET);
    // INT_CELL_UP_RX_DISABLE(); // REMOVED - keep enabled for edge correction
    sg_bcell_up_rxPriorState = true;
    sg_u8Cell_up_rxBitCount = 0;
}
```

### Edge Correction During Reception
```c
else if (ESTATE_RX_DATA == sg_ecell_up_rxState) {
    // Any edge resyncs the timer
    OCR0A = (uint8_t)(u8CurrentTimer + (VUART_BIT_TICKS/2) - VUART_BIT_TICK_OFFSET);
}
```

### Problem
- Start bit initially output for 1.5 bit periods
- Edge correction can fire at ANY time during byte
- Each correction changes the timer, affecting output bit width
- Downstream cell sees variable bit widths

## Why This Breaks Communication

### Timing Distortion Example
```
Original signal:     [START][D0][D1][D2]...
                     |--1--|--1--|--1--|--1--| (bit periods)

With edge sync:      [START----][D0-][D1---][D2]...
                     |--1.5--|--0.8--|--1.2--|--1--| (variable widths)
```

### Cascading Effect
1. Cell A transmits with perfect timing
2. Cell B receives with slight drift, applies edge correction
3. Cell B's relay to Cell C has distorted bit widths
4. Cell C may fail to decode or propagate further errors
5. By Cell 94, communication completely fails

### Start Bit Corruption
The start bit establishes synchronization. Variable start bit width causes:
- **Too short (<0.5 bit)**: Next cell misses start bit entirely
- **Too long (>1.5 bits)**: Next cell samples start bit as data
- **Variable**: Sampling points drift throughout byte

## Root Cause Analysis

### The Fundamental Conflict
Edge sync tries to solve two incompatible requirements:
1. **Improve local reception** by adjusting sampling time
2. **Maintain relay timing** for downstream transmission

These are coupled because the same timer controls both:
- Timer fires → Sample input bit AND output previous bit
- Adjusting timer for better sampling also changes output timing

### Why Original Design Worked
- Accepted occasional bit errors from timing drift
- Maintained consistent relay timing
- Errors didn't cascade through chain

### Why Edge Sync Fails
- Fixes local reception at the cost of relay integrity
- Creates NEW errors in relay path
- Errors cascade and amplify through chain

## Requirements for a Solution

### Must Have
1. Consistent output bit timing for relay path
2. Improved input sampling accuracy
3. No cascading timing distortion
4. Maintain real-time relay (no added latency)

### Nice to Have
1. Minimal code changes
2. Low CPU overhead
3. Works with existing hardware

## Potential Solutions

### Option 1: Disable Edge Sync
- Revert to original code
- Accept occasional bit errors
- **Pro**: Simple, proven to work
- **Con**: Loses timing drift correction

### Option 2: Edge Sync Only for Local Data
- Apply edge sync ONLY during final cell's transmission
- Keep relay portions without edge sync
- **Pro**: Improves local data accuracy
- **Con**: Doesn't help relay path accuracy

### Option 3: Separate Sample and Output Timing
- Use edge info to adjust sampling point
- Keep output on fixed intervals
- **Pro**: Best of both worlds
- **Con**: Complex to implement

### Option 4: Buffer and Retransmit
- Store entire byte before relaying
- Retransmit with clean timing
- **Pro**: Perfect output timing
- **Con**: Adds latency, major code change

### Option 5: Predictive Correction
- Track timing drift but don't adjust timer
- Use drift info to predict optimal sample point
- **Pro**: No output distortion
- **Con**: Complex math, may not be accurate enough

## Recommendation

**Short term**: Revert edge sync (Option 1)
- Immediate fix for broken communication
- Return to known working state

**Long term**: Implement Option 3 (Separate timing)
- Requires careful design but provides best solution
- Maintains backward compatibility
- Improves accuracy without breaking relay

## Next Steps

1. Decide on approach (revert vs. fix)
2. If fixing, design separation of sample/output timing
3. Consider testing with reduced cell count first
4. Implement comprehensive timing measurements