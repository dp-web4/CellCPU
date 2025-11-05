# VUART RX Timing Fix for CPU #5 Corruption

## Corrected Problem Analysis

**Key Insight**: Each CPU retransmits at its own clock rate - drift does NOT accumulate through the chain!

**Actual Problem**: CPU #5's slow oscillator (-8% at 20°C) causes RX sampling drift over 10 bits:

```
CPU #5 running at 7.36MHz (-8%), timer tick = 1.087μs
Upstream transmitting at nominal 8MHz, bit time = 50.00μs

Bit 0: Sample at  27.2μs (should be 25.0μs) →  2.2μs late
Bit 1: Sample at  54.3μs (should be 50.0μs) →  4.3μs late
Bit 2: Sample at  81.5μs (should be 75.0μs) →  6.5μs late
Bit 8: Sample at 217.4μs (should be 200μs) → 17.4μs late (35% into bit!)
```

**Plus ISR overhead**: Each timer reschedule (line 135) takes 4-5 cycles before the actual `TIMER_CHA_INT()` macro executes. This adds ~5μs per bit.

**Combined effect**: By bit 8, CPU #5 is sampling 22μs late = 44% into the bit window → corrupted data!

## Current Code Issue

**vUART.c line 135** - Used by BOTH RX and TX states:
```c
TIMER_CHA_INT(VUART_BIT_TICKS-VUART_ISR_OVERHEAD);  // VUART_ISR_OVERHEAD = 0
```

**Problem**:
- During **RX** (relaying): Need to compensate for ISR overhead to maintain sampling accuracy
- During **TX** (originating): Want EXACT bit timing at processor's own clock rate

Currently `VUART_ISR_OVERHEAD = 0`, so NO compensation on RX!

## Proposed Solution

### Option 1: Separate RX and TX Overhead Constants (RECOMMENDED)

Add separate constants for RX vs TX timing compensation:

**Shared.h changes**:
```c
// Virtual UART timing - How many timer ticks is a single data/start/stop bit?
#define VUART_BIT_TICKS         50

// ISR overhead compensation for RECEIVE operations (empirical tuning)
// Compensates for code execution time before timer reschedule during RX
#define VUART_RX_ISR_OVERHEAD   5  // 5μs compensation for RX path

// ISR overhead compensation for TRANSMIT operations
// Set to 0 to maintain exact bit timing at processor's own clock rate
#define VUART_TX_ISR_OVERHEAD   0  // No compensation for TX - use native timing

// How far into the first data bit to sample after start bit detection
#define VUART_SAMPLE_OFFSET     ((VUART_BIT_TICKS / 2) - VUART_RX_ISR_OVERHEAD)
```

**vUART.c changes** (line 135):
```c
ISR(TIMER_COMPA_VECTOR, ISR_BLOCK)
{
    if (ESTATE_RX_DATA == sg_ecell_up_rxState)
    {
        // RX path - compensate for ISR overhead to maintain sampling accuracy
        TIMER_CHA_INT(VUART_BIT_TICKS - VUART_RX_ISR_OVERHEAD);

        // [rest of RX code]
    }
    else if (ESTATE_TX_DATA == sg_ecell_up_rxState)
    {
        // TX path - no compensation, use exact bit timing at CPU's clock rate
        TIMER_CHA_INT(VUART_BIT_TICKS - VUART_TX_ISR_OVERHEAD);

        // [rest of TX code]
    }
}
```

**Similarly for Channel B (line 170)**:
```c
ISR(TIMER_COMPB_VECTOR, ISR_BLOCK)
{
    // This is always RX path (cell_dn_rx -> cell_up_tx relay)
    TIMER_CHB_INT(VUART_BIT_TICKS - VUART_RX_ISR_OVERHEAD);

    // [rest of code]
}
```

### Option 2: State-Based Overhead (Alternative)

Keep single constant but apply it conditionally:

**Shared.h**:
```c
#define VUART_ISR_OVERHEAD   5  // Only used for RX
```

**vUART.c line 135**:
```c
ISR(TIMER_COMPA_VECTOR, ISR_BLOCK)
{
    if (ESTATE_RX_DATA == sg_ecell_up_rxState)
    {
        TIMER_CHA_INT(VUART_BIT_TICKS - VUART_ISR_OVERHEAD);  // RX: compensate
    }
    else if (ESTATE_TX_DATA == sg_ecell_up_rxState)
    {
        TIMER_CHA_INT(VUART_BIT_TICKS);  // TX: no compensation
    }
}
```

## Why This Fixes The Problem

**Current timing (broken)**:
```
RX: Schedule next sample at TCNT0 + 50 ticks
Actual interval: ISR overhead (~5μs) + 50 ticks = 55μs
With -8% oscillator: 55 × 1.087 = 59.8μs between samples!
After 8 bits: 59.8 - 50 = 9.8μs drift per bit × 8 = 78μs total drift
```

**With fix (VUART_RX_ISR_OVERHEAD = 5)**:
```
RX: Schedule next sample at TCNT0 + 45 ticks
Actual interval: ISR overhead (~5μs) + 45 ticks = 50μs target
With -8% oscillator: 50 × 1.087 = 54.35μs between samples
After 8 bits: 54.35 - 50 = 4.35μs drift per bit × 8 = 34.8μs total drift
Still within bit window! (50μs bit ± 25μs margin)
```

**TX remains unaffected**:
```
TX: Schedule next bit at TCNT0 + 50 ticks
Transmits at CPU's own clock rate (whatever that is)
Downstream receiver syncs on each start bit, so oscillator variance doesn't matter
```

## Testing Plan

1. **Measure actual ISR overhead**:
   - Use oscilloscope on profiler pins (if available on ATtiny261A)
   - Or estimate: Line 135 to line 148 ≈ 40-50 cycles @ 8MHz = 5-6μs

2. **Start with VUART_RX_ISR_OVERHEAD = 5**:
   - Rebuild CellCPU and ModuleCPU
   - Test CPU #5 at 20°C
   - Should see immediate improvement

3. **Fine-tune if needed**:
   - If still unreliable: increase to 6-7
   - If TOO aggressive (missing start of bits): decrease to 3-4

4. **Verify TX not affected**:
   - Confirm ModuleCPU still receives clean data
   - Check that inter-byte timing is consistent

## Implementation Recommendation

**Use Option 1** (separate constants) because:
1. Makes intent crystal clear (RX needs compensation, TX doesn't)
2. Easier to tune independently
3. Self-documenting code
4. Follows embedded best practice of explicit timing requirements

## Expected Result

CPU #5 (and any other marginal oscillators) will:
- Sample RX bits earlier in the window
- Compensate for ISR execution overhead
- Maintain sampling accuracy despite slow oscillator
- Still transmit at its own clock rate (which is fine - each start bit resyncs)

This should fix the corruption at 20°C without affecting any working CPUs.
