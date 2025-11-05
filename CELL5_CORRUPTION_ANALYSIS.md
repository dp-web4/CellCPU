# CellCPU #5 Data Corruption Analysis

## Problem Summary

**Symptom**: CPU #5 in daisy chain corrupts/truncates data during relay operation
- When isolated (UPRX shorted to GND), CPU #5 works correctly with 5 cells
- When detecting CPU #6 downstream, entire chain becomes unreliable
- Reports 11-13 out of 13 cells (intermittent)
- Temperature dependent: unreliable at 20°C, better at 30°C
- Replacing CPU #5 fixes the problem completely

**Observation**: Instead of cleanly relaying received data from upstream and appending its own, CPU #5 transmits garbage/truncated data downstream.

## Root Cause Analysis

### 1. RC Oscillator Variance (Primary Suspect)

**ATtiny internal RC oscillator specification**: ±10% over temperature/voltage

**Critical timing calculations**:
```
Timer prescaler: /8
CPU clock: 8MHz (nominal)
Timer clock: 1MHz → 1μs per tick
VUART_BIT_TICKS: 50 ticks = 50μs per bit
Baud rate: 1/50μs = 20kbps
```

**Impact of oscillator variance**:
- Nominal CPU at 8.00MHz → 50.00μs bit time
- Fast CPU at 8.80MHz (+10%) → 45.45μs bit time
- Slow CPU at 7.20MHz (-10%) → 55.56μs bit time

**Cumulative drift through chain**:
- Each CPU relays data with 1-bit pipeline delay (vUART.c line 138-146)
- CPU #5 has accumulated timing drift from CPUs #1-4 upstream
- If CPU #5 is running slow (-8% at 20°C), it samples late in each bit
- By bit #8 of a byte, sampling point may have drifted outside valid window
- This explains truncated/corrupted retransmission

### 2. Critical Race Condition in Relay Logic

**The 1-Bit Pipeline Relay Mechanism** (vUART.c lines 133-157):

```c
ISR(TIMER_COMPA_VECTOR, ISR_BLOCK)  // cell_up_rx → cell_dn_tx relay
{
    TIMER_CHA_INT(VUART_BIT_TICKS-VUART_ISR_OVERHEAD);  // line 135
    if (ESTATE_RX_DATA == sg_ecell_up_rxState)
    {
        // Set the bit value for what the PRIOR state was (1-bit delay)
        if (sg_bcell_up_rxPriorState)  // line 139
        {
            CELL_DN_TX_ASSERT();  // line 141
        }
        else
        {
            CELL_DN_TX_DEASSERT();  // line 145
        }

        sg_bcell_up_rxPriorState = IS_PIN_CELL_UP_RX_ASSERTED();  // line 148
        sg_u8Cell_up_rxBitCount++;  // line 150
```

**Timing diagram** (nominal case):
```
Time:     0    25   50   75   100  125  150
UPRX:     |____|----|----|____|----|----|  (upstream data)
Sample:        ^         ^         ^        (timer interrupt fires)
Prior:    (1)  (0)  (1)  (1)  (0)  (1)     (saved from previous sample)
DNTX:     -----____-----____-----____      (output 1 bit delayed)
```

**What happens with slow oscillator** (-8% at CPU #5):
```
CPU #5 Timer: 50 ticks × 1.087 = 54.35μs actual

Time:     0    27   54   81   108  135
UPRX:     |____|----|----|____|----|----|  (upstream at 50μs/bit)
Sample:        ^         ^         ^        (CPU #5 fires late by 2μs, 4μs, 6μs...)
```

By bit #8, CPU #5 is sampling 8×4μs = 32μs late, which is 64% into the NEXT bit!
This causes:
- Wrong bit sampled → corrupted data
- Bit count may be off → truncated transmission
- Stop bit misread → thinks more data coming when there isn't (or vice versa)

### 3. Interrupt Latency Vulnerability

**ISR execution order** (vUART.c line 81-130):

Pin change interrupt (PCINT) detects start bit:
1. Line 94: `TIMER_CHA_INT(VUART_BIT_TICKS + VUART_SAMPLE_OFFSET)` - schedule first sample at 25 ticks
2. Line 97: `INT_CELL_UP_RX_DISABLE()` - disable pin change interrupt
3. Line 100: `sg_ecell_up_rxState = ESTATE_RX_DATA` - enter RX state

Timer interrupt (TIMER_COMPA) samples and relays bits:
- Line 135: First instruction is `TIMER_CHA_INT(VUART_BIT_TICKS-VUART_ISR_OVERHEAD)` - reschedule
- This MUST happen quickly to maintain timing accuracy
- Any delay here propagates to all subsequent bits

**Potential race with other interrupts**:
- Timer overflow interrupt (1ms @ 8MHz/8/256)
- Watchdog timer interrupt (periodic callbacks)
- I2C bit-bang operations (if running during VUART)

If timer interrupt is delayed by even 5-10μs due to another ISR, timing corrupts.

### 4. Why Temperature Makes It Better

**RC oscillator frequency vs temperature** (typical ATtiny behavior):
- Frequency INCREASES with temperature
- At 20°C: CPU might be -8% (7.36MHz)
- At 30°C: CPU might be -3% (7.76MHz)

Warmer CPU = faster clock = samples earlier in bit window = more margin!

## Vulnerable Code Sections

### Section 1: RX Relay Timing (vUART.c lines 135-157)

**Issue**: Timer reschedule happens AFTER sampling, creating variable latency

```c
TIMER_CHA_INT(VUART_BIT_TICKS-VUART_ISR_OVERHEAD);  // line 135 - should be FIRST
if (ESTATE_RX_DATA == sg_ecell_up_rxState)
{
    // 20+ cycles of code before next timer setting
    if (sg_bcell_up_rxPriorState) { CELL_DN_TX_ASSERT(); }
    else { CELL_DN_TX_DEASSERT(); }
    sg_bcell_up_rxPriorState = IS_PIN_CELL_UP_RX_ASSERTED();
    sg_u8Cell_up_rxBitCount++;
```

**Current overhead compensation**: `VUART_ISR_OVERHEAD = 0` (Shared.h line 19)
This was set to 0, but empirical tuning may be needed!

### Section 2: Start Bit Detection (vUART.c lines 88-103)

**Issue**: Sample offset may not account for oscillator drift in chain position

```c
// This causes a sampling in the middle of the waveform
// and accounts for code overhead.
TIMER_CHA_INT(VUART_BIT_TICKS + VUART_SAMPLE_OFFSET);  // line 94
```

Where `VUART_SAMPLE_OFFSET = (VUART_BIT_TICKS / 2) - VUART_ISR_OVERHEAD = 25 - 0 = 25`

**For CPU #5 with cumulative drift**, the "middle" of the bit is no longer at +25 ticks!

### Section 3: TX During RX State Overlap (vUART.c lines 136-148)

**Issue**: State machine allows TX pin changes during RX bit sampling

The relay output (CELL_DN_TX) is changed DURING the same ISR that samples CELL_UP_RX.
If ISR execution is slow, this could corrupt the sample read.

## Proposed Fixes

### Fix 1: Adaptive Sampling Offset (Preferred)

Add per-chain-position sampling offset to compensate for cumulative drift:

```c
// In Platform.h or Shared.h:
#define VUART_SAMPLE_OFFSET_BASE    25  // Nominal center of bit

// In main.c, pass chain position to VUART init:
extern uint8_t g_u8ChainPosition;  // 0 for first CPU, increments downstream

// In vUART.c, adjust sampling:
#define VUART_SAMPLE_OFFSET  (VUART_SAMPLE_OFFSET_BASE - (g_u8ChainPosition * 2))
```

**Rationale**: Each upstream CPU adds ~2μs drift (4% of bit time), so sample 2 ticks earlier per position.

### Fix 2: Increase ISR_OVERHEAD Compensation

Tune `VUART_ISR_OVERHEAD` empirically:

```c
// Shared.h line 19:
#define VUART_ISR_OVERHEAD    4  // Was 0, try 4μs
```

**Test procedure**:
1. Set VUART_ISR_OVERHEAD to 4
2. Test with known-good CPUs at 20°C
3. If reliable, this confirms ISR latency is the issue
4. If still fails, increase to 6-8

### Fix 3: Wider Bit Timing (Conservative)

Reduce baud rate to give more timing margin:

```c
// Shared.h line 15:
#define VUART_BIT_TICKS    60  // Was 50 (20kbps), now 16.67kbps
```

**Tradeoff**: 17% slower data rate, but 20% more margin for oscillator drift.

**Pro**: Simple, affects all CPUs equally, no chain-position logic needed
**Con**: Longer frame transmission time (94 cells × 2 bytes × 11 bits × 60μs = 124ms)

### Fix 4: Interrupt Priority Guard (Safety Enhancement)

Disable other interrupts during VUART relay:

```c
// In PCINT ISR, when starting RX:
cli();  // Disable all interrupts
sg_ecell_up_rxState = ESTATE_RX_DATA;
sei();  // Re-enable after state change
```

**Warning**: This could cause watchdog reset if VUART transaction takes too long!
Only use if we confirm other ISRs are interfering.

## Recommended Testing Sequence

1. **Measure actual oscillator frequency** of CPU #5:
   - Use scope to measure VUART bit time output
   - Calculate: actual_freq = 8MHz × (50μs / measured_bit_time)
   - If measured_bit_time = 54μs → CPU is running at 7.41MHz (-7.4%)

2. **Test Fix #2 first** (easiest, no logic changes):
   - Set `VUART_ISR_OVERHEAD = 4` in Shared.h
   - Rebuild both CellCPU and ModuleCPU (shared header!)
   - Test with CPU #5 at 20°C

3. **If Fix #2 insufficient, try Fix #3**:
   - Set `VUART_BIT_TICKS = 60`
   - Rebuild both projects
   - Verify ModuleCPU still has enough time between frames

4. **If still unreliable, implement Fix #1** (most complex):
   - Requires chain position detection logic
   - Each cell must know its position (currently not tracked)
   - Would need to be determined during first read after power-on

## Questions to Answer

1. **Is CPU #5 always the failing position, or does the bad CPU fail anywhere?**
   - If position-dependent → cumulative timing drift confirmed
   - If CPU-dependent → just a bad oscillator on that specific chip

2. **What is the actual bit time transmitted by CPU #5?**
   - Scope measurement will reveal oscillator frequency
   - Confirms whether it's -8% or something else

3. **Are there ANY other interrupts running during VUART?**
   - Review main.c for timer overflows, WDT callbacks
   - Check if I2C operations ever happen during VUART relay

4. **Does the corruption always happen at the same byte/bit position?**
   - If always byte #8 → suggests cumulative drift
   - If random → suggests interrupt race condition

## Why Replacing CPU Fixes It

If the replacement CPU's RC oscillator is closer to nominal (e.g., -2% instead of -8%), it has much more timing margin:

- Bad CPU at -8%: By bit #8, drifted 32μs → sampling wrong bit
- Good CPU at -2%: By bit #8, drifted 8μs → still within bit window

**This is NOT a design flaw** - it's marginal hardware (out-of-spec oscillator) exposing a timing sensitivity.

The fix should make the system more robust to oscillator variance, which will improve reliability across temperature and manufacturing variation.
