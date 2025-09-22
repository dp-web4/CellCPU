# Claude Context for CellCPU

## Edge Sync Revert (December 2024)

**IMPORTANT**: Edge sync for VUART was attempted but had to be reverted.

### What Was Tried
- Added edge-triggered timing correction during VUART reception
- Kept interrupts enabled during byte reception to detect edges
- Adjusted timer on each edge to resync to middle of bit

### Why It Failed
- VUART uses real-time bit relay (1-bit pipeline delay)
- Adjusting the timer for better sampling also changed OUTPUT bit timing
- This caused variable bit widths in the relayed signal
- Downstream cells saw corrupted timing, breaking communication

### Current State
- Reverted to original fixed-timing approach
- Accepts occasional bit errors but maintains consistent relay timing
- See EDGE_SYNC_ANALYSIS.md for detailed analysis

### Future Enhancement Ideas
- Track timing drift without adjusting timer
- Use drift info to optimize sampling within fixed intervals
- Consider separate timers for RX sampling vs TX generation

## Project Context System

**IMPORTANT**: A comprehensive context system exists at `/mnt/c/projects/ai-agents/misc/context-system/`

Quick access:
```bash
# Get overview of battery hierarchy projects
cd /mnt/c/projects/ai-agents/misc/context-system
python3 query_context.py project cellcpu

# See how this fits in the larger system
cat /mnt/c/projects/ai-agents/misc/context-system/projects/battery-hierarchy.md

# Find related concepts
python3 query_context.py concept "distributed intelligence"
```

## This Project's Role

CellCPU is the foundational layer of a three-tier battery management hierarchy:
- **CellCPU** (this) → ModuleCPU → Pack-Controller

Each cell has local intelligence (ATtiny45) managing:
- Temperature sensing via MCP9843
- Voltage monitoring
- Virtual UART communication at 20kbps
- Daisy-chain topology (up to 94 cells)

## Key Relationships
- **Integrates With**: ModuleCPU (via virtual UART)
- **Implements**: Distributed intelligence principles
- **Mirrors**: Synchronism's scale-based autonomy
- **Protected By**: US Patent 11,380,942

## Design Philosophy
Each cell acts as an autonomous agent with local decision-making, embodying the distributed intelligence pattern that appears across all projects. The cell's Markov blanket (its interface) hides internal complexity from the module level.