/* CellCPU
 *
 * Cell controller firmware - MCP9843 driver header
 *
 * (C) Copyright 2023-2024 Modular Battery Technologies, Inc.
 * US Patents 11,380,942; 11,469,470; 11,575,270; others. All
 * rights reserved
 */

#ifndef MPC9843_H_
#define MPC9843_H_

extern int16_t MCP9843ReadTemperature(void);
extern bool MCP9843SetEventPin(bool bHigh);

#endif
