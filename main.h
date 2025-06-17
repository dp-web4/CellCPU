/* CellCPU
 *
 * Cell controller firmware - main module header
 *
 * (C) Copyright 2023-2024 Modular Battery Technologies, Inc.
 * US Patents 11,380,942; 11,469,470; 11,575,270; others. All
 * rights reserved
 */

#ifndef _MAIN_H_
#define _MAIN_H_

// CPU speed (in hz) - internal clock
#define CPU_SPEED						8000000

// Virtual UART timing - How many timer ticks is a single data/start/stop bit? This is the baud rate in ticks.
#define VUART_BIT_TICKS					50

// APIs to handle cell_up_rx->cell_dn_tx
extern bool Celldn_txDataAvailable(void);
extern uint8_t Celldn_txDataGet(void);
extern void Celldn_txDataReset(void);

// APIs to handle cell_dn_rx->cell_up_tx
extern void Celldn_rxDataStart(void);
extern void Celldn_rxDataBit(uint8_t u8DataBit);
extern void Celldn_rxDataEnd(void);

extern void Delay(uint16_t u16Ticks);

#endif
