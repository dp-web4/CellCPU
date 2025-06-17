/* CellCPU
 *
 * Cell controller firmware - Virtual UART module
 *
 * (C) Copyright 2023-2024 Modular Battery Technologies, Inc.
 * US Patents 11,380,942; 11,469,470; 11,575,270; others. All
 * rights reserved
 **/

#ifndef _VUART_H_
#define _VUART_H_

extern bool vUARTStartcell_dn_tx(uint8_t u8StartDelayTicks);
extern void vUARTPinInit(void);
extern void vUARTInitTransmit(void);
extern void vUARTInitReceive(void);
extern bool vUARTIscell_dn_rxActive(void);

#endif
