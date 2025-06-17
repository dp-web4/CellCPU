/* CellCPU
 *
 * Cell controller firmware - fuses
 *
 * (C) Copyright 2023-2024 Modular Battery Technologies, Inc.
 * US Patents 11,380,942; 11,469,470; 11,575,270; others. All
 * rights reserved
 */

#include <avr/io.h>

// Fuse calculations were made by using this web site. There are others out there, too,
// providing the same functionality:
//
// https://www.engbedded.com/fusecalc/
//
// In all definitions below, anything that's mentioned is considered to be SET (0). 
// Anything that is not mentioned should tbe considered NOT set (1). The low/high/extended
// fuse values come directly from the site above with the functionality set as documented in
// each CPU section.

#if defined(__AVR_ATtiny13A__)

// *************************************************************************************************
// ATTINY13 fuses
// *************************************************************************************************

// 9.6Mhz, startup time 14 CK + 0 ms (CKSEL=10 SUT=00)
// Brown out level at VCC=1.8V
// Reset disabled (RSTDISBL)
//
// NOTE: Once you program this, YOU WILL NOT BE ABLE TO PROGRAM IT AGAIN unless you use a high
// voltage (12V) capable AVR programmer.

FUSES =
{
	.low=0x42,
	.high=0x76
};

#elif defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__)

// *************************************************************************************************
// ATTINY25/45 fuses
// *************************************************************************************************

// 8Mhz, startup time 6CK/14CK + 0ms (CKSEL=0010 SUT=00)
// Brown out level at VCC=1.8V
// Reset disabled (RSTDISBL)
//
// NOTE: Once you program this, YOU WILL NOT BE ABLE TO PROGRAM IT AGAIN unless you use a high 
// voltage (12V) capable AVR programmer.

FUSES =
{
	.low=0x42,
	.high=0x76,
	.extended=0xff
};

#elif defined(__AVR_ATtiny261A__)

// *************************************************************************************************
// ATTINY261A fuses
// *************************************************************************************************

// 8Mhz, startup time 6CK/14CK + 0ms (CKSEL=0010 SUT=00)
// Brown out level at VCC=1.8V
// SPI Downloading enabled
// DebugWire enabled
// ** DO NOT DISABLE RESET, SPI, NOR DEBUGWIRE - IT IS NOT NECESSARY ON BEAVIS **

FUSES =
{
// Disables SPI/debugWire
	.low=0xc2,
	.high=0xfe,
	.extended=0xff
	
// Doesn't disable SPI/debugWire
//	.low=0xe2,
//	.high=0x9f,
//	.extended=0xff
};

#else
#error No target processor defined or target processor type unknown
#endif

