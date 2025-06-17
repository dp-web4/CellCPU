/* CellCPU
 *
 * Cell controller firmware - MCP9843 driver
 *
 * (C) Copyright 2023-2024 Modular Battery Technologies, Inc.
 * US Patents 11,380,942; 11,469,470; 11,575,270; others. All
 * rights reserved
 */

#include <stdint.h>
#include <xc.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "I2c.h"
#include "mcp9843.h"
#include "../Shared/Shared.h"

// Slave address
#define MCP9843_SLAVE_ADDRESS_SENSOR	0x30
#define MCP9843_SLAVE_ADDRESS_EEPROM	0xa0
#define MCP9843_SLAVE_ADDRESS_WP		0x60

// Register definitions
#define MCP9843REG_CAPBILITIES			0x00
#define MCP9843REG_CONFIG				0x01
#define MCP9843REG_EVENT_LOWER			0x02
#define MCP9843REG_EVENT_HIGHER			0x03
#define MCP9843REG_CRIT_TEMP_TRIP		0x04
#define MCP9843REG_TEMP					0x05
#define MCP9843REG_MFG_ID				0x06
#define MCP9843REG_DEV_ID				0x07
#define MCP9843REG_RESOLUTION			0x08

// _CONFIG bits

// Byte 1 (bits 8-15) - MSB
#define MCP9843REG_CONFIG_SHUTDOWN		(1 << 0)
#define	MCP9843REG_CONFIG_HYST_0		(0 << 1)
#define	MCP9843REG_CONFIG_HYST_1_5		(0x01 << 1)
#define	MCP9843REG_CONFIG_HYST_3_0		(0x02 << 1)
#define	MCP9843REG_CONFIG_HYST_6_0		(0x03 << 1)

// Byte 2 (bits 0-7) - LSB
#define MCP9843REG_CONFIG_OUTPUT_STATUS	(1 << 4)
#define MCP9843REG_CONFIG_EV_ENABLE		(1 << 3)
#define MCP9843REG_CONFIG_EV_POLARITY	(1 << 1)

// This is the default configuration bytes for the system
#define MCP9843_CONFIG_DEFAULT_MSB		(MCP9843REG_CONFIG_HYST_0)
#define MCP9843_CONFIG_DEFAULT_LSB		(MCP9843REG_CONFIG_EV_POLARITY | MCP9843REG_CONFIG_EV_ENABLE | MCP9843REG_CONFIG_OUTPUT_STATUS)

// Sets the register pointer to the incoming value, ready for subsequent transactions.
// Routine returns true if the device acknowledged the transactions, otherwise false
// is returned.
static bool MCP9843SetSensorRegister(uint8_t u8Register,
									 bool bRead)
{
	// Attempt to start a transaction
	if (false == I2CStartTransaction(MCP9843_SLAVE_ADDRESS_SENSOR,
									 bRead))
	{
		return(false);
	}
	
	// Now write the register. Since we're the master, we're monitoring ACK from the slave.
	if (false == I2CTxByte(u8Register))
	{
		// Not good.
		return(false);
	}
	
	return(true);
}

// This sets the EVENT pin either high or low depending on the input signal. The 
// logic requires that the thresholds are NOT triggered, as it uses the 
bool MCP9843SetEventPin(bool bHigh)
{
#if defined(__AVR_ATtiny261A__)
	return(true);
#else
	bool bResult = false;
	volatile uint8_t u8ConfigByteLSB = MCP9843_CONFIG_DEFAULT_LSB;
	
	// Set up the output pins for I2C operation
	I2CSetup();
	
	// Write the configuration register
	if (false == MCP9843SetSensorRegister(MCP9843REG_CONFIG,
										  false))
	{
		goto i2cFail;
	}
	
	// Config MSB
	if (false == I2CTxByte(MCP9843_CONFIG_DEFAULT_MSB))
	{
		goto i2cFail;
	}
	
	if (bHigh)
	{
		u8ConfigByteLSB |= MCP9843REG_CONFIG_EV_POLARITY;
	}
	else
	{
		u8ConfigByteLSB &= (uint8_t) (~MCP9843REG_CONFIG_EV_POLARITY);
	}
	
	// Config LSB
	if (false == I2CTxByte(u8ConfigByteLSB))
	{
		goto i2cFail;
	}
	
	// Indicate we're OK
	bResult = true;
	
	// And a stop condition
i2cFail:
	I2CStop();
	
	return(bResult);
#endif
}

// Reads the temperature from the MCP9843 in increments of 0.0625C degrees
// Bits 0-3  - 16ths of a degree C
// Bits 4-11 - Whole degrees C
// Bit 12    - Temperature sign bit - 0=Positive, 1=Negative
// Bits 13-14- Always 0
// Bit 15    - 1=Reading of temperature is valid, 0=Not valid
int16_t MCP9843ReadTemperature(void)
{
	int16_t s16Temperature;
	
#if defined(__AVR_ATtiny261A__)
	// Let's try 23.5 degrees
	s16Temperature = (23 << 4) | (0x08);
#else
	// Set up the output pins for I2C operation
	I2CSetup();
	
	// Write the temperature register
	if (false == MCP9843SetSensorRegister(MCP9843REG_TEMP,
										  false))
	{
		// Not valid
		return(0);											  
	}

	I2CStop();
	
	// Attempt to start a transaction
	if (false == I2CStartTransaction(MCP9843_SLAVE_ADDRESS_SENSOR,
									 true))
	{
		// Not valid
		return(0);
	}
	
	// Now read MSB/LSB
	s16Temperature = I2CRxByte(true) << 8;
	s16Temperature |= I2CRxByte(false);
	
	// Now the stop condition
	I2CStop();

	// Mask off the tcrit/tupper/tlower bits - they aren't useful
#endif
	s16Temperature &= 0xfff;
	s16Temperature |= MSG_CELL_TEMP_I2C_OK;
	
	return(s16Temperature);
}
