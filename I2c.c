/* CellCPU
 *
 * Cell controller firmware - bit bang I2C module
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
#include "i2c.h"
#include "main.h"
#include "Platform.h"

// Delays for one I2C bit's worth of time
static void I2CBitDelay(void)
{
	Delay(5);
}

// Causes the code to become synchronized with 
static void I2CQuantizeTiming(void)
{
	Delay(1);
}

// Send an I2C start sequence and quantize foreground code to the overflow
// timer
void I2CStart(void)
{
	I2CQuantizeTiming();

	SDA_SET_OUTPUT();
	SDA_HIGH();
	SCL_HIGH();
	I2CBitDelay();
	SDA_LOW();
	I2CBitDelay();
	SCL_LOW();
	I2CBitDelay();
}

// Send an I2C stop sequence
void I2CStop(void)
{
	I2CQuantizeTiming();

	SDA_LOW();
	I2CBitDelay();
	SCL_HIGH();
	I2CBitDelay();
	SDA_SET_INPUT();
	I2CBitDelay();
}

// Sends a single I2C byte. Returns false if the byte is not acked, or true if it is.
bool I2CTxByte(uint8_t u8Byte)
{
	uint8_t u8Length = 8;			// 8 Bits to send
	bool bAck = false;

	I2CQuantizeTiming();
	SDA_SET_OUTPUT();

	while (u8Length)
	{
		if (u8Byte & 0x80)
		{
			SDA_HIGH();
		}
		else
		{
			SDA_LOW();
		}
		
		// Drive SCL high
		SCL_HIGH();
		
		I2CBitDelay();
		
		// Ensure SCL is low
		SCL_LOW();
	
		// SDA Is now set. Delay.
		I2CBitDelay();
		
		// Next bit
		u8Byte <<= 1;	
		u8Length--;
	}
	
	SDA_SET_INPUT();
	SCL_HIGH();
	I2CBitDelay();
	
	// See if the byte was acknowledged
	if (SDA_READ())
	{
		bAck = false;
	}
	else
	{
		bAck = true;
	}
	
	SCL_LOW();
	I2CBitDelay();
	return(bAck);
}

// Receives a single I2C byte and will optionally generate an ack at the end
uint8_t I2CRxByte(bool bAck)
{
	uint8_t u8Data = 0;
	uint8_t u8Count = 8;
	
	I2CQuantizeTiming();

	SDA_SET_INPUT();
	
	// Consume all 8 data bits
	while (u8Count)
	{
		u8Data <<= 1;

		SCL_HIGH();
		I2CBitDelay();
			
		if (SDA_READ())
		{
			u8Data |= 1;
		}
		
		SCL_LOW();
		I2CBitDelay();
		u8Count--;
	}

	// See if we acknowledge this
	SDA_SET_OUTPUT();
	if (bAck)
	{
		SDA_LOW();
	}
	else
	{
		SDA_HIGH();
	}
	
	I2CBitDelay();
	SCL_HIGH();
	I2CBitDelay();
	SCL_LOW();
	I2CBitDelay();
	SDA_LOW();

	return(u8Data);	
}

// Prepares the SCL/SDA pins for I2C operation and returns when the CPU is
// quantized to the timer (for consistent bit alignment)
void I2CSetup(void)
{
	// Deassert SCL and SDA
	SCL_HIGH();
	SDA_HIGH();

	// Set SCL and SDA lines as push/pull output drives
	SCL_SET_OUTPUT();
	
	// Turn on SDA, set as an input
	SDA_SET_INPUT();
	
	Delay(20);
}

// Sets up the I2C pins, sends slave address/read/write byte, and waits for acknowledgment.
// Returns false if no device is responding.
bool I2CStartTransaction(uint8_t u8SlaveAddress,
						 bool bRead)
{
	// Set up the pins for I2C operation
	I2CSetup();
	
	// I2C Start condition
	I2CStart();

	// If it's a read operation, clear the lower bit
	if (bRead)
	{
		u8SlaveAddress |= 1;
	}
	else
	{
		// Otherwise set it
		u8SlaveAddress &= (uint8_t) (~1);
	}
	
	// Now send out the slave address + the read/write bit
	return(I2CTxByte(u8SlaveAddress));
}


