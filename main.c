/* CellCPU
 *
 * Cell controller firmware - main module
 *
 * (C) Copyright 2023-2024 Modular Battery Technologies, Inc.
 * US Patents 11,380,942; 11,469,470; 11,575,270; others. All
 * rights reserved
 */

// Uncomment to use fake cell data for testing
// #define FAKE_CELL_DATA

#include "Platform.h"
#include "I2c.h"
#include "mcp9843.h"
#include "adc.h"
#include "main.h"
#include "../Shared/Shared.h"
#include "vUART.h"
#include <avr/sleep.h>
#include <avr/wdt.h>

// Most recent ADC (battery voltage) reading
static uint16_t sg_u16BatteryVoltage;							// Latest battery voltage reading

// Most recent temperature reading
static volatile uint16_t sg_u16BatteryTemperature;				// Latest battery temperature reading

// Target voltage. If sg_u16BatteryVoltage is higher than sg_u16BatterVoltageTarget, 
// turn on the discharge resistor
static volatile uint16_t sg_u16BatteryVoltageTarget;			// Target battery voltage (most recently received)

// Set true if the battery is currently being discharged
static volatile bool sg_bDischargeActive;

// Set true if this is the last cell CPU in the chain. If set true,
// this cell CPU will initiate transmission.
static volatile bool sg_bCellCPULast;

// This is the 16 bit quantity we get from the module controller - raw
static uint16_t sg_u16BatteryVoltageMsg;


// What state are we in?
typedef enum
{
	EACTION_NONE,
	EACTION_SEND_SENSOR_READING,
	EACTION_SEND_PATTERN,
	EACTION_INIT,
	EACTION_INITIATE_TRANSMIT
} ECellAction;

// Cell action - used by ISR to signal foreground wakeup
static ECellAction sg_eCellAction;

// Delay for at least u16Ticks timer B overflow ticks. If u16Ticks is 0, it will quantize to the
// next timer tick change.
void Delay(uint16_t u16Ticks)
{
	uint8_t u8Sample;
	
	u16Ticks++;
	
	u8Sample = TIMER_COUNTER();
	while (u16Ticks)
	{
		// Wait until we quantize
		while (u8Sample == TIMER_COUNTER());
		
		u8Sample = TIMER_COUNTER();
		--u16Ticks;
	}
}

// *************************************************************************
// cell_up_rx->cell_dn_tx handlers. This is data received from upstream cell
// CPUs and is retransmitted to other cell CPUs or to a module controller.
// *************************************************************************

// For our existing message, we will transmit 4 bytes:
//
// 0 - LSB Voltage
// 1 - MSB Voltage	(bit 7: 1=Discharge active, 0=Discharge inactive)
// 2 - LSB Temperature
// 3 - MSB Temperature (bit 7: 1=Valid I2C reading, 0=I2C communication error)

static uint8_t sg_u8TransmitOffset;

// Returns true if cell data is available. See message above for the meaning of >=4
bool Celldn_txDataAvailable(void)
{
	// We signal no more data available at the 4th byte (offset 3)
	// but still need to transmit the final byte.
	if (sg_u8TransmitOffset >= 4)
	{
		return(false);
	}
	
	return(true);
}

// Returns the next byte of cell data we want to transmit to the next cell, etc... module controller
uint8_t Celldn_txDataGet(void)
{
	uint8_t u8Data = 0xff;
	
	if (0 == sg_u8TransmitOffset)
	{
		// 0 - LSB Voltage
		u8Data = (uint8_t) sg_u16BatteryVoltage;
	}
	else
	if (1 == sg_u8TransmitOffset)
	{
		// 1 - MSB Voltage
		u8Data = (uint8_t) (sg_u16BatteryVoltage >> 8);
	}
	else
	if (2 == sg_u8TransmitOffset)
	{
		// 2 - LSB Temperature
		u8Data = (uint8_t) sg_u16BatteryTemperature;
	}
	else
	if (3 == sg_u8TransmitOffset)
	{
		// 3 - MSB Temperature
		u8Data = (uint8_t) (sg_u16BatteryTemperature >> 8);
	}
	
	sg_u8TransmitOffset++;
	return(u8Data);
}

// Resets the cell data state machine if necessary. This is called upon the falling
// edge of the start of a receive stream.
void Celldn_txDataReset(void)
{
	// Offset back to 0
	sg_u8TransmitOffset = 0;
}

// Battery target receive globals
static uint8_t sg_u8dn_rxBitCount = 0;

// Called right before data is transmitted/appended
void Celldn_txTransmitStart(void)
{
	// Sending the 0th byte
	sg_u8TransmitOffset = 0;
}

// *************************************************************************
// cell_dn_rx->cell_up_tx handlers. Received from module controller or cell
// CPU and transmitted to other cells.
// *************************************************************************

// Incoming message from the module controller:
//
// 0 - MSB Of voltage target
// 1 - LSB Of voltage target
//
// The cell_dn_rx->cell_up_tx path must do MSB first because it relies on
// getting the top two bits of the 16 bit quantity to indicate the command
// type.

// Called when start of message from module controller or upstream is started
void Celldn_rxDataStart(void)
{
	sg_u8dn_rxBitCount = 0;
	sg_u16BatteryVoltageMsg = 0;
}

// All messages from the module controller are 16 bit message. The top bit
// indicates if it's a setting of a voltage target or if it's a reading
// request. Bit 14 indicates the type of reading. Look in main.h for
// MSG_CELL_SEND_REPORT, MSG_CELL_SEND_PATTERN, and MSG_CELL_SEND_SENSORS
// for further specific illustration.

// Called for each received bit from the dn_rx direction (from the module controller)
void Celldn_rxDataBit(uint8_t u8DataBit)
{
	sg_u16BatteryVoltageMsg <<= 1;
	sg_u16BatteryVoltageMsg |= u8DataBit;
	sg_u8dn_rxBitCount++;

	// Special case after reception of the second data bit - if this is
	// a request for a reading, then signal the foreground code to do
	// a sensor reading. This hastens the response time of the initial request.

	if (2 == sg_u8dn_rxBitCount)
	{
		// After the second bit, let's see if we send a report. If so, then flag it
		// and record the type. The shift is necessary because we haven't received all 
		// 16 bits yet - just look at the two that we've received.
		if (sg_u16BatteryVoltageMsg & (MSG_CELL_SEND_REPORT >> 14))
		{
			// Indicates we want the cell CPU to send a report. Figure out which type.
			if (sg_u16BatteryVoltageMsg & (MSG_CELL_SEND_PATTERN >> 14))
			{
				sg_eCellAction = EACTION_SEND_PATTERN;
			}
			else
			{
				// Send an actual sensor reading. On the CPU at the end of the
				// chain, this will cause a reading to occur, then it will
				// originate transmission up the chain by sending its reading
				// to the next neighboring CPU. If it's not the CPU on the end,
				// it will simply perform the reading and wait for the downstream
				// message to append itself to.
				sg_eCellAction = EACTION_SEND_SENSOR_READING;
			}
		}
		else
		{
			// This is a new discharge target - ignore it
		}
	}
	else
	if (16 == sg_u8dn_rxBitCount)
	{
		if (sg_u16BatteryVoltageMsg & MSG_CELL_SEND_REPORT)
		{
			// We don't do anything because it already happened at bit 2 above.
		}
		else
		{
			// New voltage target - no need to mask off the upper bits since they'll be 0 here
			sg_u16BatteryVoltageTarget = sg_u16BatteryVoltageMsg;
		}
		
		sg_u16BatteryVoltageMsg = 0;
	}
}

/*
static void PinSetup(void)
{
	cli();
	
	// Set up up_tx and dn_tx as outputs
	PIN_CELL_UP_TX_DDR |= (1 << PIN_CELL_UP_TX);
	PIN_CELL_DN_TX_DDR |= (1 << PIN_CELL_DN_TX);
	
	// And SCL/SDA
	I2C_PORT_DDR |= (1 << I2C_SDA_PIN) | (1 << I2C_SCL_PIN);
	// And pullups
	I2C_PORT |= (1 << I2C_SDA_PIN) | (1 << I2C_SCL_PIN);

	// Set up cell_dn_rx and cell_up_rx as inputs and turn on pullups
	PIN_CELL_DN_RX_DDR &= ~(1 << PIN_CELL_DN_RX);
	PIN_CELL_DN_RX_PORT |= (1 << PIN_CELL_DN_RX);
	
	PIN_CELL_UP_RX_DDR &= ~(1 << PIN_CELL_UP_RX);
	PIN_CELL_UP_RX_PORT |= (1 << PIN_CELL_UP_RX);
}

// Endlessly toggles all outputs
static void ToggleAllOutputs(void)
{
	PinSetup();
	
	while (1)
	{
		CELL_UP_TX_ASSERT();
		CELL_DN_TX_ASSERT();
		SCL_LOW();
		SDA_LOW();
		CELL_UP_TX_DEASSERT();
		CELL_DN_TX_DEASSERT();
		SCL_HIGH();
		SDA_HIGH();
	}
}

// Mirror cell_dn_rx->cell_up_tx and cell_up_rx->cell_dn_tx
static void InputMirror(void)
{
	PinSetup();
	
	while (1)
	{
		if (IS_PIN_CELL_DN_RX_ASSERTED())
		{
			CELL_UP_TX_ASSERT();
		}
		else
		{
			CELL_UP_TX_DEASSERT();
		}
		
		if (IS_PIN_CELL_UP_RX_ASSERTED())
		{
			CELL_DN_TX_ASSERT();
		}
		else
		{
			CELL_DN_TX_DEASSERT();
		}
	}
}
*/

int main(void)
{
	// Stop all interrupts
	cli();
	
	// Shut off the watchdog timer and reset reason
	MCUSR = 0;
	DISABLE_WDT();
	
	// Init CPU clock
	CPU_CLOCK_INIT();
	
	// Ensure pullups aren't globally disabled
	MCUCR &= (uint8_t) ~(1 << PUD);
	
	// Initialize virtual UART pins
	vUARTPinInit();
	
    // Set the ADC input to MUX_SELECT
    ADC_ADMUX_SET();
	
	// Drive cell_dn_tx low. This is required so we can detect if we're end-of-chain
	// or not.
	CELL_DN_TX_ASSERT();
	
	// Now init timer B
	TIMER_INIT();
	
	// Set battery target voltage so the event/discharge is forced off
	sg_u16BatteryVoltageTarget = 0xffff;
	
	// This will cause discharge to get shut off initially
	sg_bDischargeActive = true;

	// Turn on interrupt functionality but no specific interrupts
	INT_ENABLE();
	
	// First thing is an init/sensor read
	sg_eCellAction = EACTION_INIT;
	sg_bCellCPULast = true;  // assume we're last in the chain

	// Enable CPU interrupts
	sei();
	
	// Hang on forever!
	while (1)
	{
		uint16_t u16Voltage;
		uint16_t u16Temperature;

		if ((EACTION_SEND_SENSOR_READING == sg_eCellAction) ||
			(EACTION_SEND_PATTERN == sg_eCellAction) || 
			(EACTION_INIT == sg_eCellAction))
		{
			bool bDischargeActive = false;
			
			// Read the voltage first since the MUX is switched to the ADC
			// by default
			u16Voltage = ADCRead();
			sg_u16BatteryVoltage = u16Voltage;
			
			if (IS_PIN_CELL_UP_RX_ASSERTED())  // do a quick detect prior to sensor read, but check again after
			{
				// Pin is asserted, we're NOT the last CPU
				sg_bCellCPULast = false;
			}

			
			// Now temperature
			u16Temperature = MCP9843ReadTemperature();

			// Analog channel is now an input
			SCL_DISABLE();
			
			// Set SCL back to analog input
			ADC_ADMUX_SET();
			
			if (IS_PIN_CELL_UP_RX_ASSERTED())  // do a quick detect prior to possible event set, it may happen on some cells but not others
			{
				// Pin is asserted, we're NOT the last CPU
				sg_bCellCPULast = false;
			}
			
			
			// While we have I2C operational, let's evaluate the discharge
			// state and change it if it's updated
			if (u16Voltage > sg_u16BatteryVoltageTarget)
			{
				bDischargeActive = true;
			}
			
			// If the discharge state has changed, tell the MCP9843 about it
			if (bDischargeActive != sg_bDischargeActive)
			{
				sg_bDischargeActive = bDischargeActive;
				
				// Discharge is active LOW on the EVENT pin
				MCP9843SetEventPin(sg_bDischargeActive ? false : true);
				
				// Analog channel is now an input
				SCL_DISABLE();
			
				// Set SCL back to analog input
				ADC_ADMUX_SET();
			}
			
			// If we're discharging, update the battery voltage message
			if (sg_bDischargeActive)
			{
				// Set the upper bit to indicate we're actively discharging
				u16Voltage |= MSG_CELL_DISCHARGE_ACTIVE;
			}
			
			// If this is set, it will originate (or propagate) a pattern
			if (EACTION_SEND_PATTERN == sg_eCellAction)
			{
				sg_u16BatteryTemperature = PATTERN_TEMPERATURE;
				sg_u16BatteryVoltage = PATTERN_VOLTAGE;
			}
			else
			{
				// We're sending the actual reading
				sg_u16BatteryTemperature = u16Temperature;
				sg_u16BatteryVoltage = u16Voltage;
			}
		
			// If we're just initializing, go back to sleep. We only needed
			// the first pass to set the discharge/charge state
			// do a sample window for last cpu detection to account for any differences in arriving at this point between adjacent cpus due to I2C
			if (EACTION_INIT == sg_eCellAction)
			{
				uint8_t sample_count = 10;  //number of times to sample UP_RX for last cpu detection
				sg_eCellAction = EACTION_NONE;
				
				// sample every 1ms
				while (sample_count--)
				{
				// If at any point we see the pin asserted, we're NOT the last
					if (IS_PIN_CELL_UP_RX_ASSERTED())
					{
						// Pin is asserted, we're NOT the last CPU
						sg_bCellCPULast = false;
					}
					Delay(1000);  //this also gives the cell below more time to detect us
				}

				
				// If we're not the last cell CPU, turn on receive interrupts
				if (false == sg_bCellCPULast)
				{
					// Initialize the UART for receive operation (from cell CPUs to module CPUs)
					vUARTInitReceive();
				}
				else
				{
					// Last cell CPU in the chain - not needed and will just generated spurious
					// interrupts.
				}
		
				
				// Initialize transmit communication for module->cell->cell... communication
				CELL_DN_TX_DEASSERT();				// end of last detect sample window for cell below
				vUARTInitTransmit(); 
			}
			else
			{
				// We're doing a send of the data!
				sg_eCellAction = EACTION_INITIATE_TRANSMIT;
			}
		}
	
		if (EACTION_INITIATE_TRANSMIT == sg_eCellAction)
		{
			sg_eCellAction = EACTION_NONE;
			
			
			// If we're the last CPU, then we initiate transmit, otherwise we
			// don't do anything - we wait for the last CPU to start the message
			// transmission.
			if (true == sg_bCellCPULast)
			{
				// While we're receiving something, don't transmit anything. In practice, this
				// should never loop, but during development, it might.
				while (vUARTIscell_dn_rxActive());
				
				// Start transmission. Note this routine stops and starts interrupts, too.
				vUARTStartcell_dn_tx(VUART_BIT_TICKS/2);
			}
		}
		
		// SLEEP - The CPU will wake up on the next interrupt
//		set_sleep_mode(SLEEP_MODE_IDLE);
//		sleep_mode();
	}
}

// NOTE: If you're getting unexpected AVR resets, uncomment these unused handlers to see
// if another interrupt is being taken that doesn't have a handler. AVR Defaults to jumping
// to 0 when not vector is programmed.

ISR(INT0_vect, ISR_BLOCK)
{
	while (1);
}

ISR(TIMER1_OVF_vect, ISR_BLOCK)
{
	while (1);
}

ISR(TIMER0_OVF_vect, ISR_BLOCK)
{
	while (1);
}

ISR(USI_START_vect, ISR_BLOCK)
{
	while (1);
}

ISR(USI_OVF_vect, ISR_BLOCK)
{
	while (1);
}

ISR(EE_RDY_vect, ISR_BLOCK)
{
	while (1);
}

ISR(ANA_COMP_vect, ISR_BLOCK)
{
	while (1);
}

ISR(INT1_vect, ISR_BLOCK)
{
	while (1);
}

ISR(TIMER0_CAPT_vect, ISR_BLOCK)
{
	while (1);
}

ISR(TIMER1_COMPD_vect, ISR_BLOCK)
{
	while (1);
}

ISR(FAULT_PROTECTION_vect, ISR_BLOCK)
{
	while (1);
}
