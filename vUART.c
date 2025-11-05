/* CellCPU
 *
 * Cell controller firmware - Virtual UART module
 *
 * (C) Copyright 2023-2024 Modular Battery Technologies, Inc.
 * US Patents 11,380,942; 11,469,470; 11,575,270; others. All
 * rights reserved
 *
 * This module contains the "virtual UART" functionality, implemented with a
 * combination of bit banging, interrupts, and timers. Naming conventions are
 * from the perspective of the pack controller:
 *
 * cell_up_rx (PB4) -> cell_dn_tx (PB5) (upstream) - Receives/relays cell status, to be sent to the pack controller
 * cell_dn_rx (PB1) -> cell_up_tx (PB0) (downstream) - Receives/relays messages from the pack controller 
 *
 * For each byte received, the format is as follows:
 *
 * 1 Start bit
 * 8 Data bits
 * 1 Stop bit (asserted=more coming, deasserted=nothing coming)
 * 1 Guard bit (always deasserted so there is transition time between bits)
 * 
 * The stop bit has a unique use, in that if it's a 1, it means more data is coming.
 * If it's a 0, there's no more data, leaving open the opportunity for the 
 * current controller to append more data.
 *
 * Timer0 is used for both upstream and downstream activity. Upstream is
 * handled by compare A, and downstream compare B. Timer0 is set to free run
 * and is NOT reset by a match to either channel.
 *
 * Reception of a byte is documented as follows:
 * 
 * 1) RX Line has a falling edge, causes a GPIO interrupt
 * 2) GPIO RX Pin is masked for interrupt operation
 * 3) Timer is set to TCNT0 + VUART_BIT_TICKS + (VUART_BIT_TICKS / 2) to set up
 * 4) When a timer/counter interrupt expires, sample RX pin
 * 5) Set equivalent TX pin to the same value (to pass it through)
 * 6) Timer is set to TCNT0 + VUART_BIT_TICKS, repeat until all 8 bits are received
 * 7) On the 9th bit, sample the RX pin. If it's 1, more data is coming, go back to step
 *    1. If it's 0, continue on to step 9.
 * 9) Deassert cell_dn_tx. Wait 1 bit time, advance to 10.
 * 10) Transmit/append local battery data.
 */

#include "Platform.h"
#include "vUART.h"
#include "main.h"
#include "../ModuleCPU/Shared/Shared.h"

// States for the receive state machine
typedef enum
{
	ESTATE_IDLE,				// Bus is completely idle and nothing is running
	ESTATE_RX_DATA,				// Bus is receiving data from cell_up_rx/cell_dn_rx
	ESTATE_TX_DATA,				// Bus is transmitting data to cell_dn_tx/cell_up_tx
	ESTATE_NEXT_BYTE,			// Bus is active (transmitting or receiving) but in between bytes and more data coming
} EChannelState;

// States of each channel

// cell_up_rx related
static uint8_t sg_u8Cell_up_rxBitCount;
static bool sg_bcell_up_rxPriorState;
static volatile EChannelState sg_ecell_up_rxState = ESTATE_IDLE;
static bool sg_bCell_up_rxMoreData;
static bool sg_bcell_up_rx_Enabled;

// cell_dn_rx related
static uint8_t sg_u8Cell_dn_rxBitCount;
static bool sg_bcell_dn_rxPriorState;
static volatile EChannelState sg_ecell_dn_rxState = ESTATE_IDLE;
static bool sg_bCell_dn_rxMoreData;

// cell_dn_tx related
static bool sg_bdn_txNextBit;
static uint8_t sg_u8dn_txBitCount;
static uint8_t sg_u8dn_txDataByte;
static bool sg_bdn_txMoreAvailable;

// Pin change interrupt - detecting start bit
ISR(PCINT_VECTOR, ISR_BLOCK)
{
	bool bCellUpRXAsserted = IS_PIN_CELL_UP_RX_ASSERTED();
	bool bCellDnRxAsserted = IS_PIN_CELL_DN_RX_ASSERTED();

	// If we have timers we need to start, start them at the top of the procedure	
	// so the sample times are tighter/more consistent
	if (bCellUpRXAsserted && sg_bcell_up_rx_Enabled &&
		((ESTATE_IDLE == sg_ecell_up_rxState) ||
		 (ESTATE_NEXT_BYTE == sg_ecell_up_rxState)))
	{
		// This causes a sampling in the middle of the waveform
		// and accounts for code overhead.
		TIMER_CHA_INT(VUART_BIT_TICKS + VUART_SAMPLE_OFFSET);

		// Stop cell_up_rx interrupts
		INT_CELL_UP_RX_DISABLE();
		
		// We are now receiving data
		sg_ecell_up_rxState = ESTATE_RX_DATA;
		sg_bcell_up_rxPriorState = true;
		sg_u8Cell_up_rxBitCount = 0;
	}
	
	// Handle cell_dn_rx incomings
	if ((bCellDnRxAsserted) &&
		((ESTATE_IDLE == sg_ecell_dn_rxState) ||
		 (ESTATE_NEXT_BYTE == sg_ecell_dn_rxState)))
	{
		// This causes sampling closer to the middle of the waveform
		// and accounts for code overhead.
		TIMER_CHB_INT(VUART_BIT_TICKS + VUART_SAMPLE_OFFSET);

		// Stop cell_dn_rx interrupts
		INT_CELL_DN_RX_DISABLE();
		
		// Only call the data start routine when it's the actual start of the initial
		// byte, not subsequent bytes.
		if (ESTATE_IDLE == sg_ecell_dn_rxState)
		{
			// Falling edge on cell_dn_rx
			Celldn_rxDataStart();
		}
		
		// Set the RX data state
		sg_ecell_dn_rxState = ESTATE_RX_DATA;
		sg_bcell_dn_rxPriorState = true;
		sg_u8Cell_dn_rxBitCount = 0;
	}
}

// Timer 0 compare A interrupt (bit clock) for cell_up_rx
ISR(TIMER_COMPA_VECTOR, ISR_BLOCK)
{
	// Schedule next interrupt based on state (RX needs overhead compensation, TX doesn't)
	if (ESTATE_RX_DATA == sg_ecell_up_rxState)
	{
		TIMER_CHA_INT(VUART_BIT_TICKS - VUART_RX_ISR_OVERHEAD);  // RX: compensate for ISR overhead
	}
	else if (ESTATE_TX_DATA == sg_ecell_up_rxState)
	{
		TIMER_CHA_INT(VUART_BIT_TICKS - VUART_TX_ISR_OVERHEAD);  // TX: exact timing
	}

	if (ESTATE_RX_DATA == sg_ecell_up_rxState)
	{
		// Set the bit value for what the prior state was
		if (sg_bcell_up_rxPriorState)
		{
			CELL_DN_TX_ASSERT();
		}
		else
		{
			CELL_DN_TX_DEASSERT();
		}
		
		sg_bcell_up_rxPriorState = IS_PIN_CELL_UP_RX_ASSERTED();
	
		sg_u8Cell_up_rxBitCount++;
		
		// Handles incoming start bit and data bits
		if (sg_u8Cell_up_rxBitCount < 9)
		{
			// Data bits
			return;
		}
		else
		if (9 == sg_u8Cell_up_rxBitCount)
		{
			// This is the more data vs. data stop bit
			sg_bCell_up_rxMoreData = sg_bcell_up_rxPriorState;
			
			// Always ensure that we're signaling more data since the termination
			// bit is done in the transmit phase
			sg_bcell_up_rxPriorState = true;
			return;
		}
		else
		if (10 == sg_u8Cell_up_rxBitCount)
		{
			// Deassert the dn_tx signal (guard bit)
			sg_bcell_up_rxPriorState = false;
			return;
		}
		else
		if (11 == sg_u8Cell_up_rxBitCount)
		{
			// Only way to get here is if we have more data. We are now at the start of a start
			// cycle now and we can reenable interrupts on cell_up_rx so we wait for the
			// start of the next byte.
			TIMER_CHA_INT_DISABLE();
			
			// Enable cell_up_rx interrupts
			INT_CELL_UP_RX_ENABLE();
				
			if (sg_bCell_up_rxMoreData)
			{
				// Flag that more data is coming
				sg_ecell_up_rxState = ESTATE_NEXT_BYTE;
			}
			else
			{
				// Bus is now idle		
				sg_ecell_up_rxState = ESTATE_IDLE;
				
				// Start transmission of our data
				(void) vUARTStartcell_dn_tx(VUART_BIT_TICKS*3);
			}
		}
		
		return;
	} 
	
	// This handles the transmission of data when this CPU originates it
	if (ESTATE_TX_DATA == sg_ecell_up_rxState)
	{
		// Set the state of the output pin
		if (sg_bdn_txNextBit)
		{
			CELL_DN_TX_ASSERT();
		}
		else
		{
			CELL_DN_TX_DEASSERT();
		}
		
		// Preincrement the bit count 
		sg_u8dn_txBitCount++;

		// Transmit start condition and prepare data byte
		// If this is the first bit, fetch the byte
		if( sg_u8dn_txBitCount < 9 )
		{
			// Transmit data! (msb first)
			if (sg_u8dn_txDataByte & 0x80)
			{
				sg_bdn_txNextBit = true;
			}
			else
			{
				sg_bdn_txNextBit = false;
			}
			
			sg_u8dn_txDataByte <<= 1;
			return;
		}
		// Transmit stop bit (stop or continue!)
		else 
		if (9 == sg_u8dn_txBitCount)
		{
			sg_bdn_txNextBit = sg_bdn_txMoreAvailable;
			return;
		}
		else 
		if (10 == sg_u8dn_txBitCount)
		{
			// Guard bit
			sg_bdn_txNextBit = false;
			return;
		}
		else
		if (11 == sg_u8dn_txBitCount)
		{
			// Already deasserted here
			sg_u8dn_txBitCount = 0;

			TIMER_CHA_INT_DISABLE();
				
			// If more available, reset the bit count and exit
			if (false == sg_bdn_txMoreAvailable)
			{
				sg_ecell_up_rxState = ESTATE_IDLE;

				// Terminate! stop the timer interrupts and reenable port pin interrupts
				CELL_DN_TX_DEASSERT();
			
				// Allow reception of the cell_up_rx path again
				INT_CELL_UP_RX_ENABLE();
				
				// Bail out - don't continue to process anything since it's idle
				CELL_UP_TX_DEASSERT();
				return;
			}
			else
			{
				// Set the timer 2 bits later - this can be sloppy since it's in between bytes
				TIMER_CHA_INT(VUART_BIT_TICKS*4);

				// Get the next data byte and whether or not
				sg_u8dn_txDataByte = Celldn_txDataGet();
				sg_bdn_txMoreAvailable = Celldn_txDataAvailable();
				sg_bdn_txNextBit = true;
			}
		}

		return;
	}
	
	// Stop the timer - not necessary
	TIMER_CHA_INT_DISABLE();
}

// Timer 0 compare B interrupt (bit clock) for cell_dn_rx
ISR(TIMER_COMPB_VECTOR, ISR_BLOCK)
{
	bool bData;

	// This path is always RX (cell_dn_rx -> cell_up_tx relay), so use RX overhead compensation
	TIMER_CHB_INT(VUART_BIT_TICKS - VUART_RX_ISR_OVERHEAD);
		
	// Set the bit value for what the prior state was
	if (sg_bcell_dn_rxPriorState)
	{
		CELL_UP_TX_ASSERT();
	}
	else
	{
		CELL_UP_TX_DEASSERT();
	}

	bData = sg_bcell_dn_rxPriorState;
	sg_bcell_dn_rxPriorState = IS_PIN_CELL_DN_RX_ASSERTED();
	
	sg_u8Cell_dn_rxBitCount++;
	
	// Handles cell_dn_rx
	if (1 == sg_u8Cell_dn_rxBitCount)
	{
		// Start bit
	}
	else
	if (sg_u8Cell_dn_rxBitCount < 10)
	{
		Celldn_rxDataBit(bData);
		return;
	}
	else
	if (10 == sg_u8Cell_dn_rxBitCount)
	{
		// This is the more data vs. data stop bit
		sg_bCell_dn_rxMoreData = sg_bcell_dn_rxPriorState;
		return;
	}
	else
	if (11 == sg_u8Cell_dn_rxBitCount)
	{
		// No longer asserted
		sg_bcell_dn_rxPriorState = false;
		return;
	}
	else
	if (12 == sg_u8Cell_dn_rxBitCount)
	{
		TIMER_CHB_INT_DISABLE();
		
		// Enable cell_dn_rx for next byte
		INT_CELL_DN_RX_ENABLE();
		
		if (false == sg_bCell_dn_rxMoreData)
		{
			// Bus is now idle
			sg_ecell_dn_rxState = ESTATE_IDLE;
		}
		else
		{
			// Flag that more data is coming
			sg_ecell_dn_rxState = ESTATE_NEXT_BYTE;
		}

		return;
	}
}

// This starts a transmission on cell_dn_tx. true Is returned if it 
// was successfully started, but false if the cell_dn_tx vUART is active.
bool vUARTStartcell_dn_tx(uint8_t u8StartDelayTicks)
{
	bool bReturnCode = false;

	// Is our cell_up_rx->cell_dn_tx path busy? If so, we can't start
	if (ESTATE_IDLE == sg_ecell_up_rxState)
	{
		// Shut off cell_up_rx interrupts coming from up stream so we don't hit a
		// spurious interrupt while we're transmitting
		INT_CELL_UP_RX_DISABLE();

		bReturnCode = true;
		
		// Set the state machine to transmit data.
		sg_ecell_up_rxState = ESTATE_TX_DATA;

		// Ensure the first bit is a start bit		
		sg_bdn_txNextBit = true;

		// Let the consumer code know that a transmission is starting
		Celldn_txDataReset();
		
		// Seed any data to transmit
		sg_u8dn_txDataByte = Celldn_txDataGet();
		sg_bdn_txMoreAvailable = Celldn_txDataAvailable();

		// Clear our dn_tx bit count
		sg_u8dn_txBitCount = 0;
		
		// Start up timer A to fire a half a bit later (if it's sloppy, that's OK)
		TIMER_CHA_INT(u8StartDelayTicks);		
	}
	
	return(bReturnCode);
}

void vUARTPinInit(void)
{
	// Set up cell_dn_rx
	PIN_CELL_DN_RX_DDR &= ((uint8_t) ~(1 << PIN_CELL_DN_RX));	// Set as input
#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__)
	PIN_CELL_DN_RX_PORT &= ((uint8_t) ~(1 << PIN_CELL_DN_RX));	// Turn off pullup
#elif defined(__AVR_ATtiny261A__)
	PIN_CELL_DN_RX_PORT |= ((uint8_t) (1 << PIN_CELL_DN_RX));	// Turn on pullup
#else
#error No target processor defined or target processor type unknown
#endif
		
	// Set up cell_up_tx
	PIN_CELL_UP_TX_DDR |= (1 << PIN_CELL_UP_TX);	// Set as output
#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__)
	PIN_CELL_UP_TX_PORT &= ((uint8_t) ~(1 << PIN_CELL_UP_TX));	// Turn off pullup
#elif defined(__AVR_ATtiny261A__)
	PIN_CELL_UP_TX_PORT |= ((uint8_t) (1 << PIN_CELL_UP_TX));	// Turn on pullup
#else
#error No target processor defined or target processor type unknown
#endif
	CELL_UP_TX_DEASSERT();							// Deassert cell_up_tx
	
	// Set up cell_up_rx
	PIN_CELL_UP_RX_DDR &= ((uint8_t) ~(1 << PIN_CELL_UP_RX));	// Set as input
#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__)
	PIN_CELL_UP_RX_PORT &= ((uint8_t) ~(1 << PIN_CELL_UP_RX));	// Turn off pullup
#elif defined(__AVR_ATtiny261A__)
	PIN_CELL_UP_RX_PORT |= ((uint8_t) (1 << PIN_CELL_UP_RX));	// Turn on pullup
#else
#error No target processor defined or target processor type unknown
#endif
	
	// Set up cell_dn_tx
	PIN_CELL_DN_TX_DDR |= (1 << PIN_CELL_DN_TX);	// Set as output
#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__)
	PIN_CELL_DN_TX_PORT &= ((uint8_t) ~(1 << PIN_CELL_DN_TX));	// Turn off pullup
#elif defined(__AVR_ATtiny261A__)
	PIN_CELL_DN_TX_PORT |= ((uint8_t) (1 << PIN_CELL_DN_TX));	// Turn on pullup
#else
#error No target processor defined or target processor type unknown
#endif

	CELL_DN_TX_DEASSERT();							// Deassert cell_dn_tx

	// Fire up the profiler (if available)
	PROFILER_INIT();
}

bool vUARTIscell_dn_rxActive(void)
{
	if (ESTATE_IDLE == sg_ecell_dn_rxState)
	{
		return(false);
	}
	else
	{
		return(true);
	}
}

// Enables transmit communication from module CPU to cell CPU
void vUARTInitTransmit(void)
{
	// Fire up the profiler (if available)
	PROFILER_INIT();

	// Cell_dn_rx to allow receives. We do not need cell_up_tx interrupts
	INT_CELL_DN_RX_ENABLE();
}

// Enables receive communication from cell CPUs to module CPU
void vUARTInitReceive(void)
{
	sg_bcell_up_rx_Enabled = true;
	
	// Set up receives in the cell_up_tx direction
	INT_CELL_UP_RX_ENABLE();
	
	// Ready to rock!	
}