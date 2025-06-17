/* CellCPU
 *
 * Cell controller firmware - Platform specific definitions (mini-HAL)
 *
 * (C) Copyright 2023-2024 Modular Battery Technologies, Inc.
 * US Patents 11,380,942; 11,469,470; 11,575,270; others. All
 * rights reserved
 */

#ifndef _PLATFORM_H_
#define _PLATFORM_H_

// Common to all processors
#include <stdint.h>
#include <xc.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>

/*
 * Cell CPU definitions:
 *
 * PB0 - cell up tx
 * PB1 - cell dn rx
 * PB2 - I2C SDA
 * PB3 - I2C SDL/ADC1 (sense cell plus voltage through resistor divider)
 * PB4 - cell up rx
 * PB5 - cell dn tx (note: This has the net effect of inverting the signal between CPUs)
 *
 * Discharge is on the MPC9843T on the "event" pin. MCP9843T is at I2C addresses (A0-A2 are low):
 *
 * Temperature sensor		- 0x30
 * EEPROM					- 0xa0
 * EEPROM Write protect	-	- 0x60
 */

// Specific CPU defines

#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__)

// *************************************************************************************************
// ATTINY25/45 defines
// *************************************************************************************************

#define CPU_CLOCK_INIT()					CLKPR = (1 << CLKPCE); CLKPR = 0; PCMSK = 0; PRR = 0;
#define TIMER_INIT()						TCCR0A = 0; TCCR0B = (1 << CS01); TCNT0 = 0;	// /8 On main CPU
#define TIMER_COUNTER()						TCNT0
#define TIMER_OVF_VECTOR					TIM0_OVF_vect
#define PCINT_VECTOR						PCINT0_vect
#define TIMER_COMPA_VECTOR					TIM0_COMPA_vect
#define TIMER_COMPB_VECTOR					TIM0_COMPB_vect

// ADC Input related
#define ADC_ADMUX_SET()						ADMUX = (1 << MUX1) | (1 << MUX0) | (1 << REFS1)		// ADC3, internal 1.1V reference

#define	INT_ENABLE()						GIMSK = (1 << PCIE); MCUCR = (MCUCR & ~3) | (1 << ISC00);	// Set to logical pin change interrupt - ensure INT0 isn't active

#define PIN_CELL_DN_RX						PB1
#define PIN_CELL_DN_RX_PORT					PORTB
#define PIN_CELL_DN_RX_DDR					DDRB
#define PIN_CELL_DN_RX_PIN					PINB
#define PIN_CELL_DN_RX_INT					PCINT1

#define PIN_CELL_UP_TX						PB0
#define PIN_CELL_UP_TX_PORT					PORTB
#define	PIN_CELL_UP_TX_DDR					DDRB
#define	PIN_CELL_UP_TX_PIN					PINB

#define PIN_CELL_UP_RX						PB4
#define	PIN_CELL_UP_RX_PORT					PORTB
#define PIN_CELL_UP_RX_DDR					DDRB
#define PIN_CELL_UP_RX_PIN					PINB
#define PIN_CELL_UP_RX_INT					PCINT4

#define PIN_CELL_DN_TX						PB5
#define PIN_CELL_DN_TX_PORT					PORTB
#define PIN_CELL_DN_TX_DDR					DDRB
#define PIN_CELL_DN_TX_PIN					PINB

// CELL_UP_TX macros
#define CELL_UP_TX_ASSERT()					PIN_CELL_UP_TX_PORT |= ((uint8_t) (1 << PIN_CELL_UP_TX))
#define CELL_UP_TX_DEASSERT()				PIN_CELL_UP_TX_PORT &= ((uint8_t) ~(1 << PIN_CELL_UP_TX))

// CELL_DN_TX macros
#define CELL_DN_TX_ASSERT()					PIN_CELL_DN_TX_PORT &= ((uint8_t) ~(1 << PIN_CELL_DN_TX));
#define CELL_DN_TX_DEASSERT()				PIN_CELL_DN_TX_PORT |= ((uint8_t) (1 << PIN_CELL_DN_TX));

// Receive macros
#define	IS_PIN_CELL_DN_RX_ASSERTED()		(0 == IS_PIN_CELL_DN_RX_DEASSERTED())
#define IS_PIN_CELL_DN_RX_DEASSERTED()		((1 << PIN_CELL_DN_RX) & PIN_CELL_DN_RX_PIN)

#define IS_PIN_CELL_UP_RX_ASSERTED()		((1 << PIN_CELL_UP_RX) & PIN_CELL_UP_RX_PIN)
#define IS_PIN_CELL_UP_RX_DEASSERTED()		(0 == IS_PIN_CELL_UP_RX_ASSERTED())

// External interrupt enables
#define INT_CELL_DN_RX_ENABLE()				PCMSK |= (1 << PIN_CELL_DN_RX_INT)	// PCINT1
#define INT_CELL_DN_RX_DISABLE()			PCMSK &= (uint8_t) ~(1 << PIN_CELL_DN_RX_INT)

#define INT_CELL_UP_RX_ENABLE()				PCMSK |= (1 << PIN_CELL_UP_RX_INT)	// PCINT4
#define INT_CELL_UP_RX_DISABLE()			PCMSK &= (uint8_t) ~(1 << PIN_CELL_UP_RX_INT)

#define TIMER_CHA_INT(x)					OCR0A = (uint8_t) (TIMER_COUNTER() + (x)); TIMER_CHA_INT_CLEAR(); TIMSK |= (1 << OCIE0A)
#define TIMER_CHA_INT_CLEAR()				TIFR = (1 << OCF0A);
#define TIMER_CHB_INT(x)					OCR0B = (uint8_t) (TIMER_COUNTER() + (x)); TIMER_CHB_INT_CLEAR(); TIMSK |= (1 << OCIE0B)
#define TIMER_CHB_INT_CLEAR()				TIFR = (1 << OCF0B);
#define TIMER_CHA_INT_DISABLE()				TIMSK &= (uint8_t) ~(1 << OCIE0A)
#define TIMER_CHB_INT_DISABLE()				TIMSK &= (uint8_t) ~(1 << OCIE0B)

// Profiling bits (nothing on this CPU since there are no free pins for it)

#define PROFILER_INIT()
#define PROF_1_ASSERT()
#define PROF_1_DEASSERT()
#define PROF_2_ASSERT()
#define PROF_2_DEASSERT()

// I2C Port
#define I2C_PORT							PORTB
#define I2C_PORT_READ						PINB
#define I2C_PORT_DDR						DDRB

// I2C Pins on that port
#define I2C_SDA_PIN							PORTB2
#define I2C_SCL_PIN							PORTB3

// WDT Defines (for 1hz periodic timer callback)
#define WDT_INIT							(1 << WDE) | (1 << WDCE) | (1 << WDIE) | WDT_TIMEOUT;
#define	DISABLE_WDT()						WDTCR |= (1 << WDCE) | (1 << WDE); WDTCR = 0;
#define ENABLE_1HZ_WDT()					WDTCR |= (1 << WDIE); WDTCR = WDT_INIT;

#elif defined(__AVR_ATtiny261A__)

// *************************************************************************************************
// ATTINY261A defines - Used on the Beavis board
// *************************************************************************************************

// Internal 8Mhz clock 
#define CPU_CLOCK_INIT()					CLKPR = (1 << CLKPCE); CLKPR = 0; PRR = 0; PCMSK0 = 0; PCMSK1 = 0;
#define TIMER_INIT()						TCCR0A = 0; TCCR0B = (1 << CS01); TCNT0L = 0; TCNT0H = 0;  
#define TIMER_COUNTER()						TCNT0L
#define TIMER_OVF_VECTOR					TIMER0_OVF_vect
#define PCINT_VECTOR						PCINT_vect
#define TIMER_COMPA_VECTOR					TIMER0_COMPA_vect
#define TIMER_COMPB_VECTOR					TIMER0_COMPB_vect

// ADC Input related
#define ADC_ADMUX_SET()						ADMUX = 0		// ADC0, VCC reference

// Initializes the interrupt subsystem to receive pin change interrupts
// but does not enable interrupts on any specific pin
#define	INT_ENABLE()						GIMSK |= (1 << PCIE0) | (1 << PCIE1); MCUCR = (MCUCR & ~3) | (1 << ISC00);

#define PIN_CELL_DN_RX						PORTA6		// PA6/PCINT6
#define PIN_CELL_DN_RX_PORT					PORTA
#define PIN_CELL_DN_RX_DDR					DDRA
#define PIN_CELL_DN_RX_PIN					PINA

#define PIN_CELL_UP_TX						PORTA7		// PA7/ADC6/AIN1
#define PIN_CELL_UP_TX_PORT					PORTA
#define	PIN_CELL_UP_TX_DDR					DDRA
#define	PIN_CELL_UP_TX_PIN					PINA

#define PIN_CELL_UP_RX						PORTB4		// XTAL2/CLKO/ADC2/OC1B/PCINT4
#define	PIN_CELL_UP_RX_PORT					PORTB
#define PIN_CELL_UP_RX_DDR					DDRB
#define PIN_CELL_UP_RX_PIN					PINB

#define PIN_CELL_DN_TX						PORTB5		// RESET/dW/ADC0/PCINT5
#define PIN_CELL_DN_TX_PORT					PORTB
#define PIN_CELL_DN_TX_DDR					DDRB
#define PIN_CELL_DN_TX_PIN					PINB

// CELL_UP_TX macros
#define CELL_UP_TX_ASSERT()					PIN_CELL_UP_TX_PORT |= ((uint8_t) (1 << PIN_CELL_UP_TX))
#define CELL_UP_TX_DEASSERT()				PIN_CELL_UP_TX_PORT &= ((uint8_t) ~(1 << PIN_CELL_UP_TX))

// CELL_DN_TX macros
#define CELL_DN_TX_ASSERT()					PIN_CELL_DN_TX_PORT &= ((uint8_t) ~(1 << PIN_CELL_DN_TX));
#define CELL_DN_TX_DEASSERT()				PIN_CELL_DN_TX_PORT |= ((uint8_t) (1 << PIN_CELL_DN_TX));

// Receive macros
#define	IS_PIN_CELL_DN_RX_ASSERTED()		(0 == IS_PIN_CELL_DN_RX_DEASSERTED())
#define IS_PIN_CELL_DN_RX_DEASSERTED()		((1 << PIN_CELL_DN_RX) & PIN_CELL_DN_RX_PIN)

#define IS_PIN_CELL_UP_RX_ASSERTED()		((1 << PIN_CELL_UP_RX) & PIN_CELL_UP_RX_PIN)
#define IS_PIN_CELL_UP_RX_DEASSERTED()		(0 == IS_PIN_CELL_UP_RX_ASSERTED())

// External interrupt enables
#define INT_CELL_DN_RX_ENABLE()				PCMSK0 |= (1 << PIN_CELL_DN_RX)	// PCINT6
#define INT_CELL_DN_RX_DISABLE()			PCMSK0 &= (uint8_t) ~(1 << PIN_CELL_DN_RX)

#define INT_CELL_UP_RX_ENABLE()				PCMSK1 |= (1 << PIN_CELL_UP_RX) // PCINT12
#define INT_CELL_UP_RX_DISABLE()			PCMSK1 &= (uint8_t) ~(1 << PIN_CELL_UP_RX)

#define TIMER_CHA_INT(x)					OCR0A = (uint8_t) (TIMER_COUNTER() + (x)); TIMER_CHA_INT_CLEAR(); TIMSK |= (1 << OCIE0A)
#define TIMER_CHA_INT_CLEAR()				TIFR = (1 << OCF0A);
#define TIMER_CHB_INT(x)					OCR0B = (uint8_t) (TIMER_COUNTER() + (x)); TIMER_CHB_INT_CLEAR(); TIMSK |= (1 << OCIE0B)
#define TIMER_CHB_INT_CLEAR()				TIFR = (1 << OCF0B);
#define TIMER_CHA_INT_DISABLE()				TIMSK &= (uint8_t) ~(1 << OCIE0A)
#define TIMER_CHB_INT_DISABLE()				TIMSK &= (uint8_t) ~(1 << OCIE0B)

// Profiling bits

//#define PROFILER_INIT()					DDRA |= (1 << PORTA1) | (1 << PORTA2); PORTA |=(1 << PORTA1) | (1 << PORTA2);
//#define PROF_1_ASSERT()					PORTA |= (1 << PORTA1)
//#define PROF_1_DEASSERT()					PORTA &= (uint8_t) ~(1 << PORTA1)
//#define PROF_2_ASSERT()					PORTA |= (1 << PORTA2)
//#define PROF_2_DEASSERT()					PORTA &= (uint8_t) ~(1 << PORTA2)

#define PROFILER_INIT()
#define PROF_1_ASSERT()
#define PROF_1_DEASSERT()
#define PROF_2_ASSERT()
#define PROF_2_DEASSERT()

// I2C Port
#define I2C_PORT							PORTB
#define I2C_PORT_READ						PINB
#define I2C_PORT_DDR						DDRB

// I2C Pins on that port
#define I2C_SDA_PIN							PORTB6
#define I2C_SCL_PIN							PORTB3

// WDT Defines (for 1hz periodic timer callback)
#define WDT_INIT							(1 << WDE) | (1 << WDCE) | (1 << WDIE) | WDT_TIMEOUT;
#define	DISABLE_WDT()						WDTCR |= (1 << WDCE) | (1 << WDE); WDTCR = 0;
#define ENABLE_1HZ_WDT()					WDTCR |= (1 << WDIE); WDTCR = WDT_INIT;

#else
#error No target processor defined or target processor type unknown
#endif

// Macros for controlling/reading the I2C data lines
#define SCL_LOW()				(I2C_PORT &= (uint8_t) ~(1 << I2C_SCL_PIN))				// Logic 0
#define SCL_HIGH()				(I2C_PORT |= (uint8_t) (1 << I2C_SCL_PIN))				// Logic 1
#define	SCL_READ()				((I2C_PORT_READ & (1 << I2C_SCL_PIN)) ? true : false)
#define SCL_SET_OUTPUT()		(I2C_PORT_DDR |= (1 << I2C_SCL_PIN))
#define	SCL_SET_INPUT()			(I2C_PORT_DDR &= (uint8_t) ~(1 << I2C_SCL_PIN)); I2C_PORT &= (uint8_t) ~(1 << I2C_SCL_PIN)
#define SDA_LOW()				(I2C_PORT &= (uint8_t) ~(1 << I2C_SDA_PIN))				// Logic 0
#define SDA_HIGH()				(I2C_PORT |= (uint8_t) (1 << I2C_SDA_PIN))				// Logic 1
#define	SDA_READ()				((I2C_PORT_READ & (1 << I2C_SDA_PIN)) ? true : false)
#define SDA_SET_OUTPUT()		(I2C_PORT_DDR |= (1 << I2C_SDA_PIN));
#define SDA_SET_INPUT()			(I2C_PORT_DDR &= ((uint8_t) ~(1 << I2C_SDA_PIN))); SDA_HIGH()
#define SCL_DISABLE()			SCL_SET_INPUT(); SCL_LOW()


#endif