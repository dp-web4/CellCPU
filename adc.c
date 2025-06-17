/* CellCPU
 *
 * Cell controller firmware - ADC module
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
#include "adc.h"
#include "main.h"
#include "platform.h"

// Configures PB3 as an analog input and converts/samples it
uint16_t ADCRead(void)
{
	uint16_t u16Reading;
	
    // Set the prescaler to clock/2 & enable ADC
    ADCSRA |= (1 << ADPS0) | (1 << ADEN);

	// Minor delay so we get a reasonable value
	Delay(20);
    
    // Read the ADC in order to get a 10 bit reading of where it is.
    // Start the conversion
    ADCSRA |= (1 << ADSC);

    // Wait for it to finish - blocking
    while (ADCSRA & (1 << ADSC));
	
	// Get the actual reading
	u16Reading = ADC;
	
	// Power down the ADC
	ADCSRA &= ~(1 << ADEN);
    
    return(u16Reading);
}
