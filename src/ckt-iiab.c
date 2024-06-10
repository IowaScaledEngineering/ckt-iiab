/*************************************************************************
Title:    Interlocking-In-A-Box (simple)
Authors:  Michael Petersen <railfan@drgw.net>
File:     CKT-IIAB
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2024 Michael Petersen

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#include <stdlib.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <util/atomic.h>

#include "io.h"
#include "interlocking.h"
#include "debouncer.h"

#define NUM_DIRECTIONS 2
#define OPPOSITE_DIRECTION(d) ((d)%2)?((d)-1):((d)+1)

typedef enum
{
	STATE_IDLE,
	STATE_DELAY,
	STATE_REQUEST,
	STATE_CLEARANCE,
	STATE_TIMEOUT,
	STATE_OCCUPIED,
	STATE_LOCKOUT,
	STATE_CLEARING,
	STATE_RESET,
} InterlockState;

InterlockState state[NUM_DIRECTIONS];

uint8_t timeoutSeconds;
volatile uint16_t timeoutTimer[NUM_DIRECTIONS];  // decisecs

uint8_t lockoutSeconds;
volatile uint16_t lockoutTimer[NUM_DIRECTIONS];  // decisecs


void InterlockingToSignals(void)
{
	uint8_t dir;
	
	for(dir=0; dir<NUM_DIRECTIONS; dir++)
	{
		switch(state[dir])
		{
			case STATE_CLEARANCE:
			case STATE_TIMEOUT:
				if(0 == dir)
					setSignal(APPROACH_A, GREEN);
				else if(1 == dir)
					setSignal(APPROACH_B, GREEN);
				break;
			default:
				// Default to most restrictive aspect
				if(0 == dir)
					setSignal(APPROACH_A, RED);
				else if(1 == dir)
					setSignal(APPROACH_B, RED);
				break;
		}
	}
}


volatile uint32_t millis = 0;

ISR(TIMER0_COMPA_vect) 
{
	millis++;
}

uint32_t getMillis()
{
	uint32_t retmillis;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
	{
		retmillis = millis;
	}

	return retmillis;
}



void init(void)
{
	uint8_t i;
	
	// Kill watchdog
	MCUSR = 0;
	wdt_reset();
	WDTCR = _BV(WDE) | _BV(WDP2) | _BV(WDP1);   // Enable WDT (1s)
	wdt_reset();

	PORTA = 0x3F;  // Pull-ups on PA0 - PA5
	DDRA = _BV(PB7);  // Aux LED output
	PORTB = 0x7F;  // Pull-ups on PB0 - PB2
	setSignal(APPROACH_A, RED);
	setSignal(APPROACH_B, RED);
	DDRB = _BV(PB3) | _BV(PB4) | _BV(PB5) | _BV(PB6);

	TIMSK = 0;                                    // Timer interrupts OFF
	// Set up Timer/Counter0 for 100Hz clock
	TCCR0A = 0b00000001;  // CTC Mode
	TCCR0B = 0b00000011;  // CS01 + CS00 - 1/64 prescale
	OCR0A = 125;           // 8MHz / 64 / 125 = 1kHz
	TIMSK = _BV(OCIE0A);

	sei();
	wdt_reset();

	for(i=0; i<NUM_DIRECTIONS; i++)
	{
		timeoutTimer[i] = 0;
		timeoutSeconds = 1;
		lockoutTimer[i] = 0;
		lockoutSeconds = 1;
	}
}


int main(void)
{
	uint8_t dir;
	uint16_t temp_uint16;

	// Application initialization
	init();

	wdt_reset();

	// Initialization, board check
	setAuxLed();
	_delay_ms(500);
	wdt_reset();
	setSignal(APPROACH_A, GREEN);
	_delay_ms(500);
	wdt_reset();
	setSignal(APPROACH_A, RED);
	_delay_ms(500);
	wdt_reset();
	setSignal(APPROACH_B, GREEN);
	_delay_ms(500);
	wdt_reset();
	setSignal(APPROACH_B, RED);
	_delay_ms(500);
	wdt_reset();
	clearAuxLed();
	
	while(1)
	{
		wdt_reset();
		for(dir=0; dir<NUM_DIRECTIONS; dir++)
		{
			wdt_reset();
			switch(state[dir])
			{
				case STATE_IDLE:
					if( approachBlockOccupancy(dir) && !lockoutTimer[dir] )
					{
						state[dir] = STATE_DELAY;
					}
					break;

				case STATE_DELAY:
					// Do the delay stuff here
					state[dir] = STATE_REQUEST;
					break;

				case STATE_REQUEST:
					if(requestInterlocking(dir))
					{
						// Request for interlocking approved
						state[dir] = STATE_CLEARANCE;
					}
					break;

				case STATE_CLEARANCE:
					if(interlockingBlockOccupancy())
					{
						// Train has entered interlocking, proceed
						state[dir] = STATE_OCCUPIED;
					}
					else if(!approachBlockOccupancy(dir))
					{
						// No occupany in approach block, start timeout
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
						{
							timeoutTimer[dir] = 10 * timeoutSeconds;
						}
						state[dir] = STATE_TIMEOUT;
					}
					// Wait here if no exit conditions met
					break;

				case STATE_TIMEOUT:
					ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
					{
						temp_uint16 = timeoutTimer[dir];
					}

					// Give priority to turnouts, then occupancy, then timeout
					if(interlockingBlockOccupancy())
					{
						// Train has entered interlocking, proceed
						state[dir] = STATE_OCCUPIED;
					}
					else if(approachBlockOccupancy(dir))
					{
						// Approach detector covered again, go back
						state[dir] = STATE_CLEARANCE;
					}
					else if(!temp_uint16)
					{
						// Timed out.  Reset
						state[dir] = STATE_RESET;
					}
					break;

				case STATE_OCCUPIED:
					if(!interlockingBlockOccupancy())
					{
						// Proceed if interlocking block is clear
						state[dir] = STATE_LOCKOUT;
					}
					else if(approachBlockOccupancy(OPPOSITE_DIRECTION(dir)))
					{
						// Opposite approach occupied
						state[dir] = STATE_CLEARING;
					}
					break;

				case STATE_LOCKOUT:
					// Set lockout on opposite approach
					ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
					{
						lockoutTimer[OPPOSITE_DIRECTION(dir)] = 10 * lockoutSeconds;
					}
					state[dir] = STATE_RESET;
					break;

				case STATE_CLEARING:
					if(!approachBlockOccupancy(OPPOSITE_DIRECTION(dir)))
					{
						// Opposite approach cleared
						state[dir] = STATE_RESET;
					}
					break;

				case STATE_RESET:
					clearInterlocking();
					state[dir] = STATE_IDLE;
					break;
			}
		}

		InterlockingToSignals();
		readInputs();
		
		// Blink codes
		uint32_t tempMillis = getMillis();
		uint8_t blinkSeq = (tempMillis % 1000) / 50;
		if(blinkCode[blinkSeq])
			setAuxLed();
		else
			clearAuxLed();
	}
}


