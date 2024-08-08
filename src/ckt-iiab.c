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

#define OPPOSITE_DIRECTION(d) (((d)==APPROACH_A)?APPROACH_B:APPROACH_A)

typedef enum
{
	STATE_IDLE      = 1,
	STATE_DELAY     = 0,
	STATE_REQUEST   = 2,
	STATE_CLEARANCE = 3,
	STATE_TIMEOUT   = 4,
	STATE_OCCUPIED  = 5,
	STATE_LOCKOUT   = 6,
	STATE_CLEARING  = 7,
	STATE_RESET     = 8,
} InterlockState;

uint8_t timeoutSeconds;
volatile uint32_t timeoutTimer;

uint8_t lockoutSeconds;
volatile uint32_t lockoutTimer;

uint8_t delaySeconds;
volatile uint32_t delayTimer;

volatile uint32_t millis = 0;

ISR(TIMER0_COMPA_vect) 
{
	millis++;
	
	if(lockoutTimer)
		lockoutTimer--;
	
	if(timeoutTimer)
		timeoutTimer--;
	
	if(delayTimer)
		delayTimer--;
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
	// Kill watchdog
	MCUSR = 0;
	wdt_reset();
	WDTCR = _BV(WDE) | _BV(WDP2) | _BV(WDP1);   // Enable WDT (1s)
	wdt_reset();

	PORTA = 0x3F;  // Pull-ups on PA0 - PA5
	DDRA = _BV(PB7);  // Aux LED output
	PORTB = 0x7F;  // Drive PB0 - PB3 high.  Pull-ups on PB4 - PB6.
	setSignal(APPROACH_A, RED);
	setSignal(APPROACH_B, RED);
	DDRB = _BV(PB0) | _BV(PB1) | _BV(PB2) | _BV(PB3);

	TIMSK = 0;                                    // Timer interrupts OFF
	// Set up Timer/Counter0 for 100Hz clock
	TCCR0A = 0b00000001;  // CTC Mode
	TCCR0B = 0b00000011;  // CS01 + CS00 - 1/64 prescale
	OCR0A = 125;           // 8MHz / 64 / 125 = 1kHz
	TIMSK = _BV(OCIE0A);

	sei();
	wdt_reset();

	timeoutTimer = 0;
	timeoutSeconds = 10;
	lockoutTimer = 0;
	lockoutSeconds = 20;
	delayTimer = 0;
	delaySeconds = 7;
}


int main(void)
{
	Block dir = NONE;
	uint16_t temp_uint16;
	InterlockState state = STATE_IDLE;
	
	// Application initialization
	init();
	initializeInputOutput();

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
		
		readInputs();
		readDipSwitches();

		wdt_reset();

		switch(state)
		{
			case STATE_IDLE:
				if( approachBlockOccupancy(APPROACH_A) && !lockoutTimer )
				{
					dir = APPROACH_A;
				}
				else if( approachBlockOccupancy(APPROACH_B) && !lockoutTimer )
				{
					dir = APPROACH_B;
				}

				if(NONE != dir)
				{
					ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
					{
						delayTimer = 1000 * delaySeconds;
					}
					state = STATE_DELAY;
				}
				break;

			case STATE_DELAY:
				// Do the delay stuff here
				ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
				{
					temp_uint16 = delayTimer;
				}

				if(!temp_uint16)
				{
					// Delay expired.  Continue.
					state = STATE_REQUEST;
				}
				break;

			case STATE_REQUEST:
				if(requestInterlocking(dir))
				{
					// Request for interlocking approved
					state = STATE_CLEARANCE;
				}
				break;

			case STATE_CLEARANCE:
				if(interlockingBlockOccupancy())
				{
					// Train has entered interlocking, proceed
					state = STATE_OCCUPIED;
				}
				else if(!approachBlockOccupancy(dir))
				{
					// No occupany in approach block, start timeout
					ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
					{
						timeoutTimer = 1000 * timeoutSeconds;
					}
					state = STATE_TIMEOUT;
				}
				// Wait here if no exit conditions met
				break;

			case STATE_TIMEOUT:
				ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
				{
					temp_uint16 = timeoutTimer;
				}

				// Give priority to turnouts, then occupancy, then timeout
				if(interlockingBlockOccupancy())
				{
					// Train has entered interlocking, proceed
					state = STATE_OCCUPIED;
				}
				else if(approachBlockOccupancy(dir))
				{
					// Approach detector covered again, go back
					state = STATE_CLEARANCE;
				}
				else if(!temp_uint16)
				{
					// Timed out.  Reset
					state = STATE_RESET;
				}
				break;

			case STATE_OCCUPIED:
				if(!interlockingBlockOccupancy())
				{
					// Interlocking block is clear, start lockout timer
					ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
					{
						lockoutTimer = 1000 * lockoutSeconds;
					}
					state = STATE_LOCKOUT;
				}
				else if(approachBlockOccupancy(OPPOSITE_DIRECTION(dir)))
				{
					// Opposite approach occupied
					state = STATE_CLEARING;
				}
				break;

			case STATE_LOCKOUT:
				ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
				{
					temp_uint16 = lockoutTimer;
				}

				if(!temp_uint16)
				{
					// Timed out.  Reset
					state = STATE_RESET;
				}
				break;

			case STATE_CLEARING:
				if(!approachBlockOccupancy(OPPOSITE_DIRECTION(dir)))
				{
					// Opposite approach cleared
					state = STATE_RESET;
				}
				break;

			case STATE_RESET:
				clearInterlocking();
				dir = NONE;
				state = STATE_IDLE;
				break;
		}

		// Set Signals
		switch(state)
		{
			case STATE_CLEARANCE:
			case STATE_TIMEOUT:
				if(APPROACH_A == dir)
					setSignal(APPROACH_A, GREEN);
				else if(APPROACH_B == dir)
					setSignal(APPROACH_B, GREEN);
				break;
			default:
				// Default to most restrictive aspect
				setSignal(APPROACH_A, RED);
				setSignal(APPROACH_B, RED);
				break;
		}

		wdt_reset();

		// Blink codes
		uint32_t tempMillis = getMillis();
		uint8_t blinkSeq;
		if(STATE_DELAY == state)
		{
			if(!(tempMillis % 500))
			{
				if(!(tempMillis % 1000))
					setAuxLed();
				else
					clearAuxLed();
			}
		}
		else
		{
			blinkSeq = (tempMillis % 4000) / 100;  // Create 20 states (0-19)
			if(blinkSeq < state*4)
			{
				if(blinkSeq % 4)
					clearAuxLed();
				else
					setAuxLed();
			}
			else
				clearAuxLed();
		}
	}
}


