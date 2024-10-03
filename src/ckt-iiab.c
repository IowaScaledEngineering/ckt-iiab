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
#include <avr/interrupt.h>

#include "io.h"
#include "interlocking.h"
#include "debouncer.h"
#include "signalHead.h"

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

SignalState_t signalA;
SignalState_t signalB;
volatile uint8_t signalHeadOptions;

// Signal Port Connections
// These are in the order of:
//  Red address, bitmask
//  Yellow address, bitmask
//  Green address, bitmask

#define SIGNAL_HEAD_A_DEF   &PORTB, _BV(PB0), &PORTB, 0, &PORTB, _BV(PB1)
#define SIGNAL_HEAD_B_DEF   &PORTB, _BV(PB2), &PORTB, 0, &PORTB, _BV(PB3)

ISR(TIMER0_COMPA_vect) 
{
	static uint8_t flasherCounter = 0;
	static uint8_t flasher = 0;
	static uint8_t pwmPhase = 0;
	static uint8_t subMillisCounter = 0;
	
	// The ISR does two main things - updates the LED outputs since
	//  PWM is done through software, and updates millis which is used
	//  to trigger various events
	// We need this to run at roughly 125 Hz * number of PWM levels (32).  That makes a nice round 4kHz
	
	// First thing, output the signals so that the PWM doesn't get too much jitter

	signalHeadISR_OutputPWM(&signalA, signalHeadOptions, pwmPhase, SIGNAL_HEAD_A_DEF);
	signalHeadISR_OutputPWM(&signalB, signalHeadOptions, pwmPhase, SIGNAL_HEAD_B_DEF);

	// Now do all the counter incrementing and such
	// This will run every millisecond since the timer is running at 4kHz
	if (++subMillisCounter >= 4)
	{
		subMillisCounter = 0;
		millis++;

		if(lockoutTimer)
			lockoutTimer--;
		
		if(timeoutTimer)
			timeoutTimer--;
		
		if(delayTimer)
			delayTimer--;

	}

	pwmPhase = (pwmPhase + 1) & 0x1F;

	if (0 == pwmPhase)
	{
		pwmPhase = 0;
		flasherCounter++;
		if (flasherCounter > 94)
		{
			flasher ^= 0x01;
			flasherCounter = 0;
		}

		// We rolled over the PWM counter, calculate the next PWM widths
		// This runs at 125 frames/second essentially

		signalHeadISR_AspectToNextPWM(&signalA, flasher, signalHeadOptions);
		signalHeadISR_AspectToNextPWM(&signalB, flasher, signalHeadOptions);
	}
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

void initializeTimer()
{
	TIMSK = 0;                                    // Timer interrupts OFF
	// Set up Timer/Counter0 for 100Hz clock
	TCCR0A = 0b00000001;  // CTC Mode
	TCCR0B = 0b00000010;  // CS01 - 1:8 prescaler
	OCR0A = 250;           // 8MHz / 8 / 125 = 8kHz
	TIMSK = _BV(OCIE0A);
}

void init(void)
{
	// Kill watchdog
	MCUSR = 0;
	wdt_reset();
	WDTCR = _BV(WDE) | _BV(WDP2) | _BV(WDP1);   // Enable WDT (1s)
	wdt_reset();


	PORTA = 0x0F;  // Pull-ups on PA0 - PA3
	DDRA = _BV(PA7);  // Aux LED output
	PORTB = 0x70;  // Pull-ups on PB4 - PB6
	DDRB = _BV(PB0) | _BV(PB1) | _BV(PB2) | _BV(PB3);

	initializeTimer();

	signalHeadInitialize(&signalA);
	signalHeadInitialize(&signalB);

	signalHeadAspectSet(&signalA, ASPECT_RED);
	signalHeadAspectSet(&signalB, ASPECT_RED);

	sei();
	wdt_reset();

	timeoutTimer = 0;
	timeoutSeconds = 0;
	lockoutTimer = 0;
	lockoutSeconds = 0;
	delayTimer = 0;
	delaySeconds = 0;
}


int main(void)
{
	Block dir = NONE;
	uint32_t temp_uint32;
	uint16_t delayMin, delayMax;
	InterlockState state = STATE_IDLE;
	
	// Application initialization
	init();
	initializeInputOutput();

	signalHeadOptions = isCommonAnode()?SIGNAL_OPTION_COMMON_ANODE:0;

	wdt_reset();

	// Initialization, board check
	setStatusLed(STATUS_OFF);
	_delay_ms(500);
	wdt_reset();
	
	signalHeadAspectSet(&signalA, ASPECT_GREEN);
	_delay_ms(500);
	wdt_reset();

	signalHeadAspectSet(&signalA, ASPECT_RED);
	_delay_ms(500);
	wdt_reset();
	
	signalHeadAspectSet(&signalB, ASPECT_GREEN);
	_delay_ms(500);
	wdt_reset();

	signalHeadAspectSet(&signalB, ASPECT_RED);
	_delay_ms(500);
	wdt_reset();

	setStatusLed(STATUS_RED);
	_delay_ms(500);
	wdt_reset();

	setStatusLed(STATUS_YELLOW);
	_delay_ms(500);
	wdt_reset();

	setStatusLed(STATUS_GREEN);
	_delay_ms(500);
	wdt_reset();

	setStatusLed(STATUS_BLUE);
	_delay_ms(500);
	wdt_reset();

	setStatusLed(STATUS_PURPLE);
	_delay_ms(500);
	wdt_reset();

	setStatusLed(STATUS_WHITE);
	_delay_ms(500);
	wdt_reset();

	setStatusLed(STATUS_OFF);
	
	clearInterlocking();

	while(1)
	{
		wdt_reset();
		
		readInputs();
		readDipSwitches();
		timeoutSeconds = 15 + (getTimeoutSetting() * 15);  // 15, 30, 45, 60s
		lockoutSeconds = timeoutSeconds;
		
		uint8_t delaySetting = getDelaySetting();
		if(isRandomized())
		{
			// Do something random
			delayMin = 0;
			delayMax = 0;
			switch(delaySetting)
			{
				case 0:
					delayMin = 0;
					delayMax = 5;
					break;
				case 1:
					delayMin = 0;
					delayMax = 10;
					break;
				case 2:
					delayMin = 0;
					delayMax = 15;
					break;
				case 3:
					delayMin = 0;
					delayMax = 20;
					break;

				case 4:
					delayMin = 5;
					delayMax = 15;
					break;
				case 5:
					delayMin = 5;
					delayMax = 20;
					break;
				case 6:
					delayMin = 5;
					delayMax = 25;
					break;
				case 7:
					delayMin = 5;
					delayMax = 30;
					break;

				case 8:
					delayMin = 15;
					delayMax = 30;
					break;
				case 9:
					delayMin = 15;
					delayMax = 40;
					break;
				case 10:
					delayMin = 15;
					delayMax = 50;
					break;
				case 11:
					delayMin = 15;
					delayMax = 60;
					break;

				case 12:
					delayMin = 30;
					delayMax = 60;
					break;
				case 13:
					delayMin = 30;
					delayMax = 120;
					break;

				case 14:
					delayMin = 60;
					delayMax = 120;
					break;
				case 15:
					delayMin = 60;
					delayMax = 300;
					break;
			}
			delaySeconds = (random() % (delayMax - delayMin + 1)) + delayMin;
		}
		else
		{
			// Fixed delays
			delaySeconds = delaySetting * 5;
		}

		wdt_reset();

		uint32_t tempMillis = getMillis();
		switch(state)
		{
			case STATE_IDLE:
				setStatusLed(STATUS_OFF);
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
					srandom(getMillis());  // Re-seed the random generator for next time

					ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
					{
						delayTimer = 1000 * delaySeconds;
					}
					state = STATE_DELAY;
				}
				break;

			case STATE_DELAY:
				setStatusLed(STATUS_YELLOW);
				// Do the delay stuff here
				ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
				{
					temp_uint32 = delayTimer;
				}

				if(!temp_uint32)
				{
					// Delay expired.  Continue.
					state = STATE_REQUEST;
				}
				break;

			case STATE_REQUEST:
				if(!(tempMillis % 250))
				{
					if(!(tempMillis % 500))
						setStatusLed(STATUS_YELLOW);
					else
						setStatusLed(STATUS_OFF);
				}
				if(requestInterlocking(dir))
				{
					// Request for interlocking approved
					state = STATE_CLEARANCE;
				}
				break;

			case STATE_CLEARANCE:
				setStatusLed(STATUS_GREEN);
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
				setStatusLed(STATUS_WHITE);
				ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
				{
					temp_uint32 = timeoutTimer;
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
				else if(!temp_uint32)
				{
					// Timed out.  Reset
					state = STATE_RESET;
				}
				break;

			case STATE_OCCUPIED:
				setStatusLed(STATUS_RED);
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
				setStatusLed(STATUS_BLUE);
				ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
				{
					temp_uint32 = lockoutTimer;
				}

				if(!temp_uint32)
				{
					// Timed out.  Reset
					state = STATE_RESET;
				}
				break;

			case STATE_CLEARING:
				setStatusLed(STATUS_PURPLE);
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
				{
					signalHeadAspectSet(&signalA, ASPECT_GREEN);
					signalHeadAspectSet(&signalB, ASPECT_RED);
				}
				else if(APPROACH_B == dir)
				{
					signalHeadAspectSet(&signalA, ASPECT_RED);
					signalHeadAspectSet(&signalB, ASPECT_GREEN);
				}
				break;
			default:
				// Default to most restrictive aspect
				signalHeadAspectSet(&signalA, ASPECT_RED);
				signalHeadAspectSet(&signalB, ASPECT_RED);
				break;
		}

		wdt_reset();

	}
}


