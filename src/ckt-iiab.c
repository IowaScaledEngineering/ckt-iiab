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
#include <stdbool.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <avr/interrupt.h>

#include "io.h"
#include "interlocking.h"
#include "debouncer.h"
#include "signalHead.h"

typedef enum
{
	DELAY_PCNT_NONE = 0,
	DELAY_PCNT_LOW,
	DELAY_PCNT_MID,
	DELAY_PCNT_HIGH,
} DelayPcnt;

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

uint32_t timeoutSeconds;
volatile uint32_t timeoutTimer;

uint32_t lockoutSeconds;
volatile uint32_t lockoutTimer;

uint32_t delaySeconds;
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

	initializeInputOutput();

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
	uint32_t delayMin, delayMax;
	DelayPcnt delayPcnt;
	InterlockState state = STATE_IDLE;
	bool first = true;
	uint8_t dipSetting, oldDipSetting;
	
	// Application initialization
	init();

	signalHeadOptions = isCommonAnode()?SIGNAL_OPTION_COMMON_ANODE:0;  // Get CA/CC info before startup test (ignores searchlight mode)

	wdt_reset();

	// Initialization, board check, init debouncers
	setStatusLed(STATUS_OFF);
	readInputs();
	readDipSwitches();
	_delay_ms(200);
	wdt_reset();
	
	signalHeadAspectSet(&signalA, ASPECT_GREEN);
	readInputs();
	readDipSwitches();
	_delay_ms(300);
	wdt_reset();

	signalHeadAspectSet(&signalA, ASPECT_RED);
	readInputs();
	readDipSwitches();
	_delay_ms(300);
	wdt_reset();
	
	signalHeadAspectSet(&signalB, ASPECT_GREEN);
	readInputs();
	readDipSwitches();
	_delay_ms(300);
	wdt_reset();

	signalHeadAspectSet(&signalB, ASPECT_RED);
	readInputs();
	readDipSwitches();
	_delay_ms(300);
	wdt_reset();



	setStatusLed(STATUS_RED);
	_delay_ms(200);
	wdt_reset();

	setStatusLed(STATUS_YELLOW);
	_delay_ms(200);
	wdt_reset();

	setStatusLed(STATUS_GREEN);
	_delay_ms(200);
	wdt_reset();

	setStatusLed(STATUS_BLUE);
	_delay_ms(200);
	wdt_reset();

	setStatusLed(STATUS_PURPLE);
	_delay_ms(200);
	wdt_reset();

	setStatusLed(STATUS_WHITE);
	_delay_ms(200);
	wdt_reset();

	setStatusLed(STATUS_OFF);

	dipSetting = getDipSetting();  // Preload with current value
	oldDipSetting = dipSetting;
	
	clearInterlocking();

	while(1)
	{
		wdt_reset();
		
		readInputs();
		readDipSwitches();

		signalHeadOptions = (isCommonAnode()?SIGNAL_OPTION_COMMON_ANODE:0) | (isSearchlight()?SIGNAL_OPTION_SEARCHLIGHT:0); 

		timeoutSeconds = 15 + (getTimeoutSetting() * 15);  // 15, 30, 45, 60s
		lockoutSeconds = timeoutSeconds;

		wdt_reset();

		uint32_t tempMillis = getMillis();
		switch(state)
		{
			case STATE_IDLE:
				setStatusLed(STATUS_OFF);
				
				// Blink LED when DIP switches change
				dipSetting = getDipSetting();
				if(oldDipSetting != dipSetting)
				{
					setStatusLed(STATUS_RED);
					_delay_ms(50);
					setStatusLed(STATUS_OFF);
					oldDipSetting = dipSetting;
				}
				
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
					if(first)
					{
						srandom(getMillis());  // Seed the random generator
						first = false;
					}

					uint8_t delaySetting = getDelaySetting();
					if(isRandomized())
					{
						// Do something random
						delayMin = 0;
						delayMax = 0;
						delayPcnt = DELAY_PCNT_NONE;
						switch(delaySetting)
						{
							// Simple Ranges
							case 0:
								delayMin = 0;
								delayMax = 10;
								break;
							case 1:
								delayMin = 5;
								delayMax = 20;
								break;
							case 2:
								delayMin = 15;
								delayMax = 30;
								break;
							case 3:
								delayMin = 30;
								delayMax = 60;
								break;
							// Bimodal range 15-30s
							case 4:
								delayMin = 15;
								delayMax = 30;
								delayPcnt = DELAY_PCNT_LOW;
								break;
							case 5:
								delayMin = 15;
								delayMax = 30;
								delayPcnt = DELAY_PCNT_MID;
								break;
							case 6:
								delayMin = 15;
								delayMax = 30;
								delayPcnt = DELAY_PCNT_HIGH;
								break;
							// Bimodal range 30-60s
							case 7:
								delayMin = 30;
								delayMax = 60;
								delayPcnt = DELAY_PCNT_LOW;
								break;
							case 8:
								delayMin = 30;
								delayMax = 60;
								delayPcnt = DELAY_PCNT_MID;
								break;
							case 9:
								delayMin = 30;
								delayMax = 60;
								delayPcnt = DELAY_PCNT_HIGH;
								break;
							// Bimodal range 60-120s
							case 10:
								delayMin = 60;
								delayMax = 120;
								delayPcnt = DELAY_PCNT_LOW;
								break;
							case 11:
								delayMin = 60;
								delayMax = 120;
								delayPcnt = DELAY_PCNT_MID;
								break;
							case 12:
								delayMin = 60;
								delayMax = 120;
								delayPcnt = DELAY_PCNT_HIGH;
								break;
							// Bimodal range 180-300s
							case 13:
								delayMin = 180;
								delayMax = 300;
								delayPcnt = DELAY_PCNT_LOW;
								break;
							case 14:
								delayMin = 180;
								delayMax = 300;
								delayPcnt = DELAY_PCNT_MID;
								break;
							case 15:
								delayMin = 180;
								delayMax = 300;
								delayPcnt = DELAY_PCNT_HIGH;
								break;
						}
						// https://c-faq.com/lib/randrange.html
						if( (DELAY_PCNT_LOW == delayPcnt) && (random() < ((uint32_t)RANDOM_MAX+1u) / 10 * 9) )
						{
							// No delay 90% of the time
							delaySeconds = 1;  // Some minimal delay
						}
						else if( (DELAY_PCNT_MID == delayPcnt) && (random() < ((uint32_t)RANDOM_MAX+1u) / 10 * 7) )
						{
							// No delay 70% of the time
							delaySeconds = 1;  // Some minimal delay
						}
						else if( (DELAY_PCNT_HIGH == delayPcnt) && (random() < ((uint32_t)RANDOM_MAX+1u) / 4) )
						{
							// No delay 25% of the time
							delaySeconds = 1;  // Some minimal delay
						}
						else
						{
							delaySeconds = delayMin + random() / (RANDOM_MAX / (delayMax - delayMin + 1) + 1);
						}
					}
					else
					{
						// Fixed delays
						delaySeconds = delaySetting * 5;
					}

					ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
					{
						delayTimer = 1000 * delaySeconds;
					}
					state = STATE_DELAY;
				}
				break;

			case STATE_DELAY:
				setStatusLed(STATUS_YELLOW);
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

				// Give priority to occupancy then timeout
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
				if(!approachBlockOccupancy(OPPOSITE_DIRECTION(dir)) && !interlockingBlockOccupancy())
				{
					// Opposite approach and interlocking cleared
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


