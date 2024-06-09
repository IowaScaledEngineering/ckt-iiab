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

typedef enum
{
	APPROACH_A = 0,
	APPROACH_B,
	DIAMOND,
} Block;

typedef enum
{
	RED = 0,
	GREEN,
} Aspect;

uint8_t readInput(Block input)
{
	uint8_t inputPin;
	switch(input)
	{
		case APPROACH_A:
			inputPin = PB0;
			break;
		case APPROACH_B:
			inputPin = PB1;
			break;
		case DIAMOND:
			inputPin = PB2;
			break;
	}
	return(PINB & _BV(inputPin));
}

void setSignal(Block block, Aspect aspect)
{
	switch(block)
	{
		case APPROACH_A:
			switch(aspect)
			{
				case RED:
					PORTB &= ~(_BV(PB3));
					PORTB |= _BV(PB4);
					break;
				case GREEN:
					PORTB |= _BV(PB3);
					PORTB &= ~(_BV(PB4));
					break;
			}
			break;
		case APPROACH_B:
			switch(aspect)
			{
				case RED:
					PORTB &= ~(_BV(PB5));
					PORTB |= _BV(PB6);
					break;
				case GREEN:
					PORTB |= _BV(PB5);
					PORTB &= ~(_BV(PB6));
					break;
			}
			break;
		default:
			return;
	}
}


inline void setAuxLed(void)
{
	PORTA |= _BV(PA7);
}

inline void clearAuxLed(void)
{
	PORTA &= ~(_BV(PA7));
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
	PORTB = 0x7F;  // Pull-ups on PB0 - PB2
	setSignal(APPROACH_A, RED);
	setSignal(APPROACH_B, RED);
	DDRB = _BV(PB3) | _BV(PB4) | _BV(PB5) | _BV(PB6);
}


int main(void)
{
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
	}
}
