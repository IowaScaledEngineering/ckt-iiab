#include <avr/io.h>
#include <stdint.h>
#include <stdbool.h>
#include "io.h"
#include "debouncer.h"

DebounceState8_t inputDebouncer;

// Inputs (bit / io / name):  
//  0 - PB0 - Approach A
//  1 - PB1 - Approach B
//  2 - PB2 - Diamond

void readInputs()
{
	static uint32_t lastRead = 0;
	uint8_t currentInputState = 0;

	if ((millis - lastRead) > 10)
	{
		lastRead = millis;

		currentInputState = ~( (PINB & _BV(PB0)) | (PINB & _BV(PB1)) | (PINB & _BV(PB2)) );
		debounce8(currentInputState, &inputDebouncer);
	} 
}

bool getInput(Block input)
{
	switch(input)
	{
		case APPROACH_A:
			return( 0 != (getDebouncedState(&inputDebouncer) & _BV(0)) );
			break;
		case APPROACH_B:
			return( 0 != (getDebouncedState(&inputDebouncer) & _BV(1)) );
			break;
		case DIAMOND:
			return( 0 != (getDebouncedState(&inputDebouncer) & _BV(2)) );
			break;
	}
	return false;
}

bool approachBlockOccupancy(uint8_t direction)
{
	switch(direction)
	{
		case 0:
			return getInput(APPROACH_A);
			break;
		case 1:
			return getInput(APPROACH_B);
			break;
		default:
			return false;
			break;
	}
}

bool interlockingBlockOccupancy(void)
{
	return getInput(DIAMOND);
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


void setAuxLed(void)
{
	PORTA |= _BV(PA7);
}

void clearAuxLed(void)
{
	PORTA &= ~(_BV(PA7));
}


