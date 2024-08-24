#include <avr/io.h>
#include <stdint.h>
#include <stdbool.h>
#include "io.h"
#include "debouncer.h"

DebounceState8_t inputDebouncer;

bool randomDelay;
uint8_t delaySetting;
uint8_t timeoutSetting;

void initializeInputOutput()
{
	ADMUX  = 0b00000101;  // VCC reference voltage; right-adjust; ADC5 (PA6)
	ADCSRA = 0b10100111;  // ADC enabled; Auto trigger; 1/128 prescaler
	ADCSRB = 0b10000000;  // Unipolar; 1x gain; Free running mode
	DIDR0 |= _BV(ADC5D);  // Disabled ADC5 digital input buffer
	
	ADCSRA |= _BV(ADSC);  // Start the first conversion
}

void readDipSwitches()
{
	randomDelay = (_BV(PA4) != (PINA & _BV(PA4)));
	
	delaySetting = PINA & 0xF;
	
	// Read ADC for timeout
}

// Inputs (bit / io / name):  
//  0 - PB4 - Approach A
//  1 - PB5 - Approach B
//  2 - PB6 - Diamond

void readInputs()
{
	static uint32_t lastRead = 0;
	uint32_t millisTemp;
	uint8_t currentInputState = 0;

	millisTemp = getMillis();
	if ((millisTemp - lastRead) > 10)
	{
		lastRead = millisTemp;

		currentInputState = ~( ((PINB & _BV(PB4))>>4) | ((PINB & _BV(PB5))>>4) | ((PINB & _BV(PB6))>>4) );
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
		default:
			return false;
			break;
	}
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
					PORTB &= ~(_BV(PB0));
					PORTB |= _BV(PB1);
					break;
				case GREEN:
					PORTB |= _BV(PB0);
					PORTB &= ~(_BV(PB1));
					break;
			}
			break;
		case APPROACH_B:
			switch(aspect)
			{
				case RED:
					PORTB &= ~(_BV(PB2));
					PORTB |= _BV(PB3);
					break;
				case GREEN:
					PORTB |= _BV(PB2);
					PORTB &= ~(_BV(PB3));
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


