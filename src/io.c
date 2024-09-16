#include <avr/io.h>
#include <stdint.h>
#include <stdbool.h>
#include <avr/wdt.h>
#include "io.h"
#include "debouncer.h"
#include "light_ws2812.h"
#include "signalHead.h"

DebounceState8_t inputDebouncer;

extern volatile uint8_t signalHeadOptions;


bool randomDelay;
bool searchlight;
uint8_t delaySetting;
uint8_t timeoutSetting;

void initializeInputOutput()
{
	ADMUX  = 0b00100011;  // VCC reference voltage; left-adjust; ADC3 (PA4)
	ADCSRA = 0b10000111;  // ADC enabled; Manual trigger; 1/128 prescaler
	ADCSRB = 0b00000000;  // Unipolar; 1x gain; Free running mode
	DIDR0 |= _BV(ADC3D) | _BV(ADC4D);  // Disable ADC3 (PA4) and ADC4 (PA5) digital input buffer
}

bool isCommonAnode(void)
{
	return ((PINA & _BV(PA6)) == _BV(PA6));
}

void readDipSwitches()
{
	uint8_t adcVal;
	uint8_t newSignalHeadOptions = 0;

	delaySetting = ~PINA & 0x0F;
	
	// Read ADC for random, searchlight
	wdt_reset();
	ADMUX &= ~(_BV(MUX2)); //  ADC3 (PA4)
	ADMUX |= _BV(MUX1) | _BV(MUX0);
	ADCSRA |= _BV(ADSC);  // Start the conversion
	while(ADCSRA & _BV(ADSC));  // Wait for conversion to complete; no WD reset in case it takes too long
	adcVal = ADCH;
	if(adcVal > 212)
	{
		searchlight = 0;
		randomDelay = 0;
	}
	else if(adcVal > 149)
	{
		searchlight = 1;
		randomDelay = 0;
	}
	else if(adcVal > 115)
	{
		searchlight = 0;
		randomDelay = 1;
	}
	else
	{
		searchlight = 1;
		randomDelay = 1;
	}

	// Read ADC for timeout
	wdt_reset();
	ADMUX |= _BV(MUX2); //  ADC4 (PA5)
	ADMUX &= ~(_BV(MUX1) | _BV(MUX0));
	ADCSRA |= _BV(ADSC);  // Start the conversion
	while(ADCSRA & _BV(ADSC));  // Wait for conversion to complete; no WD reset in case it takes too long
	adcVal = ADCH;
	if(adcVal > 212)
		timeoutSetting = 0;
	else if(adcVal > 149)
		timeoutSetting = 1;
	else if(adcVal > 115)
		timeoutSetting = 2;
	else
		timeoutSetting = 3;
		

	newSignalHeadOptions = isCommonAnode()?SIGNAL_OPTION_COMMON_ANODE:0;
	newSignalHeadOptions |= searchlight?SIGNAL_OPTION_SEARCHLIGHT:0;
	signalHeadOptions = newSignalHeadOptions;
}

uint8_t getDelaySetting(void)
{
	return delaySetting;
}

uint8_t getTimeoutSetting(void)
{
	return timeoutSetting;
}

bool isRandomized(void)
{
	return randomDelay;
}

bool isSearchlight(void)
{
	return searchlight;
}

// Inputs (bit / io / name):
// Note: Approach B and Diamond labels are swapped on v1.2 hardware, fixed in code
//  0 - PB4 - Approach B
//  1 - PB5 - Diamond
//  2 - PB6 - Approach A

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
			return( 0 != (getDebouncedState(&inputDebouncer) & _BV(2)) );
			break;
		case APPROACH_B:
			return( 0 != (getDebouncedState(&inputDebouncer) & _BV(0)) );
			break;
		case DIAMOND:
			return( 0 != (getDebouncedState(&inputDebouncer) & _BV(1)) );
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
			if( ((RED == aspect) & isCommonAnode()) || ((GREEN == aspect) & !isCommonAnode()) )
			{
				PORTB &= ~(_BV(PB0));
				PORTB |= _BV(PB1);
			}
			else if( ((GREEN == aspect) & isCommonAnode()) || ((RED == aspect) & !isCommonAnode()) )
			{
				PORTB |= _BV(PB0);
				PORTB &= ~(_BV(PB1));
			}
			else if(OFF == aspect)
			{
				if(isCommonAnode())
				{
					PORTB |= _BV(PB0);
					PORTB |= _BV(PB1);
				}
				else
				{
					PORTB &= ~(_BV(PB0));
					PORTB &= ~(_BV(PB1));
				}
			}
			break;
		case APPROACH_B:
			if( ((RED == aspect) & isCommonAnode()) || ((GREEN == aspect) & !isCommonAnode()) )
			{
				PORTB &= ~(_BV(PB2));
				PORTB |= _BV(PB3);
			}
			else if( ((GREEN == aspect) & isCommonAnode()) || ((RED == aspect) & !isCommonAnode()) )
			{
				PORTB |= _BV(PB2);
				PORTB &= ~(_BV(PB3));
			}
			else if(OFF == aspect)
			{
				if(isCommonAnode())
				{
					PORTB |= _BV(PB2);
					PORTB |= _BV(PB3);
				}
				else
				{
					PORTB &= ~(_BV(PB2));
					PORTB &= ~(_BV(PB3));
				}
			}
			break;
		default:
			return;
	}
}


void setStatusLed(Status status)
{
	static Status oldStatus = STATUS_UNKNOWN;
	struct cRGB led;

	if(status != oldStatus)
	{
		switch(status)
		{
			case STATUS_RED:
				led.r = 64;
				led.g = 0;
				led.b = 0;
				break;
			case STATUS_YELLOW:
				led.r = 64;
				led.g = 24;
				led.b = 0;
				break;
			case STATUS_GREEN:
				led.r = 0;
				led.g = 64;
				led.b = 0;
				break;
			case STATUS_BLUE:
				led.r = 0;
				led.g = 0;
				led.b = 64;
				break;
			case STATUS_PURPLE:
				led.r = 64;
				led.g = 0;
				led.b = 64;
				break;
			case STATUS_WHITE:
				led.r = 64;
				led.g = 64;
				led.b = 64;
				break;
			case STATUS_OFF:
				led.r = 0;
				led.g = 0;
				led.b = 0;
				break;
			case STATUS_UNKNOWN:
				break;
		}
		ws2812_setleds(&led,1);
		oldStatus = status;
	}
}
