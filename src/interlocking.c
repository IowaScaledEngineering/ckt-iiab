#include <avr/io.h>
#include <stdint.h>
#include <stdbool.h>
#include "interlocking.h"
#include "io.h"

uint8_t interlockingStatus;

bool requestInterlocking(uint8_t direction)
{
	if(interlockingStatus & INTERLOCKING_LOCKED)
		return false;  // Already locked
	else if(interlockingBlockOccupancy())
		return false;  // Interlocking occupied
	
	// Everything good.  Take it.
	interlockingStatus = INTERLOCKING_LOCKED | _BV(direction);
	return true;
}


void clearInterlocking(void)
{
	interlockingStatus = 0;
}
