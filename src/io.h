#ifndef _IO_H_
#define _IO_H_

#include <stdbool.h>

typedef enum
{
	APPROACH_A = 0,
	APPROACH_B = 1,
	DIAMOND,
	NONE,
} Block;

typedef enum
{
	RED = 0,
	GREEN,
} Aspect;

extern uint32_t getMillis();

void initializeInputOutput();
void readDipSwitches();
void readInputs();
bool getInput(Block input);
bool approachBlockOccupancy(uint8_t direction);
bool interlockingBlockOccupancy(void);

void setSignal(Block block, Aspect aspect);
void setAuxLed(void);
void clearAuxLed(void);

#endif
