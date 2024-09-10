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

typedef enum
{
	STATUS_RED,
	STATUS_YELLOW,
	STATUS_GREEN,
	STATUS_BLUE,
	STATUS_PURPLE,
	STATUS_WHITE,
	STATUS_OFF,
	STATUS_UNKNOWN,
} Status;

extern uint32_t getMillis();

void initializeInputOutput();
void readDipSwitches();
uint8_t getDelaySetting();
uint8_t getTimeoutSetting();
bool getRandomDelay();
bool getSearchlight();
void readInputs();
bool getInput(Block input);
bool approachBlockOccupancy(uint8_t direction);
bool interlockingBlockOccupancy(void);

void setSignal(Block block, Aspect aspect);
void setStatusLed(Status status);

#endif
