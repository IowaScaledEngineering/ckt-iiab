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
	OFF,
	RED,
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
uint8_t getDipSetting();
uint8_t getDelaySetting();
uint8_t getTimeoutSetting();
bool isRandomized();
bool isSearchlight();
void readInputs();
bool getInput(Block input);
bool approachBlockOccupancy(uint8_t direction);
bool interlockingBlockOccupancy(void);

void setStatusLed(Status status);
bool isCommonAnode(void);

#endif
