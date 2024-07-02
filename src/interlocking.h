#ifndef _INTERLOCKING_H_
#define _INTERLOCKING_H_

#include <stdbool.h>

#define INTERLOCKING_LOCKED 0x80

bool requestInterlocking(uint8_t direction);
void clearInterlocking(void);

#endif
