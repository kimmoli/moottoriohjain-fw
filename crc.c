#include <string.h>
#include <stdio.h>

#include "crc.h"


void initCrc()
{
	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 13);
	LPC_CRC->MODE = 0x15; // CRC-16
	LPC_CRC->SEED = 0xFFFF;
}
