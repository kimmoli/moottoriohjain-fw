/*
 * sct.c
 *
 *  Created on: 27.6.2013
 *      Author: kimmo.lindholm
 */


#include <string.h>
#include <stdio.h>

#include "sct.h"

void SCT_IRQHandler(void)
{

	NVIC_DisableIRQ(SCT_IRQn);
	LPC_SCT->EVFLAG = 0x3F;
	LPC_SCT->CTRL_U &= ~(1 << 1); // remove stop
	printf("SCT");
	NVIC_EnableIRQ(SCT_IRQn);

}


void initSct()
{
	/* Enable clocks and toggle reset */
	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 8);
	LPC_SYSCON->PRESETCTRL    &= ~(1 << 8);
	LPC_SYSCON->PRESETCTRL    |=  (1 << 8);

	LPC_SCT->CONFIG = (LPC_SCT->CONFIG & ~0x00060001) | 0x00000001; /* UNIFIED */

	/* MATCH/CAPTURE registers */

	/* Unified counter - register side L is used and accessed as 32 bit value, reg H is not used */
	LPC_SCT->REGMODE_L = 0x00000000;         /* U: 1x MATCH, 0x CAPTURE, 4 unused */

	LPC_SCT->MATCH[0].U = 5280;             /* Timer_expired */
	LPC_SCT->MATCHREL[0].U = 5280;

	/* OUTPUT registers */
	  /* Unused outputs must not be affected by any event */
	LPC_SCT->OUT[0].SET = 0;
	LPC_SCT->OUT[0].CLR = 0;
	LPC_SCT->OUT[1].SET = 0;
	LPC_SCT->OUT[1].CLR = 0;
	LPC_SCT->OUT[2].SET = 0;
	LPC_SCT->OUT[2].CLR = 0;
	LPC_SCT->OUT[3].SET = 0;
	LPC_SCT->OUT[3].CLR = 0;

	/* Conflict resolution register */

	/* EVENT registers */
	LPC_SCT->EVENT[0].CTRL = 0x00006000;     /* U: --> state state_1 */ // load state 0, CTIO0=low
	LPC_SCT->EVENT[0].STATE = 0xFFFFFFFF;
	LPC_SCT->EVENT[1].CTRL = 0x0000D000;     /* U: --> state state_2 */ // load state 1, match reg0
	LPC_SCT->EVENT[1].STATE = 0x00000001;
	  /* Unused events must not have any effect */
	LPC_SCT->EVENT[2].STATE = 0;
	LPC_SCT->EVENT[3].STATE = 0;
	LPC_SCT->EVENT[4].STATE = 0;
	LPC_SCT->EVENT[5].STATE = 0;

	/* STATE registers */
	LPC_SCT->STATE_L = 0; /* implicit value */

	/* state names assignment: */
	  /* State U 0: state_1 */
	  /* State U 1: state_2 */

	/* CORE registers */
	LPC_SCT->START_L = 0x00000003;
	LPC_SCT->STOP_L =  0x00000000;
	LPC_SCT->HALT_L =  0x00000000;
	LPC_SCT->LIMIT_L = 0x00000000;
	LPC_SCT->EVEN =    0x00000002;

	LPC_SCT->EVFLAG = 0x3F;
	LPC_SCT->EVEN = 0x02; // enable interrupt from event 1

	LPC_SCT->CTRL_U &= ~(1 << 2); // remove halt

	NVIC_EnableIRQ(SCT_IRQn);
}
