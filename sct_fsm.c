
/* Generated by fzmparser version 2.2 --- DO NOT EDIT! */

/* Uses following resources: */
/* [6 events, 2+0 states, 1 inputs, 2 outputs, 3 match regs, 2 capture regs] */

#include "sct_fsm.h"


void sct_fsm_init (void)
{
LPC_SCT->CONFIG = (LPC_SCT->CONFIG & ~0x00060001); /* SPLIT */

/* MATCH/CAPTURE registers */

LPC_SCT->REGMODE_L = 0x0010;         /* U: 1x MATCH, 1x CAPTURE, 2 unused */
LPC_SCT->REGMODE_H = 0x0010;         /* U: 1x MATCH, 1x CAPTURE, 2 unused */

LPC_SCT->MATCH[2].L = val_match_no_input;             /* match_no_input */
LPC_SCT->MATCHREL[2].L = val_match_no_input;
LPC_SCT->CAPCTRL[4].L = 0x00000002;

LPC_SCT->MATCH[2].H = val_match_no_input;             /* match_no_input */
LPC_SCT->MATCHREL[2].H = val_match_no_input;
LPC_SCT->CAPCTRL[4].H = 0x00000008;

/* Conflict resolution register */

/* EVENT registers */

/* Signal1 no signal */
LPC_SCT->EVENT[0].STATE = 1;
LPC_SCT->EVENT[0].CTRL  = (1 << 12) | 2;   // match 2 condition

/* Signal1 capture */
LPC_SCT->EVENT[1].STATE = 1;
LPC_SCT->EVENT[1].CTRL  = (2 << 12) | (2 << 10);   //  CTIN_0 input

/* Signal2 no signal */
LPC_SCT->EVENT[2].STATE = 1;
LPC_SCT->EVENT[2].CTRL  = (1 << 12) | (1 << 4) | 2;   // match 2 condition

LPC_SCT->EVENT[3].STATE = 1;
LPC_SCT->EVENT[3].CTRL  = (2 << 12) | (2 << 10) | (1 << 6) | (1 << 4);   //  CTIN_1 input

LPC_SCT->EVENT[4].STATE = 1;
LPC_SCT->EVENT[4].CTRL  = (2 << 12) | (1 << 10);

LPC_SCT->EVENT[5].STATE = 1;
LPC_SCT->EVENT[5].CTRL  = (2 << 12) | (1 << 10) | (1 << 6) | (1 << 4);

/* STATE registers */
LPC_SCT->STATE_L = 0;
LPC_SCT->STATE_H = 0;

/* state names assignment: */
  /* State U 0: U_ENTRY */
  /* State U 1: state_1 */

/* CORE registers */
LPC_SCT->START_L = 0x0000;
LPC_SCT->STOP_L =  0x0000;
LPC_SCT->HALT_L =  0x0000;
LPC_SCT->LIMIT_L = 0x0011;
LPC_SCT->START_H = 0x0000;
LPC_SCT->STOP_H =  0x0000;
LPC_SCT->HALT_H =  0x0000;
LPC_SCT->LIMIT_H = 0x0024;
LPC_SCT->EVEN =    0x0000000F;

}
