#ifndef __SCT_FSM_H__
#define __SCT_FSM_H__

/* Generated by fzmparser version 2.2 --- DO NOT EDIT! */

#include "sct_user.h"

extern void sct_fsm_init (void);

/* macros for defining the mapping between IRQ and events */
#define SCT_IRQ_EVENT_no_signal (0)
#define SCT_IRQ_EVENT_width (5)

/* Input assignments */
#define SCT_INPUT_pwm_input (0)

/* Output assignments (and their defaults if specified) */
#define SCT_OUTPUT_timeout (0)
#define SCT_OUTPUTPRELOAD_timeout (1)
#define SCT_OUTPUT_width_error (1)
#define SCT_OUTPUTPRELOAD_width_error (1)

/* Capture registers */
#define SCT_CAPTURE_cap_period LPC_SCT->CAP[3].U
#define SCT_CAPTURE_cap_width LPC_SCT->CAP[4].U


/* Match register reload macro definitions */
#define reload_match_max_width(value) LPC_SCT->MATCHREL[0].U = value;
#define reload_match_min_width(value) LPC_SCT->MATCHREL[1].U = value;
#define reload_match_no_input(value) LPC_SCT->MATCHREL[2].U = value;

#endif