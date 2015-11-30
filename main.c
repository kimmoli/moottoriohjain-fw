/**************************************************************************/
/*!
    @file     main.c

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013, K. Townsend (microBuilder.eu)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/
#include <stdio.h>
#include <string.h>
#include "LPC8xx.h"
#include "gpio.h"
#include "mrt.h"
#include "uart.h"
#include "crc.h"
#include "sct.h"
#include "spi.h"

#if defined(__CODE_RED)
  #include <cr_section_macros.h>
  #include <NXP/crp.h>
  __CRP const unsigned int CRP_WORD = CRP_NO_CRP ;
#endif

#define MIN 1500

/* This define should be enabled if you want to      */
/* maintain an SWD/debug connection to the LPC810,   */
/* but it will prevent you from having access to the */
/* LED on the LPC810 Mini Board, which is on the     */
/* SWDIO pin (PIO0_2).                               */
// #define USE_SWD


  /*
	PIO0_6	M1CW	Output
	PIO0_7	M1CCW	Output
	PIO0_8	M2CW	Output
	PIO0_9	M2CCW	Output

	PIO0_1	ST	Input

	PIO0_13	LED	Output
	PIO0_4	TXD
	PIO0_0	RXD

	PIO0_3	S2	Input
	PIO0_2	S1	Input
   * */


void configurePins()
{
    /* Enable SWM clock */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<7);


    /* Pin Assign 8 bit Configuration */
    /* U0_TXD */
    /* U0_RXD */
    LPC_SWM->PINASSIGN0 = 0xffff0004UL;

    /* Pin Assign 1 bit Configuration */
    /* RESET */
    LPC_SWM->PINENABLE0 = 0xffffffbfUL;


    /* Outputs */
    LPC_GPIO_PORT->DIR0 |= (1 << 6);
    LPC_GPIO_PORT->DIR0 |= (1 << 7);
    LPC_GPIO_PORT->DIR0 |= (1 << 8);
    LPC_GPIO_PORT->DIR0 |= (1 << 9);
    LPC_GPIO_PORT->DIR0 |= (1 << 13); // PIO0_13 = LED~ out

	/* Pin I/O Configuration */
	/* LPC_IOCON->PIO0_0 = 0x90; */
	/* LPC_IOCON->PIO0_1 = 0x90; */
	LPC_IOCON->PIO0_2 = 0x88;
	LPC_IOCON->PIO0_3 = 0x88;
	/* LPC_IOCON->PIO0_4 = 0x90; */
	/* LPC_IOCON->PIO0_5 = 0x90; */
	LPC_IOCON->PIO0_6 = 0x88;
	LPC_IOCON->PIO0_7 = 0x88;
	LPC_IOCON->PIO0_8 = 0x88;
	LPC_IOCON->PIO0_9 = 0x88;
	/* LPC_IOCON->PIO0_10 = 0x80; */
	/* LPC_IOCON->PIO0_11 = 0x80; */
	/* LPC_IOCON->PIO0_12 = 0x90; */
	/* LPC_IOCON->PIO0_13 = 0x90; */

}


int main(void)
{
  /* Configure the core clock/PLL via CMSIS */
  SystemCoreClockUpdate();

  /* Initialize the GPIO block */
  gpioInit();

  /* Initialize the UART0 block for printf output */
  uart0Init(115200);

  /* Configure the multi-rate timer for 100us ticks */
  mrtInit(SystemCoreClock/1000);

  /* Configure the switch matrix (setup pins for UART0 and GPIO) */
  configurePins();

  LPC_GPIO_PORT->CLR0 = 1 << 13; // lit led

  printf("SystemCoreClock %d\r\n", SystemCoreClock );
  printf("MainClock %d\r\n", MainClock );
  printf("LPC_SYSCON->UARTFRGMULT %d\r\n", LPC_SYSCON->UARTFRGMULT);
  printf("LPC_SYSCON->UARTFRGDIV %d\r\n", LPC_SYSCON->UARTFRGDIV);
  printf("LPC_USART0->BRG %d\r\n", LPC_USART0->BRG);
  printf("LPC_SYSCON->SYSAHBCLKDIV %d\r\n", LPC_SYSCON->SYSAHBCLKDIV);
  printf("LPC_SYSCON->SYSPLLCTRL %x\r\n", LPC_SYSCON->SYSPLLCTRL);

  rxRead = 0;

  volatile uint32_t mrt_last = 0;

  rxRead = 0;


  initSct();
  LPC_GPIO_PORT->CLR0 = 0xf << 6; // clear all outputs

  while(1)
  {
	  __WFI();
	  if (mrt_counter > (mrt_last + 100))
	  {
		  mrt_last = mrt_counter;
		  LPC_GPIO_PORT->NOT0 = 1 << 13; // toggle led
	  }
  }
}



