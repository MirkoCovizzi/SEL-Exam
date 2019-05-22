/***********************************************************************
 * $Id:: retarget.c 8242 2011-10-11 15:15:25Z nxp28536                 $
 *
 * Project: LPC43xx Common
 *
 * Description:
 *     'Retarget' layer for target-dependent low level functions
 *
 ***********************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
 **********************************************************************/

#include <stdio.h>
#include <rt_misc.h>
#include <stdint.h>

#if defined CORE_M4
#include "LPC43xx.h"                    /* LPC43xx definitions                */
#endif

#ifdef CORE_M0
#include "LPC43xx_M0.h"                /* LPC43xx definitions                */
#endif

#include "config.h"
#include "uart.h"

#pragma import(__use_no_semihosting_swi)

uint32_t retarget_default = 0;

/*----------------------------------------------------------------------------
  Init retarget
 *----------------------------------------------------------------------------*/
void RetargetInit(RETARGET_Type retarget, uint32_t baudrate)
{
	retarget_default = retarget;
	switch (retarget)
	{
		case RETARGET_USART0:
			SetClock(BASE_UART0_CLK, SRC_PL160M_0, DIV1);	
			UARTInit(0, baudrate);	/* baud rate setting */
			break;
		case RETARGET_UART1:
			SetClock(BASE_UART1_CLK, SRC_PL160M_0, DIV1);
			UARTInit(1, baudrate);	/* baud rate setting */
			break;
		case RETARGET_USART2:
			SetClock(BASE_UART2_CLK, SRC_PL160M_0, DIV1);
			UARTInit(2, baudrate);	/* baud rate setting */
			break;
		case RETARGET_USART3:
			SetClock(BASE_UART3_CLK, SRC_PL160M_0, DIV1);
			UARTInit(3, baudrate);	/* baud rate setting */
			break;
		default:
			break;
	}	
}

/*----------------------------------------------------------------------------
  Write character to Serial Port
 *----------------------------------------------------------------------------*/
int sendchar (int c) {

	uint8_t data[1];

	data[0] = c;
	UARTSend(retarget_default, data , 1);
	return (c);
}

/*----------------------------------------------------------------------------
  Read character from Serial Port   (blocking read)
 *----------------------------------------------------------------------------*/
int getkey (void) {

//  return (SER_getChar(1));
	return(0);
}

struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f) {
  return (sendchar(ch));
}

int fgetc(FILE *f) {
  return (getkey());
}

int ferror(FILE *f) {
  /* Your implementation of ferror */
  return EOF;
}

void _ttywrch(int ch) {
  sendchar(ch);
}

void _sys_exit(int return_code) {
label:  goto label;  /* endless loop */
}
