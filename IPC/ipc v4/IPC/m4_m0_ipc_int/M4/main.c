/*****************************************************************************
 * $Id:: main.c 9200 2012-02-22 08:44:47Z nxp28536                           $
 *
 * Project: Mailbox M4<->M0 Application Example for LPC43xx
 *
 * Description:
 *   Implements a Mailbox application
 *----------------------------------------------------------------------------
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
 *****************************************************************************/
#ifndef CORE_M4
	  #error "Build Error: please define CORE_M4 in the project settings" 
#endif
#ifndef IPC_MASTER
	  #error "Build Error: please define IPC_MASTER in the project settings" 
#endif

#include "platform_config.h"
#include "ipc_int.h"
#include "platform_check.h"



static volatile uint32_t u32Milliseconds;
static volatile uint32_t counter;
static volatile uint32_t sendMsg = 1;

uint8_t slaveStarted = 0;

#define CLOCK_SPEED (120000000UL)

/* fromelf.exe always generates an unsigned char LR0[] type of array */
const	/* DO NOT REMOVE THIS CONST QUALIFIER, IS USED TO PLACE THE IMAGE IN M4 ROM */
#include SLAVE_IMAGE_FILE


int main (void) 
{
	/* initializes the platfor dependent things - I/O, clock, fpu etc */
	platformInit();

	#if (PLATFORM == HITEX_BOARD)
	// P 4.15					
	LPC_GPIO4->CLR |= (1UL << 15);    
	#endif

	// just in case
	IPC_haltSlave();

	/* setup the local master interrupt system */
	IPC_masterInitInterrupt(masterInterruptCallback);

	/* download the cpu image */
	IPC_downloadSlaveImage(SLAVE_ROM_START, &LR0[0], sizeof(LR0));

	/* start the remote processor */
	IPC_startSlave();
	
	SysTick_Config(CLOCK_SPEED/1000); // generate interrupts at 1Khz / 1msec
	
	// wait for the M0 to signal being ready via a message
	while(intFlag != MSG_PENDING) __WFE();

	slaveStarted = 1;   /* just track the M0 is alive */

	/* acknowledge the interrupt */
	IPC_resetIntFlag();
	
	while(1)
	{
		// check if we got a message from M0
		if(intFlag == MSG_PENDING)
		{
			// blink when we got the response				
			#if (PLATFORM == HITEX_BOARD)
			// P 4.15					
			LPC_GPIO4->NOT |= (1UL << 15);    
			#endif

			IPC_resetIntFlag();
		};

		// periodically send an interrupt
		if(sendMsg) {
	
			IPC_sendInterrupt();	
			sendMsg = 0;
		};

		__WFE();	  // wait for event either systick or interrupt
	};
}



void SysTick_Handler (void) 					
{           
	if(u32Milliseconds > 0)
	{
		u32Milliseconds--; 
	} 
	else {
		sendMsg = 1;
		u32Milliseconds = 1000;
	}

}


