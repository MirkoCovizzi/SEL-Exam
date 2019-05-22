/*********************************************************************** 
 * $Id: main.c 9200 2012-02-22 08:44:47Z nxp28536 $
 * 
 * Project: Mailbox M4<->M0 Application Example for LPC43xx 
 * 
 * Description: Implements a Mailbox application
 * 
 * Copyright(C) 2011, NXP Semiconductor
 * All rights reserved.
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
#ifndef CORE_M0
	  #error "Build Error: please define CORE_M0 in the project settings" 
#endif
#ifndef IPC_SLAVE
	  #error "Build Error: please define IPC_SLAVE in the project settings" 
#endif

#include "LPC43xx.h"
#include "scu.h"
#include "type.h"

/* ipc comms */
#include "ipc_int.h"
#include "platform_check.h"

static volatile uint32_t u32Milliseconds = 1000;
static volatile uint32_t counter;
static volatile uint32_t sendResponse = 0;

#define COUNTER_VAL 10000

void vIOInit(void);
void initRiTimer(uint32_t timerInterval);

/*****************************************************************************
**   Main Function  M0_main()
*****************************************************************************/
#define CLOCK_SPEED (120000000)
int main(void)
{		

	vIOInit();

	#if (PLATFORM == HITEX_BOARD)
	// P.4.1
	LPC_GPIO4->SET |= (1UL << 1);     
	#endif
			
	/* SysTick is not implemented, use another timer like RITIMER if needed */
	// initRiTimer(CLOCK_SPEED/1000);

	// init local mailbox system 
	IPC_slaveInitInterrupt(slaveInterruptCallback);

	// signal back to M4 we are now ready
	IPC_sendInterrupt();

   	counter = COUNTER_VAL;

	while(1)
	{					
		// check for interrupt pending
		if(intFlag == MSG_PENDING)
		{
			#if (PLATFORM == HITEX_BOARD)
			// P.4.1
			LPC_GPIO4->NOT |= (1UL << 1);     
			#endif

			sendResponse = 1;

			IPC_resetIntFlag();
		}

		// query if we can send back a response to M4
		if(sendResponse) {

			IPC_sendInterrupt();
			sendResponse = 0;
		};

		/* wait for a message to arrive after response has been sent */
		/* otherwise will keep trying until CMD MBX is free */
		__WFE();
			
	}
		
}

/*----------------------------------------------------------------------------
  Initialize board specific IO
 *----------------------------------------------------------------------------*/
void vIOInit(void)
{
	#if (PLATFORM == HITEX_BOARD)
		// P8.1 : GPIO4_1
		scu_pinmux(0x8 ,1 , PDN_ENABLE, FUNC0); 	
		LPC_GPIO4->DIR |= (1UL << 1);
	#endif
}

/**********************************************************************
 ** Function name:		
 **
 ** Description:		
 **						
 ** Parameters:			
 **
 ** Returned value:		
 **********************************************************************/
void RIT_WDT_IRQHandler (void) 					
{           
	LPC_RITIMER->CTRL |=(1<<0);	// Clear interrupt flag
	if(u32Milliseconds > 0)
	{
		u32Milliseconds--; 
	}
	else
	{
		u32Milliseconds = 1000;
	}
}

void initRiTimer(uint32_t TimerInterval)
{
	LPC_RITIMER->COMPVAL = TimerInterval; 			// Set match value
	LPC_RITIMER->COUNTER=0;							// Set count value to 0
	LPC_RITIMER->CTRL = (1<<3)|(1<<2)|(1<<1);		// Enable, enable break, reset on mask compare, clear interrupt flag

	NVIC_EnableIRQ(RITIMER_WDT_IRQn);

}

/*****************************************************************************
**                            End Of File
*****************************************************************************/
