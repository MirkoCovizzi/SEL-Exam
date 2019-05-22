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

#include "LPC43xx.h"
#include "scu.h"
#include "type.h"

/* ipc comms */
#include "ipc_queue.h"
#include "platform_check.h"



void vIOInit(void);
void initRiTimer(uint32_t timerInterval);


static volatile uint32_t u32Milliseconds = 2000;

#define CLOCK_SPEED (120000000)

srvMsg 		serviceMessage;
rdStsMsg 	readStatusMessage;
wrStsMsg 	writeStatusMessage;	  
rdMsg		readMessage;

maxCmd_t maxCommand;

#define TASK_B_ID		0x1234
#define ARGUMENT_B		0x89A
#define SERVICE_TYPE	0x55
#define PARAMETER		0xABBABEEF

qStat status;
uint32_t validCmds = 0;
uint32_t receivedCmds = 0;
uint32_t sentMsgs = 0;

uint32_t qFull = 0;
uint32_t qEmpty = 0;
uint32_t qError = 0;

/*****************************************************************************
**   Main Function  M0_main()
*****************************************************************************/
int main(void)
{				
	int i = 0;
	
	vIOInit();
	
	/* SysTick is not implemented, use another timer like RITIMER if needed */
	initRiTimer(CLOCK_SPEED/1000);

	// init local mailbox system 
	IPC_slaveInitQueue();

	LPC_GPIO4->SET |= (1UL << 1);

	// signal to the M0 being ready via a message
	MAKE_SRV_MSG_HEADER(serviceMessage,TASK_B_ID,SERVICE_TYPE);
	status = IPC_slavePushMsg(&serviceMessage);
	IPC_msgNotifyMaster();

	LPC_GPIO4->CLR |= (1UL << 1);
		
	/* now push some other messages as a test */
	MAKE_RD_MSG_HEADER(readMessage,TASK_B_ID,ARGUMENT_B); 
	readMessage.param = PARAMETER;
	status = IPC_slavePushMsg((msgToken*)&readMessage);

	MAKE_RDSTS_MSG_HEADER(readStatusMessage,TASK_B_ID,ARGUMENT_B,INVALID_ARG); 
	status = IPC_slavePushMsg(&readStatusMessage);

	MAKE_WRSTS_MSG_HEADER(writeStatusMessage,TASK_B_ID,ARGUMENT_B,WRITE_FAILED); 
	status = IPC_slavePushMsg(&writeStatusMessage);

	IPC_msgNotifyMaster();
	
	while(1)
	{					
		/* sleep */
		__WFE();

		/* see if something has arrived */
		while(IPC_cmdPending()) {
			
			status = IPC_slavePopCmd(&maxCommand);

			if(QEMPTY != status)  {
				receivedCmds++; 
			
				if(QVALID == status) { 
					validCmds++;
				} 
				else if (QERROR == status) {
					qError++;
				};
			}
			else {
			  	qEmpty++;
			};

		};

		/* now send something back */
		MAKE_RD_MSG_HEADER(readMessage,TASK_B_ID,ARGUMENT_B); 
		readMessage.param = PARAMETER;
		status = IPC_slavePushMsg((msgToken*)&readMessage);

		/* keep some statistics */
		if(QFULL == status) qFull++;
		if(QINSERT == status) sentMsgs++;
		if(QERROR == status) qError++;

		MAKE_RDSTS_MSG_HEADER(readStatusMessage,TASK_B_ID,ARGUMENT_B,INVALID_ARG); 
		status = IPC_slavePushMsg(&readStatusMessage);

		if(QFULL == status) qFull++;
		if(QINSERT == status) sentMsgs++;
		if(QERROR == status) qError++;

		MAKE_WRSTS_MSG_HEADER(writeStatusMessage,TASK_B_ID,ARGUMENT_B,WRITE_FAILED); 
		status = IPC_slavePushMsg(&writeStatusMessage);

		if(QFULL == status) qFull++;
		if(QINSERT == status) sentMsgs++;
		if(QERROR == status) qError++;

		/* notify the host */
		IPC_msgNotifyMaster();
		
		for(i=0;i<100000;i++);
		
		// blink 		
		if(u32Milliseconds == 0) {
			#if (PLATFORM == HITEX_BOARD)
			// P4.1
			LPC_GPIO4->NOT |= (1UL << 1);     
			#endif			
			u32Milliseconds = 2000;
		}

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
