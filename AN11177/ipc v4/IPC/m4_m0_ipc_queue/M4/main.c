/*****************************************************************************
 * $Id:: main.c 9200 2012-02-22 08:44:47Z nxp28536                           $
 *
 * Project: Message Queue M4<->M0 Application Example for LPC43xx
 *
 * Description:
 *   Implements a Message Queue application
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

#include "LPC43xx.h"
#include "platform_config.h"

#include "ipc_queue.h"
#include "platform_check.h"

/* fromelf.exe always generates an unsigned char LR0[] type of array */
const	/* DO NOT REMOVE THIS CONST QUALIFIER, IS USED TO PLACE THE IMAGE IN M4 ROM */
#include SLAVE_IMAGE_FILE

/**********************************************************************
 ** Function prototypes
 **********************************************************************/

static volatile uint32_t u32Milliseconds = 1000;
static volatile uint32_t counter;

#define CLOCK_SPEED (120000000UL)
#define COUNTER_VAL 10000

#define TASK_A_ID 0x1234
#define ARGUMENT_A 0x567

rdCmd testRdCmd;
wrCmd testWrCmd;

maxMsg_t msgBack;
uint32_t validMsgs = 0;
uint32_t receivedMsgs = 0;
uint32_t sentCmds = 0;

uint32_t qFull = 0;
uint32_t qEmpty = 0;
uint32_t qError = 0;

qStat status;


int main (void) 
{
	/* initializes the platfor dependent things - I/O, clock, fpu etc */
	platformInit();

	/* setup the queue system - if used - */
	IPC_masterInitQueue(&_hostCmdBufferData[0], MASTER_CMDBUF_SIZE, &_hostMsgBufferData[0], SLAVE_MSGBUF_SIZE);

	/* download the slave image */
	IPC_downloadSlaveImage(SLAVE_ROM_START, &LR0[0], sizeof(LR0));

	/* start the slave processor */
	IPC_startSlave();

	/* generate interrupts at 1Khz / 1msec */
	SysTick_Config(CLOCK_SPEED/1000); 

	// LPC_GPIO4->SET |= (1UL << 15);

	// wait for the M0 to signal being ready via a message
	while(!IPC_msgPending()) __WFE();
	status = IPC_masterPopMsg(&msgBack);
										
	while(1)
	{		
		/* sleep */
		__WFE();
		
		/* see if something has arrived */
		while(IPC_msgPending()) {
			
			status = IPC_masterPopMsg(&msgBack);

			if(QEMPTY != status)  {
				receivedMsgs++; 
			
				if(QVALID == status) { 
					validMsgs++;
				} 
				else if (QERROR == status) {
					qError++;
				};
			}
			else {
			  	qEmpty++;
			};

		};
		
		/* now send some other commands */
		MAKE_READ_CMD_HEADER(testRdCmd,TASK_A_ID,ARGUMENT_A);
		status = IPC_masterPushCmd(&testRdCmd);
		
		/* keep some stats */
		if(QFULL == status) qFull++;
		if(QINSERT == status) sentCmds++;
		if(QERROR == status) qError++;
		 
		MAKE_WRITE_CMD_HEADER(testWrCmd, TASK_A_ID, ARGUMENT_A);
		testWrCmd.param = 0xCAFECAFE;
		status = IPC_masterPushCmd((cmdToken*) &testWrCmd);
		
		if(QFULL == status) qFull++;
		if(QINSERT == status) sentCmds++;
		if(QERROR == status) qError++;
		
		/* notify the slave */
		IPC_cmdNotifySlave();
				
		// blink 		
		if(u32Milliseconds == 0) {
			#if (PLATFORM == HITEX_BOARD)
			// P4.15
			LPC_GPIO4->NOT |= (1UL << 15);  
			#endif
			u32Milliseconds = 1000;
		}

	};
}

/*----------------------------------------------------------------------------
  Initialize board specific IO
 *----------------------------------------------------------------------------*/
	
/*----------------------------------------------------------------------------
  Initialize clocks
 *----------------------------------------------------------------------------*/

void SysTick_Handler (void) 					
{           
	if(u32Milliseconds > 0)
	{
		u32Milliseconds--; 
	}
}


