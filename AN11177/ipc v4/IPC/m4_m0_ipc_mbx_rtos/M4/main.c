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

#include "LPC43xx.h"
#include "platform_config.h"

#include "ipc_mbx.h"
#include "platform_check.h"

#include "FreeRTOS.h"
#include "task.h"

#define taskSendCmdSTACK_SIZE		( ( unsigned short ) configMINIMAL_STACK_SIZE )
#define taskRcvCmdSTACK_SIZE		( ( unsigned short ) configMINIMAL_STACK_SIZE )

/* fromelf.exe always generates an unsigned char LR0[] type of array */
const	/* DO NOT REMOVE THIS CONST QUALIFIER, IS USED TO PLACE THE IMAGE IN M4 ROM */
#include SLAVE_IMAGE_FILE

/**********************************************************************
 ** Function prototypes
 **********************************************************************/
void vIOInit(void);
void ClockInit(void);

uint8_t slaveStarted = 0;

#define CLOCK_SPEED (120000000UL)
#define COUNTER_VAL 100

void fpu_init(void);

void vCmdResponseTask( void *pvParameters ) {

	uint32_t id;

	uint32_t counter = COUNTER_VAL;

	// just in case
	IPC_haltSlave();

	/* setup the local master mailbox system */
	IPC_initMasterMbx(&Master_CbackTable[0], (Mbx*) MASTER_MBX_START, (Mbx*) SLAVE_MBX_START);

	/* download the cpu image */
	IPC_downloadSlaveImage(SLAVE_ROM_START, &LR0[0], sizeof(LR0));

	/* start the remote processor */
	IPC_startSlave();
			
	// wait for the M0 to signal being ready via a message
	while(mbxFlags[MASTER_MBX_CMD] != MSG_PENDING) __WFE();

	if(RESPONSE_SLAVE_STARTED == IPC_getMsgType(MASTER_MBX_CMD)) {
		
		slaveStarted = 1;   /* just track the M0 is alive */

		/* free our mbx */
		IPC_freeMbx(MASTER_MBX_CMD);
	};

	while(1) {
		
		// check if we got a message from M0
		if(mbxFlags[MASTER_MBX_CMD] == MSG_PENDING)
		{
			IPC_lockMbx(MASTER_MBX_CMD);
	
			/* check the response we got */
			if(RESPONSE_SLAVE_LED_BLINKED == IPC_getMsgType(MASTER_MBX_CMD))
			{
				id = IPC_getMsgId(MASTER_MBX_CMD);
				
				// blink when we got the response				
				if(counter > 0)
				{
					counter--; 
				}
				else {
					#if (PLATFORM == HITEX_BOARD)
					// P4.15
					LPC_GPIO4->NOT |= (1UL << 15);    
					#endif
					counter = COUNTER_VAL;
				}				
	
			};
	
			IPC_freeMbx(MASTER_MBX_CMD);
		};

		__WFE();
	};

}

void vCmdSendTask( void *pvParameters ) {

	static uint32_t id = 0;
	while(1) {
		// if there is space on the remote mailbox, send a new msg to the M0
		if(IPC_queryRemoteMbx(SLAVE_MBX_CMD) == READY) {

			IPC_sendMsg(SLAVE_MBX_CMD, CMD_SLAVE_BLINK, (msgId_t) id, (mbxParam_t) id);	

			id++;
		};

		__WFE();
	};
}

#define taskSend_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define taskReceive_PRIORITY			( tskIDLE_PRIORITY + 1 )

int main (void) 
{

	/* initializes the platfor dependent things - I/O, clock, fpu etc */
	platformInit();

	xTaskCreate( vCmdResponseTask, "AnsTask", taskRcvCmdSTACK_SIZE, ( void * ) NULL, taskReceive_PRIORITY, NULL );
	xTaskCreate( vCmdSendTask, "SendTask", taskSendCmdSTACK_SIZE, ( void * ) NULL, taskSend_PRIORITY, NULL );

	/* Start the scheduler. */
	vTaskStartScheduler();

    /* Will only get here if there was insufficient memory to create the idle
    task.  The idle task is created within vTaskStartScheduler(). */
	for( ;; );

}



void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	/* This function will get called if a task overflows its stack. */

	( void ) pxTask;
	( void ) pcTaskName;

	for( ;; );
}

/* The time between cycles of the 'check' functionality (defined within the
tick hook. */
#define mainCHECK_DELAY						( ( portTickType ) 5000 / portTICK_RATE_MS )

void vApplicationTickHook( void )
{
	static unsigned long ulTicksSinceLastDisplay = 0;

	ulTicksSinceLastDisplay++;
	if( ulTicksSinceLastDisplay >= mainCHECK_DELAY )
	{
		/* Reset the counter so these checks run again in mainCHECK_DELAY
		ticks time. */
		ulTicksSinceLastDisplay = 0;

		/* here perform some application level checks...*/

	}
}



