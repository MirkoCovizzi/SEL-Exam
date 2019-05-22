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

#include "FreeRTOS.h"
#include "task.h"

#define taskSendCmdSTACK_SIZE		( ( unsigned short ) configMINIMAL_STACK_SIZE )
#define taskRcvCmdSTACK_SIZE		( ( unsigned short ) configMINIMAL_STACK_SIZE )

/**********************************************************************
 ** Function prototypes
 **********************************************************************/

static volatile uint32_t u32Milliseconds = 1000;
static volatile uint32_t counter;


#define CLOCK_SPEED (120000000UL)
#define COUNTER_VAL 10000

/* fromelf.exe always generates an unsigned char LR0[] type of array */
const	/* DO NOT REMOVE THIS CONST QUALIFIER, IS USED TO PLACE THE IMAGE IN M4 ROM */
#include SLAVE_IMAGE_FILE


#define TASK_A_ID 0x1234
#define ARGUMENT_A 0x567





void fpu_init(void);


void vCmdResponseTask( void *pvParameters ) {

	static uint32_t qEmpty = 0;
	static uint32_t qError = 0;
	static uint32_t validMsgs = 0;
	static uint32_t receivedMsgs = 0;
	maxMsg_t msgBack;

	uint32_t counter = COUNTER_VAL;
	qStat status;

	/* setup the queue systems */
	IPC_masterInitQueue(&_hostCmdBufferData[0], MASTER_CMDBUF_SIZE, &_hostMsgBufferData[0], SLAVE_MSGBUF_SIZE);

	/* download the slave image */
	IPC_downloadSlaveImage(SLAVE_ROM_START, &LR0[0], sizeof(LR0));

	/* start the slave processor */
	IPC_startSlave();
	
	// wait for the M0 to signal being ready via a message
	while(!IPC_msgPending()) __WFE();
	status = IPC_masterPopMsg(&msgBack);


	while(1) {
				
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

			// blink 				
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

		/* wait for a message to arrive */
		__WFE();

	};

}

void vCmdSendTask( void *pvParameters ) {

	static uint32_t qFull = 0;
	static uint32_t qError = 0;
	static uint32_t sentCmds = 0;
	rdCmd testRdCmd;
	wrCmd testWrCmd;
	qStat status;

	while(1) {
		
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

		/* throttle the number of sent messages */
		vTaskDelay(1/portTICK_RATE_MS);

	};
}



#define taskSend_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define taskReceive_PRIORITY			( tskIDLE_PRIORITY + 2 )

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




