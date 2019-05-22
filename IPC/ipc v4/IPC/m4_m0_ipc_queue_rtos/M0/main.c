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

#include "FreeRTOS.h"
#include "task.h"

#define taskSendCmdSTACK_SIZE		( ( unsigned short ) configMINIMAL_STACK_SIZE )
#define taskRcvCmdSTACK_SIZE		( ( unsigned short ) configMINIMAL_STACK_SIZE )


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

#define COUNTER_VAL 100

void vCmdResponseTask( void *pvParameters ) {

	qStat status;
	static uint32_t validCmds = 0;
	static uint32_t receivedCmds = 0;

	static uint32_t qEmpty = 0;
	static uint32_t qError = 0;
	static uint32_t counter = COUNTER_VAL;

	while(1) {
		
		/* wait for a message to arrive */
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

		if(counter > 0) {
				counter--; 
		}
		else {
			#if (PLATFORM == HITEX_BOARD)
			// Pin 16 of LCD connector, an LED can be connected on 15-16
			LPC_GPIO4->NOT |= (1UL << 1);     // Pin 16 of LCD connector
			#endif
			counter = COUNTER_VAL;
		};


	 };
}

void vCmdSendTask( void *pvParameters ) {

	qStat status;
	
	static uint32_t sentMsgs = 0;	
	static uint32_t qFull = 0;
	static uint32_t qError = 0;
	
	
	// signal to the M0 being ready via a message
	MAKE_SRV_MSG_HEADER(serviceMessage,TASK_B_ID,SERVICE_TYPE);
	do {
		status = IPC_slavePushMsg(&serviceMessage);
	} while(status != QINSERT);
	
	IPC_msgNotifyMaster();


	while(1) {
		
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
		
		/* throttle number of sent messages */
		vTaskDelay(1/portTICK_RATE_MS);
		
	};
}

#define taskSend_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define taskReceive_PRIORITY			( tskIDLE_PRIORITY + 1 )

/*****************************************************************************
**   Main Function  M0_main()
*****************************************************************************/
int main(void)
{				
	vIOInit();
	

	// init local mailbox system 
	IPC_slaveInitQueue();

	xTaskCreate( vCmdResponseTask, "AnsTask", taskRcvCmdSTACK_SIZE, ( void * ) NULL, taskReceive_PRIORITY, NULL );
	xTaskCreate( vCmdSendTask, "SendTask", taskSendCmdSTACK_SIZE, ( void * ) NULL, taskSend_PRIORITY, NULL );

	/* Start the scheduler. */
	vTaskStartScheduler();

    /* Will only get here if there was insufficient memory to create the idle
    task.  The idle task is created within vTaskStartScheduler(). */
	for( ;; );
		
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
/*****************************************************************************
**                            End Of File
*****************************************************************************/
