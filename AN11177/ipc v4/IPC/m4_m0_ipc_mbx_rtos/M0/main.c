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
#include "ipc_mbx.h"
#include "platform_check.h"

static volatile uint32_t u32Milliseconds = 1000;
static volatile uint32_t counter;
#define COUNTER_VAL 100

void vIOInit(void);
void initRiTimer(uint32_t timerInterval);

#include "FreeRTOS.h"
#include "task.h"

#define taskSendCmdSTACK_SIZE		( ( unsigned short ) configMINIMAL_STACK_SIZE )
#define taskRcvCmdSTACK_SIZE		( ( unsigned short ) configMINIMAL_STACK_SIZE )


void vCmdResponseTask( void *pvParameters ) {

	uint32_t id;

	uint32_t counter = COUNTER_VAL;

	while(1) {
		

		// check for command MSG pending
		if(mbxFlags[SLAVE_MBX_CMD] == MSG_PENDING)
		{
			IPC_lockMbx(SLAVE_MBX_CMD);
			id = IPC_getMsgId(SLAVE_MBX_CMD);
	
			// blink if asked to
			if (CMD_SLAVE_BLINK == IPC_getMsgType(SLAVE_MBX_CMD))
			{
				
				if(counter > 0) {
						counter--; 
				}
				else {
					#if (PLATFORM == HITEX_BOARD)
					// P4.1
					LPC_GPIO4->NOT |= (1UL << 1);     // Pin 16 of LCD connector
					#endif
					counter = COUNTER_VAL;
				};				
			
			};
	
			IPC_freeMbx(SLAVE_MBX_CMD);
		};

		/* wait for a message to arrive */
		__WFE();
	 };
}

void vCmdSendTask( void *pvParameters ) {

	static uint32_t id = 0;

	// check for the M4 mailbox to be initialized
	while(IPC_queryRemoteMbx(MASTER_MBX_CMD) != READY) ;
	
	// signal back to M4 we are now ready
	IPC_sendMsg(MASTER_MBX_CMD, RESPONSE_SLAVE_STARTED, (msgId_t) 0, (mbxParam_t) 1);

	while(1) {
		
		// query if we can send back some response to M4
		if(IPC_queryRemoteMbx(MASTER_MBX_CMD) == READY) {
			
			IPC_sendMsg(MASTER_MBX_CMD, RESPONSE_SLAVE_LED_BLINKED, (msgId_t) id+1, (mbxParam_t) id);
		};
		
		__WFE();
	};
}


/*****************************************************************************
**   Main Function  M0_main()
*****************************************************************************/
#define CLOCK_SPEED (120000000)
#define taskSend_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define taskReceive_PRIORITY			( tskIDLE_PRIORITY + 1 )


int main(void)
{		
	
	vIOInit();

	// init local mailbox system 
	IPC_initSlaveMbx(&Slave_CbackTable[0], (Mbx*) MASTER_MBX_START, (Mbx*) SLAVE_MBX_START);

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
