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

#include "uarthandler.h"

/* ipc comms */
#include "ipc_mbx.h"
#include "platform_check.h"

#include "stdlib.h"
#include "string.h"
#include "stdio.h"

#define COUNTER_VAL 100 // check every half second	for sending

static volatile uint32_t counter = COUNTER_VAL ;


void vIOInit(void);
void initRiTimer(uint32_t timerInterval);
void initTimer0(uint32_t TimerInterval);

volatile uint32_t numMsgs = 0;
volatile uint32_t numTaskResponses = 0;
#define BUFSIZE	80
char printBuffer[BUFSIZE]; 
char lowerCaseString[] __attribute__ ((section ("lowerCaseString_sec"))) = "lpc4300 has two cores inside"  ;
char directString[] __attribute__ ((section ("directString_sec"))) = "heureka"  ;

mbxParam_t returnString;
mbxParam_t returnMessages;
mbxParam_t returnPower;
uint32_t localBase = 0;

volatile uint8_t sendA = 0;
volatile uint8_t sendB = 0;
volatile uint8_t sendC = 0;

/*****************************************************************************
**   Main Function  M0_main()
*****************************************************************************/
#define CLOCK_SPEED (120000000)
int main(void)
{		
	int id= 0;
			
	vIOInit();
	UART1Init(115200);

	writeString("\n\r--- M0 Started ---\n\r\n\r");

	/* SysTick is not implemented, use another timer like RITIMER if needed */
	initRiTimer(CLOCK_SPEED/1000);

	// init local mailbox system 
	IPC_initSlaveMbx(&Slave_CbackTable[0], (Mbx*) MASTER_MBX_START, (Mbx*) SLAVE_MBX_START);

	// signal back to M4 we are now ready
	IPC_sendMsg(MASTER_MBX_CMD, NOTIFY_SLAVE_STARTED, (msgId_t) 0, (mbxParam_t) 1);

	initTimer0(CLOCK_SPEED/1000);   // Khz tick

	while(1)
	{					
		/* wait for a message to arrive */
		if(counter > 0) {
			
			counter--; 
			
			// check for command MSG pending from M4
			if(mbxFlags[SLAVE_MBX_CMD] == MSG_PENDING)
			{
				IPC_lockMbx(SLAVE_MBX_CMD);
				
				numMsgs++;
				id = IPC_getMsgId(SLAVE_MBX_CMD);
	
				// print num messages if asked to
				if (PRINT_NUM_MESSAGES == IPC_getMsgType(SLAVE_MBX_CMD)) {

					returnMessages = IPC_getMbxParameter(SLAVE_MBX_CMD);					
					memset(&printBuffer[0], 0, BUFSIZE);
					sprintf(printBuffer,"\t\n\r--- M0 : CM4 processed %d calls ---",returnMessages);
					strcat(printBuffer,"\n\r\n\r");
					writeString(printBuffer);					
				};

				if (PRINT_WELCOME_MESSAGE == IPC_getMsgType(SLAVE_MBX_CMD)) {
					
					returnString = IPC_getMbxParameter(SLAVE_MBX_CMD);
					memset(&printBuffer[0], 0, BUFSIZE);
					strcat(printBuffer,(char*) returnString);
					writeString((char*) printBuffer);
				};
	
				IPC_freeMbx(SLAVE_MBX_CMD);
			}
			
			// check for taskA response MSG pending from M4
			if(mbxFlags[SLAVE_MBX_TASKD] == MSG_PENDING)
			{
				IPC_lockMbx(SLAVE_MBX_TASKD);
				numMsgs++;
				id = IPC_getMsgId(SLAVE_MBX_TASKD);
	
				// get the converted string
				if (DATA_RESULT == IPC_getMsgType(SLAVE_MBX_TASKD))
				{
					returnString = IPC_getMbxParameter(SLAVE_MBX_TASKD);
					numTaskResponses++;	
					memset(&printBuffer[0], 0, BUFSIZE);
					sprintf(printBuffer,"\t[ M4 :");
					strcat(printBuffer,(char*) returnString);
					strcat(printBuffer," ]\n\r\n\r");
					writeString((char*) printBuffer);	
				};
	
				IPC_freeMbx(SLAVE_MBX_TASKD);
			}

			// check for taskB response MSG pending from M4
			if(mbxFlags[SLAVE_MBX_TASKE] == MSG_PENDING)
			{
				IPC_lockMbx(SLAVE_MBX_TASKE);
				numMsgs++;
				id = IPC_getMsgId(SLAVE_MBX_TASKE);
	
				// get the power result
				if (DATA_RESULT == IPC_getMsgType(SLAVE_MBX_TASKE))
				{
					returnPower = IPC_getMbxParameter(SLAVE_MBX_TASKE);					
					memset(&printBuffer[0], 0, BUFSIZE);
					sprintf(printBuffer,"\t[ M4: %d ^ 3 = %d ]\n\r\n\r",localBase,returnPower);
					localBase++;
					writeString(printBuffer);
				};
	
				IPC_freeMbx(SLAVE_MBX_TASKE);
			}

			// check for taskC response MSG pending from M4
			if(mbxFlags[SLAVE_MBX_TASKF] == MSG_PENDING)
			{
				IPC_lockMbx(SLAVE_MBX_TASKF);
				numMsgs++;
				id = IPC_getMsgId(SLAVE_MBX_TASKF);
	
				// get the reversed string
				if (DATA_RESULT == IPC_getMsgType(SLAVE_MBX_TASKF))
				{
					returnString = IPC_getMbxParameter(SLAVE_MBX_TASKF);
					numTaskResponses++;	
					memset(&printBuffer[0], 0, BUFSIZE);
					sprintf(printBuffer,"\t[ M4:");
					strcat(printBuffer,(char*)returnString);
					strcat(printBuffer," ]\n\r\n\r");
					writeString((char*) printBuffer);		
				};
	
				IPC_freeMbx(SLAVE_MBX_TASKF);
			}
			__WFE();
		}
		else {

			// checked every hundred msec	
			if(sendA) {

				// query if we can send back another task to M4
				if(IPC_queryRemoteMbx(MASTER_MBX_TASKA) == READY) {

		   			memset(&printBuffer[0], 0, BUFSIZE);
					strcat(printBuffer,"> M0 Sending: ");
					strcat(printBuffer,lowerCaseString);
					strcat(printBuffer,"\n\r");
					writeString(printBuffer);			
					IPC_sendMsg(MASTER_MBX_TASKA, REQUEST_PROCESS_DATA, (msgId_t) id+1, (mbxParam_t) &lowerCaseString[0]);
				}
				
				sendA = 0;
				
			}
			
			if(sendB) {

				// query if we can send back another task to M4
				if(IPC_queryRemoteMbx(MASTER_MBX_TASKB) == READY) {
					memset(&printBuffer[0], 0, BUFSIZE);					
					sprintf(printBuffer,"> M0 Sending: request for pow(%d,3)\n\r",localBase);
					writeString(printBuffer);			
					IPC_sendMsg(MASTER_MBX_TASKB, REQUEST_PROCESS_DATA, (msgId_t) id+1, (mbxParam_t) localBase);
				}

				sendB = 0;			
			}
			
			if(sendC) {

				// query if we can send back another task to M4				
				if(IPC_queryRemoteMbx(MASTER_MBX_TASKC) == READY) {
		   			memset(&printBuffer[0], 0, BUFSIZE);
					strcat(printBuffer,"> M0 Sending: ");
					strcat(printBuffer,directString);
					strcat(printBuffer,"\n\r");
					writeString(printBuffer);			
					IPC_sendMsg(MASTER_MBX_TASKC, REQUEST_PROCESS_DATA, (msgId_t) id+1, (mbxParam_t) &directString[0]);				
				};
				
				sendC = 0;						
			}			
			
			
			counter = COUNTER_VAL;
		};

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
static volatile uint32_t u32Milliseconds = 1000;
void RIT_WDT_IRQHandler (void) 					
{           
	LPC_RITIMER->CTRL |=(1<<0);	// Clear interrupt flag
	if(u32Milliseconds > 0)
	{
		u32Milliseconds--; 
	}
	else
	{
		#if (PLATFORM == HITEX_BOARD)
		LPC_GPIO4->NOT |= (1UL << 1);    
		#endif
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

#define HZ1 1000
#define PERIOD	5
void initTimer0(uint32_t TimerInterval)
{
	
	LPC_TIMER0->PR = HZ1; // scale down to 1HZ = 120000
	
	// period 3Hz, reset
	LPC_TIMER0->MR[3] = (TimerInterval) * PERIOD; 

	// these give interrupts
	LPC_TIMER0->MR[2] = (90*PERIOD*HZ1);
	LPC_TIMER0->MR[1] = (60*PERIOD*HZ1);
	LPC_TIMER0->MR[0] = (30*PERIOD*HZ1);

	LPC_TIMER0->MCR = (((0x2) << 9)) | (((0x1) << 6)) | (((0x1) << 3)) | ((0x1) << 0);

	LPC_TIMER0->TC = 0;	

	LPC_TIMER0->IR = 0xFF;	

	NVIC_EnableIRQ(TIMER0_IRQn);
	
	LPC_TIMER0->TCR = 0x1;	 // start the timer
}

void TIMER0_IRQHandler(void)
{
	// check which match sent the interrupt
	if(LPC_TIMER0->IR & 0x1)
	{
		LPC_TIMER0->IR = 0x1;
		sendA = 1;
	}

	if(LPC_TIMER0->IR & 0x2)
	{
		LPC_TIMER0->IR = 0x2;
		sendB = 1;
	}

	if(LPC_TIMER0->IR & 0x4)
	{
		LPC_TIMER0->IR = 0x4;
		sendC = 1;
	}
}

/*****************************************************************************
**                            End Of File
*****************************************************************************/
