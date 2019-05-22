/***********************************************************************
 * Name: uarthandler.c
 *
 * Project: NXP LPC4300 LCD example
 *
 * Description: Routines to set up the UART1 and write to the port.
 *              This is a minimum set of functionality for this demo
 *              and not a generic UART driver.
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
                                                                                 
// Project includes
#include "LPC43xx.h"
#include "scu.h"

#include "uarthandler.h"
#include "type.h"




/*****************************************************************************
 * Function name:		UARTInit
 *
 * Descriptions:		Initialize UART1 port, setup pin select,
 *						clock, parity, stop bits, FIFO, etc.
 *
 * parameters:			portNum
 *                      Note: only the code for UART1 is implemented. 
 *
 * Returned value:		TRUE or FALSE (for later implementation)
 *
 *****************************************************************************/

uint32_t UART1Init(uint32_t baudrate )
{

#ifdef FRACTIONAL
	uint32_t uClk;
	uint32_t calcBaudrate = 0;
	uint32_t temp = 0;

	uint32_t mulFracDiv, dividerAddFracDiv;
	uint32_t divisor = 0;
	uint32_t mulFracDivOptimal = 1;
	uint32_t dividerAddOptimal = 0;
	uint32_t divisorOptimal = 0;

	uint32_t relativeError = 0;
	uint32_t relativeOptimalError = 100000;
#endif 	

    Status errorStatus = ERROR;

	#define UART_TER_TXEN			((uint8_t)(1<<7))



	// Select the correct UART1 RX/TX function for the LPC4350
	scu_pinmux(0xC, 13, UART_TX_PAD ,FUNC2);    // PC.13, U1_TXD
	scu_pinmux(0xC, 14, UART_RX_PAD, FUNC2);    // PC.14, U1_RXD

#ifdef FRACTIONAL
	// Work with the fractional devider
	uClk = UART1Frequency / 16;
	
	for (mulFracDiv = 1 ; mulFracDiv <= 15 ;mulFracDiv++)
	{
		for (dividerAddFracDiv = 0 ; dividerAddFracDiv <= 15 ; dividerAddFracDiv++)
		{
	  		temp = (mulFracDiv * uClk) / ((mulFracDiv + dividerAddFracDiv));

	  		divisor = temp / baudrate;
	  		if ((temp % baudrate) > (baudrate / 2))
				divisor++;

	  		if (divisor > 2 && divisor < 65536)
	  		{
				calcBaudrate = temp / divisor;

				if (calcBaudrate <= baudrate)
				{
		  			relativeError = baudrate - calcBaudrate;
				}
				else
				{
		  			relativeError = calcBaudrate - baudrate;
				}

				if ((relativeError < relativeOptimalError))
				{
		  			mulFracDivOptimal = mulFracDiv ;
		  			dividerAddOptimal = dividerAddFracDiv;
		  			divisorOptimal = divisor;
		  			relativeOptimalError = relativeError;
		  			if (relativeError == 0)
						break;
				}
	  		}
		}
		if (relativeError == 0)
	  	break;
	}

	if (relativeOptimalError < ((baudrate * UART_ACCEPTED_BAUDRATE_ERROR)/100))
	{
		LPC_UART1->LCR |= UART_LCR_DLAB_EN;
		LPC_UART1->DLM = UART_LOAD_DLM(divisorOptimal);
		LPC_UART1->DLL = UART_LOAD_DLL(divisorOptimal);
		LPC_UART1->LCR &= (~UART_LCR_DLAB_EN) & UART_LCR_BITMASK;
		LPC_UART1->FDR = 0x7C; 
		
		LPC_UART1->FCR = 0x07;					// Enable and reset TX and RX FIFO
    	LPC_UART1->TER |= UART_TER_TXEN;  		// TX is enabled by default after a reset
    	LPC_UART1->IER = IER_RBR; 				// Enable UART1 RX interrupt 

		errorStatus = SUCCESS;
	}

#else
    // For any reason here is also the fixed version for some baudrates/frequencies
	// 120MHz: DLM = 0 , DLL = 6 , FDR = 0xE5   921600 baud
	// 120MHz: DLM = 0 , DLL = 37 , FDR = 0x43	115200 baud
	//  72MHz: DLM = 0 , DLL = 4 , FDR = 0x51	921600 baud


    LPC_UART1->LCR = 0x83;							// 8 bits, no Parity, 1 Stop bit

	// Baudrate 115200 
	LPC_UART1->DLM = 0x00;
	LPC_UART1->DLL = 37;
	LPC_UART1->FDR = 0x43;

	LPC_UART1->LCR = 0x03;							// DLAB = 0
    LPC_UART1->FCR = 0x07;							// Enable and reset TX and RX FIFO
    LPC_UART1->TER |= UART_TER_TXEN;  				// TX is enabled by default after a reset
    LPC_UART1->IER = IER_RBR; 						// Enable UART1 RX interrupt 
	errorStatus = SUCCESS;
#endif
    
		
	if(errorStatus ==SUCCESS)
		return (TRUE);
	else
		return (FALSE);

}



/******************************************************************************
 *  Function:  hex_to_asci
 *
 *	Description:
 *
 ******************************************************************************/
char hex_to_ascii(char ch)
{
	if ( ch < 10) ch += 0x30;
	else ch += (0x41 - 0x0A);
	return (ch);
}




/******************************************************************************
 *  Function:  writeString
 *
 *	Description:
 *
 ******************************************************************************/
void writeString (const char *p )    
{
 
 while( *p )
	{	
		// Read the UART1 status register and check for HOLDEMPTY bit.
		while( (LPC_UART1->LSR & LSR_THRE) == 0 )
		{
		  ;
		}
		LPC_UART1->THR =  *p++ ; 
	}
}


/******************************************************************************
 *  Function:  getChar
 *
 *	Description:
 *
 ******************************************************************************/
unsigned char getChar (void)  
{                      
    return (LPC_UART1->RBR & UART_RBR_MASKBIT);	   // Read character from UART1
}






