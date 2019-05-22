/*****************************************************************************
 *   uart.c:  UART API file for NXP LPC43xx Family Microprocessors
 *
 *   Copyright(C) 2011, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2011.06.21  ver 1.00    Initial version
 *
******************************************************************************/

#if defined CORE_M4
#include "LPC43xx.h"                    /* LPC43xx definitions                */
#endif

#ifdef CORE_M0
#include "LPC43xx_M0.h"                /* LPC43xx definitions                */
#endif

#include "config.h"
#include "type.h"
#include "uart.h"
#include "scu.h"

volatile uint32_t UART0Status, UART1Status, UART2Status, UART3Status;
volatile uint8_t UART0TxEmpty = 1, UART1TxEmpty = 1, UART2TxEmpty = 1, UART3TxEmpty = 1;
volatile uint8_t UART0Buffer[RETARGET_UART_BUFSIZE], UART1Buffer[RETARGET_UART_BUFSIZE],
	UART2Buffer[RETARGET_UART_BUFSIZE], UART3Buffer[RETARGET_UART_BUFSIZE];
volatile uint32_t UART0Count = 0, UART1Count = 0, UART2Count = 0, UART3Count = 0;

/*****************************************************************************
** Function name:		UART0_IRQHandler
**
** Descriptions:		UART0 interrupt handler
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void UART0_IRQHandler (void) 
{
  uint8_t IIRValue, LSRValue;
  uint8_t Dummy = Dummy;
	
  IIRValue = LPC_USART0->IIR;
    
  IIRValue >>= 1;			/* skip pending bit in IIR */
  IIRValue &= 0x07;			/* check bit 1~3, interrupt identification */
  if ( IIRValue == IIR_RLS )		/* Receive Line Status */
  {
	LSRValue = LPC_USART0->LSR;
	/* Receive Line Status */
	if ( LSRValue & (LSR_OE|LSR_PE|LSR_FE|LSR_RXFE|LSR_BI) )
	{
	  /* There are errors or break interrupt */
	  /* Read LSR will clear the interrupt */
	  UART0Status = LSRValue;
	  Dummy = LPC_USART0->RBR;		/* Dummy read on RX to clear 
							interrupt, then bail out */
	  return;
	}
	if ( LSRValue & LSR_RDR )	/* Receive Data Ready */			
	{
	  /* If no error on RLS, normal ready, save into the data buffer. */
	  /* Note: read RBR will clear the interrupt */
	  UART0Buffer[UART0Count] = LPC_USART0->RBR;
	  UART0Count++;
	  if ( UART0Count == RETARGET_UART_BUFSIZE )
	  {
		UART0Count = 0;		/* buffer overflow */
	  }	
	}
  }
  else if ( IIRValue == IIR_RDA )	/* Receive Data Available */
  {
	/* Receive Data Available */
	UART0Buffer[UART0Count] = LPC_USART0->RBR;
	UART0Count++;
	if ( UART0Count == RETARGET_UART_BUFSIZE )
	{
	  UART0Count = 0;		/* buffer overflow */
	}
  }
  else if ( IIRValue == IIR_CTI )	/* Character timeout indicator */
  {
	/* Character Time-out indicator */
	UART0Status |= 0x100;		/* Bit 9 as the CTI error */
  }
  else if ( IIRValue == IIR_THRE )	/* THRE, transmit holding register empty */
  {
	/* THRE interrupt */
	LSRValue = LPC_USART0->LSR;		/* Check status in the LSR to see if
									valid data in U0THR or not */
	if ( LSRValue & LSR_THRE )
	{
	  UART0TxEmpty = 1;
	}
	else
	{
	  UART0TxEmpty = 0;
	}
  }
  return;    
}

/*****************************************************************************
** Function name:		UART1_IRQHandler
**
** Descriptions:		UART1 interrupt handler
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void UART1_IRQHandler (void) 
{
  uint8_t IIRValue, LSRValue;
  uint8_t Dummy = Dummy;
	
  IIRValue = LPC_UART1->IIR;
    
  IIRValue >>= 1;			/* skip pending bit in IIR */
  IIRValue &= 0x07;			/* check bit 1~3, interrupt identification */
  if ( IIRValue == IIR_RLS )		/* Receive Line Status */
  {
	LSRValue = LPC_UART1->LSR;
	/* Receive Line Status */
	if ( LSRValue & (LSR_OE|LSR_PE|LSR_FE|LSR_RXFE|LSR_BI) )
	{
	  /* There are errors or break interrupt */
	  /* Read LSR will clear the interrupt */
	  UART1Status = LSRValue;
	  Dummy = LPC_UART1->RBR;		/* Dummy read on RX to clear 
								interrupt, then bail out */
	  return;
	}
	if ( LSRValue & LSR_RDR )	/* Receive Data Ready */			
	{
	  /* If no error on RLS, normal ready, save into the data buffer. */
	  /* Note: read RBR will clear the interrupt */
	  UART1Buffer[UART1Count] = LPC_UART1->RBR;
	  UART1Count++;
	  if ( UART1Count == RETARGET_UART_BUFSIZE )
	  {
		UART1Count = 0;		/* buffer overflow */
	  }	
	}
  }
  else if ( IIRValue == IIR_RDA )	/* Receive Data Available */
  {
	/* Receive Data Available */
	UART1Buffer[UART1Count] = LPC_UART1->RBR;
	UART1Count++;
	if ( UART1Count == RETARGET_UART_BUFSIZE )
	{
	  UART1Count = 0;		/* buffer overflow */
	}
  }
  else if ( IIRValue == IIR_CTI )	/* Character timeout indicator */
  {
	/* Character Time-out indicator */
	UART1Status |= 0x100;		/* Bit 9 as the CTI error */
  }
  else if ( IIRValue == IIR_THRE )	/* THRE, transmit holding register empty */
  {
	/* THRE interrupt */
	LSRValue = LPC_UART1->LSR;		/* Check status in the LSR to see if
								valid data in U0THR or not */
	if ( LSRValue & LSR_THRE )
	{
	  UART1TxEmpty = 1;
	}
	else
	{
	  UART1TxEmpty = 0;
	}
  }
  return;
}

/*****************************************************************************
** Function name:		UART2_IRQHandler
**
** Descriptions:		UART2 interrupt handler
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void UART2_IRQHandler (void) 
{
  uint8_t IIRValue, LSRValue;
  uint8_t Dummy = Dummy;
	
  IIRValue = LPC_USART2->IIR;
    
  IIRValue >>= 1;			/* skip pending bit in IIR */
  IIRValue &= 0x07;			/* check bit 1~3, interrupt identification */
  if ( IIRValue == IIR_RLS )		/* Receive Line Status */
  {
	LSRValue = LPC_USART2->LSR;
	/* Receive Line Status */
	if ( LSRValue & (LSR_OE|LSR_PE|LSR_FE|LSR_RXFE|LSR_BI) )
	{
	  /* There are errors or break interrupt */
	  /* Read LSR will clear the interrupt */
	  UART2Status = LSRValue;
	  Dummy = LPC_USART2->RBR;		/* Dummy read on RX to clear 
							interrupt, then bail out */
	  return;
	}
	if ( LSRValue & LSR_RDR )	/* Receive Data Ready */			
	{
	  /* If no error on RLS, normal ready, save into the data buffer. */
	  /* Note: read RBR will clear the interrupt */
	  UART2Buffer[UART2Count] = LPC_USART2->RBR;
	  UART2Count++;
	  if ( UART2Count == RETARGET_UART_BUFSIZE )
	  {
		UART2Count = 0;		/* buffer overflow */
	  }	
	}
  }
  else if ( IIRValue == IIR_RDA )	/* Receive Data Available */
  {
	/* Receive Data Available */
	UART2Buffer[UART2Count] = LPC_USART2->RBR;
	UART2Count++;
	if ( UART2Count == RETARGET_UART_BUFSIZE )
	{
	  UART2Count = 0;		/* buffer overflow */
	}
  }
  else if ( IIRValue == IIR_CTI )	/* Character timeout indicator */
  {
	/* Character Time-out indicator */
	UART2Status |= 0x100;		/* Bit 9 as the CTI error */
  }
  else if ( IIRValue == IIR_THRE )	/* THRE, transmit holding register empty */
  {
	/* THRE interrupt */
	LSRValue = LPC_USART2->LSR;		/* Check status in the LSR to see if
									valid data in U0THR or not */
	if ( LSRValue & LSR_THRE )
	{
	  UART2TxEmpty = 1;
	}
	else
	{
	  UART2TxEmpty = 0;
	}
  }
  return;    
}

/*****************************************************************************
** Function name:		UART3_IRQHandler
**
** Descriptions:		UART3 interrupt handler
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void UART3_IRQHandler (void) 
{
  uint8_t IIRValue, LSRValue;
  uint8_t Dummy = Dummy;
	
  IIRValue = LPC_USART3->IIR;
    
  IIRValue >>= 1;			/* skip pending bit in IIR */
  IIRValue &= 0x07;			/* check bit 1~3, interrupt identification */
  if ( IIRValue == IIR_RLS )		/* Receive Line Status */
  {
	LSRValue = LPC_USART3->LSR;
	/* Receive Line Status */
	if ( LSRValue & (LSR_OE|LSR_PE|LSR_FE|LSR_RXFE|LSR_BI) )
	{
	  /* There are errors or break interrupt */
	  /* Read LSR will clear the interrupt */
	  UART3Status = LSRValue;
	  Dummy = LPC_USART3->RBR;		/* Dummy read on RX to clear 
							interrupt, then bail out */
	  return;
	}
	if ( LSRValue & LSR_RDR )	/* Receive Data Ready */			
	{
	  /* If no error on RLS, normal ready, save into the data buffer. */
	  /* Note: read RBR will clear the interrupt */
	  UART3Buffer[UART3Count] = LPC_USART3->RBR;
	  UART3Count++;
	  if ( UART3Count == RETARGET_UART_BUFSIZE )
	  {
		UART3Count = 0;		/* buffer overflow */
	  }	
	}
  }
  else if ( IIRValue == IIR_RDA )	/* Receive Data Available */
  {
	/* Receive Data Available */
	UART3Buffer[UART3Count] = LPC_USART3->RBR;
	UART3Count++;
	if ( UART3Count == RETARGET_UART_BUFSIZE )
	{
	  UART3Count = 0;		/* buffer overflow */
	}
  }
  else if ( IIRValue == IIR_CTI )	/* Character timeout indicator */
  {
	/* Character Time-out indicator */
	UART3Status |= 0x100;		/* Bit 9 as the CTI error */
  }
  else if ( IIRValue == IIR_THRE )	/* THRE, transmit holding register empty */
  {
	/* THRE interrupt */
	LSRValue = LPC_USART3->LSR;		/* Check status in the LSR to see if
									valid data in U0THR or not */
	if ( LSRValue & LSR_THRE )
	{
	  UART3TxEmpty = 1;
	}
	else
	{
	  UART3TxEmpty = 0;
	}
  }
  return;    
}

/*****************************************************************************
** Function name:		UARTInit
**
** Descriptions:		Initialize UART port, setup pin select,
**						clock, parity, stop bits, FIFO, etc.
**
** parameters:			portNum(0 or 1) and UART baudrate
** Returned value:		true or false, return false only if the 
**						interrupt handler can't be installed to the 
**						VIC table
** 
*****************************************************************************/
uint32_t UARTInit( uint32_t PortNum, uint32_t baudrate )
{
  uint32_t Fdiv;

  if ( PortNum == 0 )
  {
	scu_pinmux(0xF,10,MD_PLN,FUNC1);		/* PF_10 U0_TXD */
	scu_pinmux(0xF,11,MD_PLN|MD_EZI,FUNC1);	/* PF_11 U0_RXD */

    LPC_USART0->LCR = 0x83;		/* 8 bits, no Parity, 1 Stop bit */
	Fdiv = ( UART0Frequency / 16 ) / baudrate ;	/*baud rate */
    LPC_USART0->DLM = Fdiv / 256;							
    LPC_USART0->DLL = Fdiv % 256;
	LPC_USART0->LCR = 0x03;		/* DLAB = 0 */
    LPC_USART0->FCR = 0x07;		/* Enable and reset TX and RX FIFO. */

   	NVIC_EnableIRQ(USART0_IRQn);

    LPC_USART0->IER = IER_RBR | IER_THRE | IER_RLS;	/* Enable UART0 interrupt */
    return (TRUE);
  }
  else if ( PortNum == 1 )
  {
	scu_pinmux(0xC,13,MD_PLN,FUNC2);		/* PC_13 U1_TXD */
	scu_pinmux(0xC,14,MD_PLN|MD_EZI,FUNC2);	/* PC_14 U1_RXD */


    LPC_UART1->LCR = 0x83;		/* 8 bits, no Parity, 1 Stop bit */
	Fdiv = ( UART1Frequency / 16 ) / baudrate ;	/*baud rate */
    LPC_UART1->DLM = Fdiv / 256;							
    LPC_UART1->DLL = Fdiv % 256;
	LPC_UART1->LCR = 0x03;		/* DLAB = 0 */
    LPC_UART1->FCR = 0x07;		/* Enable and reset TX and RX FIFO. */

   	NVIC_EnableIRQ(UART1_IRQn);

    LPC_UART1->IER = IER_RBR | IER_THRE | IER_RLS;	/* Enable UART1 interrupt */
    return (TRUE);
  }
  else if ( PortNum == 2 )
  {
	scu_pinmux(0x1,15,MD_PLN,FUNC1);		/* P1_15 U2_TXD */
	scu_pinmux(0x1,16,MD_PLN|MD_EZI,FUNC1);	/* P1_16 U2_RXD */
	
    LPC_USART2->LCR = 0x83;		/* 8 bits, no Parity, 1 Stop bit */
	Fdiv = ( UART2Frequency / 16 ) / baudrate ;	/*baud rate */
    LPC_USART2->DLM = Fdiv / 256;							
    LPC_USART2->DLL = Fdiv % 256;
	LPC_USART2->LCR = 0x03;		/* DLAB = 0 */
    LPC_USART2->FCR = 0x07;		/* Enable and reset TX and RX FIFO. */

#if defined CORE_M4
   	NVIC_EnableIRQ(USART2_IRQn);
#endif
#ifdef CORE_M0
   	NVIC_EnableIRQ(USART2_C_CAN1_IRQn);
#endif

    LPC_USART2->IER = IER_RBR | IER_THRE | IER_RLS;	/* Enable UART2 interrupt */
    return (TRUE);
  }
  else if ( PortNum == 3 )
  {
	scu_pinmux(0xF, 2,MD_PLN,FUNC1);		/* PF_2 U3_TXD */
	scu_pinmux(0xF, 3,MD_PLN|MD_EZI,FUNC1);	/* PF_3 U3_RXD */
	
    LPC_USART3->LCR = 0x83;		/* 8 bits, no Parity, 1 Stop bit */
	Fdiv = ( UART3Frequency / 16 ) / baudrate ;	/*baud rate */
    LPC_USART3->DLM = Fdiv / 256;							
    LPC_USART3->DLL = Fdiv % 256;
	LPC_USART3->LCR = 0x03;		/* DLAB = 0 */
    LPC_USART3->FCR = 0x07;		/* Enable and reset TX and RX FIFO. */

   	NVIC_EnableIRQ(USART3_IRQn);

    LPC_USART3->IER = IER_RBR | IER_THRE | IER_RLS;	/* Enable UART3 interrupt */
    return (TRUE);
  }
  return( FALSE ); 
}

/*****************************************************************************
** Function name:		UARTSend
**
** Descriptions:		Send a block of data to the UART 0 port based
**						on the data length
**
** parameters:			portNum, buffer pointer, and data length
** Returned value:		None
** 
*****************************************************************************/
void UARTSend( uint32_t portNum, uint8_t *BufferPtr, uint32_t Length )
{
  if ( portNum == 0 )
  {
    while ( Length != 0 )
    {
	  /* THRE status, contain valid data */
	  while ( !(UART0TxEmpty & 0x01) );	
	  LPC_USART0->THR = *BufferPtr;
	  UART0TxEmpty = 0;	/* not empty in the THR until it shifts out */
	  BufferPtr++;
	  Length--;
	}
  }
  else if ( portNum == 1 )
  {
	while ( Length != 0 )
    {
	  /* THRE status, contain valid data */
	  while ( !(UART1TxEmpty & 0x01) );	
	  LPC_UART1->THR = *BufferPtr;
	  UART1TxEmpty = 0;	/* not empty in the THR until it shifts out */
	  BufferPtr++;
	  Length--;
    }
  }
  else if ( portNum == 2 )
  {
	while ( Length != 0 )
    {
	  /* THRE status, contain valid data */
	  while ( !(UART2TxEmpty & 0x01) );	
	  LPC_USART2->THR = *BufferPtr;
	  UART2TxEmpty = 0;	/* not empty in the THR until it shifts out */
	  BufferPtr++;
	  Length--;
    }
  }
  else if ( portNum == 3 )
  {
	while ( Length != 0 )
    {
	  /* THRE status, contain valid data */
	  while ( !(UART3TxEmpty & 0x01) );	
	  LPC_USART3->THR = *BufferPtr;
	  UART3TxEmpty = 0;	/* not empty in the THR until it shifts out */
	  BufferPtr++;
	  Length--;
    }
  }
  return;
}

/******************************************************************************
**                            End Of File
******************************************************************************/
