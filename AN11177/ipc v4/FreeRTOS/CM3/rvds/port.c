/*
    FreeRTOS V7.0.2 - Copyright (C) 2011 Real Time Engineers Ltd.
	

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    >>>NOTE<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the ARM CM3 port.
 *----------------------------------------------------------*/

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Constants required to set up the initial stack. */
#define portINITIAL_XPSR			( 0x01000000 )

/* Each task maintains its own interrupt status in the critical nesting
variable. */
//static unsigned portBASE_TYPE uxCriticalNesting = 0xaaaaaaaa;
static unsigned portBASE_TYPE uxCriticalNesting = 0xaaaaaaaa;
/*
 * Exception handlers.
 */
void xPortPendSVHandler( void );
void xPortSysTickHandler( void );
void vPortSVCHandler( void );

/*
 * Start first task is a separate function so it can be tested in isolation.
 */
void vPortStartFirstTask( void );

/*-----------------------------------------------------------*/

/* 
 * See header file for description. 
 */
portSTACK_TYPE *pxPortInitialiseStack( portSTACK_TYPE *pxTopOfStack, pdTASK_CODE pxCode, void *pvParameters )
{
	/* Simulate the stack frame as it would be created by a context switch
	interrupt. */
	pxTopOfStack--; /* Offset added to account for the way the MCU uses the stack on entry/exit of interrupts. */
	*pxTopOfStack = portINITIAL_XPSR;	/* xPSR */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) pxCode;	/* PC */
	pxTopOfStack--;
	*pxTopOfStack = 0;	/* LR */
	pxTopOfStack -= 5;	/* R12, R3, R2 and R1. */
	*pxTopOfStack = ( portSTACK_TYPE ) pvParameters;	/* R0 */
	pxTopOfStack -= 8;	/* R11, R10, R9, R8, R7, R6, R5 and R4. */

	return pxTopOfStack;
}
/*-----------------------------------------------------------*/
/* called only once at the beginning to hand off */
/* to the first task which will start running */
/* basically it gets the task stack pointer, pops off the (dummy) */
/* stack frame from the TCB and branches to the task entry point */
__asm void vPortSVCHandler( void )
{
	PRESERVE8

	ldr	r3, =pxCurrentTCB		/* Restore the context. */
	ldr r1, [r3]				/* Use pxCurrentTCBConst to get the pxCurrentTCB address. */
	ldr r0, [r1]				/* The first item in pxCurrentTCB is the task top of stack. */
	ldmia r0!, {r4-r11}		/* Pop the registers that are not automatically saved on exception entry and the critical nesting count. */
	msr psp, r0					/* Restore the task stack pointer. */
	mov r0, #0
	msr	basepri, r0
	orr r14, #0xd				
	bx r14
	
	align 4						
}
/*-----------------------------------------------------------*/

__asm void vPortStartFirstTask( void )
{
	PRESERVE8

	/* Use the NVIC offset register to locate the stack. */
	ldr r0, =0xE000ED08
	ldr r0, [r0]
	ldr r0, [r0]
	/* Set the msp back to the start of the stack. */
	msr msp, r0
	/* Globally enable interrupts. */
	cpsie i
	/* Call SVC to start the first task. */
	svc 0
	nop

	align 4
}
/*-----------------------------------------------------------*/

/* 
 * See header file for description. 
 */
portBASE_TYPE xPortStartScheduler( void )
{
	/* Make PendSV and SysTick the same priority as the kernel. */
	NVIC_SetPriority(PendSV_IRQn, configKERNEL_INTERRUPT_PRIORITY);

	/* Start the timer that generates the tick ISR.  Interrupts are disabled
	here already. */
	SysTick_Config(configCPU_CLOCK_HZ/configTICK_RATE_HZ);
	
	/* set priority explicitly since SysTick_Config defaults internally to 1 */
	NVIC_SetPriority(SysTick_IRQn, configKERNEL_INTERRUPT_PRIORITY);

	/* Initialise the critical nesting count ready for the first task. */
	uxCriticalNesting = 0;

	/* Start the first task. */
	vPortStartFirstTask();

	/* Should not get here! */
	return 0;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
	/* It is unlikely that the CM3 port will require this function as there
	is nothing to return to.  */
}
/*-----------------------------------------------------------*/

void vPortYieldFromISR( void )
{
	/* Set a PendSV to request a context switch. */
	SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}
/*-----------------------------------------------------------*/

void vPortEnterCritical( void )
{
	portDISABLE_INTERRUPTS();
	uxCriticalNesting++;
}
/*-----------------------------------------------------------*/

void vPortExitCritical( void )
{
	uxCriticalNesting--;
	if( uxCriticalNesting == 0 )
	{
		portENABLE_INTERRUPTS();
	}
}
/*-----------------------------------------------------------*/
/* need to use the shifted priority here since it is applied to the register BASEPRI */
/* configMAX_SYSCALL_INTERRUPT_PRIORITY is CMSIS like, so from 0 to 7 */
/* value in register need to be shifted */
#define config_OS_MAX_SYSCALL (configMAX_SYSCALL_INTERRUPT_PRIORITY << (8 - __NVIC_PRIO_BITS) & 0xFF)

__asm void xPortPendSVHandler( void )
{
	extern uxCriticalNesting;
	extern pxCurrentTCB;
	extern vTaskSwitchContext;

	PRESERVE8

	mrs r0, psp						 

	ldr	r3, =pxCurrentTCB		 	 /* Get the location of the current TCB. */
	ldr	r2, [r3]						

	stmdb r0!, {r4-r11}				 /* Save the remaining registers. */
	str r0, [r2]					 /* Save the new top of stack into the first member of the TCB. */

	stmdb sp!, {r3, r14}		
	mov r0, #config_OS_MAX_SYSCALL
	msr basepri, r0		 
	bl vTaskSwitchContext
	mov r0, #0
	msr basepri, r0
	ldmia sp!, {r3, r14}			

	ldr r1, [r3]					 
	ldr r0, [r1]			/* The first item in pxCurrentTCB is the task top of stack. */
	ldmia r0!, {r4-r11}		/* Pop the registers and the critical nesting count. */
	msr psp, r0						 
	bx r14
	nop

	align 4
}
/*-----------------------------------------------------------*/

void xPortSysTickHandler( void )
{
unsigned long ulDummy;

	/* If using preemption, also force a context switch. */
	#if configUSE_PREEMPTION == 1
		SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
	#endif

	ulDummy = portSET_INTERRUPT_MASK_FROM_ISR();
	{
		vTaskIncrementTick();
	}
	portCLEAR_INTERRUPT_MASK_FROM_ISR( ulDummy );
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

__asm void vPortSetInterruptMask( void )
{
	PRESERVE8

	push { r0 }
	mov r0, #config_OS_MAX_SYSCALL
	msr basepri, r0
	pop { r0 }
	bx r14

	align 4
}

/*-----------------------------------------------------------*/

__asm void vPortClearInterruptMask( void )
{
	PRESERVE8

	push { r0 }
	mov r0, #0
	msr basepri, r0
	pop { r0 }
	bx r14

	align 4
}

