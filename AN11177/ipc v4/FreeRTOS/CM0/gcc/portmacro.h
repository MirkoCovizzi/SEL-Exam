/*
    FreeRTOS V7.0.1 - Copyright (C) 2011 Real Time Engineers Ltd.


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


#ifndef PORTMACRO_H
#define PORTMACRO_H

#ifdef __cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------
 * Port specific definitions.
 *
 * The settings in this file configure FreeRTOS correctly for the
 * given hardware and compiler.
 *
 * These settings should not be altered.
 *-----------------------------------------------------------
 */

/* Type definitions. */
#define portCHAR                    char
#define portFLOAT                   float
#define portDOUBLE                  double
#define portLONG                    long
#define portSHORT                   short
#define portSTACK_TYPE              unsigned portLONG
#define portBASE_TYPE               long

#if( configUSE_16_BIT_TICKS == 1 )
	typedef unsigned portSHORT portTickType;
	#define portMAX_DELAY ( portTickType ) 0xffff
#else
	typedef unsigned portLONG portTickType;
	#define portMAX_DELAY ( portTickType ) 0xffffffff
#endif
/*-----------------------------------------------------------*/

/* Architecture specifics. */
#define portSTACK_GROWTH                        (-1)
#define portTICK_RATE_MS                        ((portTickType) 1000 / configTICK_RATE_HZ)
#define portBYTE_ALIGNMENT                      8
/*-----------------------------------------------------------*/


/* Scheduler utilities. */

#define portYIELDFromISR()                      \
        do {                                    \
            SCB->ICSR = SCB_ICSR_PENDSVSET_Msk; \
        } while(0)

#define portYIELD() portYIELDFromISR()

#define portEND_SWITCHING_ISR(xSwitchRequired)  \
        do {                                    \
            if (xSwitchRequired) {              \
                portYIELDFromISR();             \
            }                                   \
        } while(0)

/*-----------------------------------------------------------*/

/* Critical section management. */
#if (configEMULATE_BASEPRIO == 0)

	/* NOTE
	 * Interrupts of same priority cannot preempt each other.
	 * Still we need to disable interrupts globally in critical sections for the task context
	 * (Note that the Cortex-M0 doesn't have the BASEPRI mechanism of the Cortex-M3).
	 * In this configuration, interrupts are disabled globally in critical sections 
	 */
	#define portSET_INTERRUPT_MASK() 	__set_PRIMASK(1)
	#define portCLEAR_INTERRUPT_MASK() 	__set_PRIMASK(0)

#elif (configEMULATE_BASEPRIO == 1) 

	extern void vPortSetInterruptMask (void);
	extern void vPortClearInterruptMask (void);


/* NOTE
 * This will disable all interrupts that must be blocked while the kernel is in
 *  a critical section. This includes the SysTick exception handler, and all
 *  external interrupts that call the kernel API.
 *  Other interrupts are still allowed, but because of the M0 architecture they
 *  have to be disabled for a very short time during the process of disabling
 *  and enabling the critical interrupts. That short time period will be
 *  noticeable as jitter for the fast OS-independent interrupts.
 */
	
/* In this configuration, interrupts are selectively disabled. 
 * Only interrupts that can make OS calls are disabled, the others are kept enabled
 * Still we need to disable Os-related interrupts in critical sections for the task context
 * (Note that the Cortex-M0 doesn't have the BASEPRI mechanism of the Cortex-M3).
 */

	 /* the user needs to define this constant mask to provide information about */
	 /* which interrupts are allowed to make FreeRTOS API calls */
	#ifndef configBITMASK_SYSCALL_IRQ
		#error "configBITMASK_SYSCALL_IRQ mask not defined, please define in FreeRTOSConfig.h"
	#endif

	#define portSET_INTERRUPT_MASK()                vPortSetInterruptMask()
	#define portCLEAR_INTERRUPT_MASK()              vPortClearInterruptMask()

#else
	#error "configEMULATE_BASEPRIO should be set to 1 or 0"
#endif /* configEMULATE_BASEPRIO */

#if (configKERNEL_INTERRUPT_PRIORITY == configMAX_SYSCALL_INTERRUPT_PRIORITY)

	/* should not need to disable interrupts within an ISR context if OS related */
	/* ISR, PendSV and SysTick have all the same priority. Is enforced by hardware */
	/* in this case there should be nothing to do */
	#define portSET_INTERRUPT_MASK_FROM_ISR()		0;
	#define portCLEAR_INTERRUPT_MASK_FROM_ISR(x)	(void)x

#else
		
	#define portSET_INTERRUPT_MASK_FROM_ISR()       0;portSET_INTERRUPT_MASK()
	#define portCLEAR_INTERRUPT_MASK_FROM_ISR(x)    portCLEAR_INTERRUPT_MASK();(void)x

#endif	/* if (configKERNEL_INTERRUPT_PRIORITY == configMAX_SYSCALL_INTERRUPT_PRIORITY) */

extern void vPortEnterCritical (void);
extern void vPortExitCritical (void);

#define portDISABLE_INTERRUPTS()                portSET_INTERRUPT_MASK()
#define portENABLE_INTERRUPTS()                 portCLEAR_INTERRUPT_MASK()
#define portENTER_CRITICAL()                    vPortEnterCritical()
#define portEXIT_CRITICAL()                     vPortExitCritical()

/*-----------------------------------------------------------*/

/* Task function macros as described on the FreeRTOS.org WEB site. */
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters ) void vFunction( void *pvParameters )
#define portTASK_FUNCTION( vFunction, pvParameters ) void vFunction( void *pvParameters )

#define portNOP()

#ifdef __cplusplus
}
#endif

#endif /* PORTMACRO_H */

