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

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * M0 Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *----------------------------------------------------------*/
#include "lpc11xx.h"

#define configUSE_PREEMPTION		1
#define configUSE_IDLE_HOOK			0
#define configMAX_PRIORITIES		( ( unsigned portBASE_TYPE ) 5 )
#define configUSE_TICK_HOOK			1
#define configCPU_CLOCK_HZ			( ( unsigned long ) 120000000 )
#define configTICK_RATE_HZ			( ( portTickType ) 1000 )
#define configMINIMAL_STACK_SIZE	( ( unsigned short ) 400 )
#define configTOTAL_HEAP_SIZE		( ( size_t ) ( 19 * 1024 ) )
#define configMAX_TASK_NAME_LEN		( 12 )
#define configUSE_TRACE_FACILITY	1
#define configUSE_16_BIT_TICKS		0
#define configIDLE_SHOULD_YIELD		0
#define configUSE_CO_ROUTINES 		0
#define configUSE_MUTEXES			1

#define configMAX_CO_ROUTINE_PRIORITIES ( 2 )

#define configUSE_COUNTING_SEMAPHORES 	0
#define configUSE_ALTERNATIVE_API 		0
#define configCHECK_FOR_STACK_OVERFLOW	2
#define configUSE_RECURSIVE_MUTEXES		1
#define configQUEUE_REGISTRY_SIZE		10
#define configGENERATE_RUN_TIME_STATS	0

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */

#define INCLUDE_vTaskPrioritySet			1
#define INCLUDE_uxTaskPriorityGet			1
#define INCLUDE_vTaskDelete					1
#define INCLUDE_vTaskCleanUpResources		0
#define INCLUDE_vTaskSuspend				1
#define INCLUDE_vTaskDelayUntil				1
#define INCLUDE_vTaskDelay					1
#define INCLUDE_uxTaskGetStackHighWaterMark	1


/* Interrupts at higher priority than configMAX_SYSCALL_INTERRUPT_PRIORITY */
/* can still preempt the scheduler but as for FreeRtos design are NOT allowed to call OS APIs. */
#define configKERNEL_INTERRUPT_PRIORITY  (3)  
#define configMAX_SYSCALL_INTERRUPT_PRIORITY  (2)



/* define if emulation of PRIMASK is desired for low latency non-Os interrupts or */
/* if interrupts can be disabled globally during task switches in PendSV call */
/* 1 = emulate BASEPRI register in sw, by disabling all interrupts which can make OS calls */
/* 0 = disable interrupts globally during critical sections and task switches, see notes */
#define configEMULATE_BASEPRIO (1)

/* NOTES: */
/* disabling interrupts globally is quicker, but adds jitter / latency to the non-OS ISRs */

/* without the sw mask: */
/* interrupts are disabled globally in xPortPendSVHandler only if */
/* configKERNEL_INTERRUPT_PRIORITY is different that configMAX_SYSCALL_INTERRUPT_PRIORITY */
/* All the critical sections are always protected with a global interrupt disable / enable */
/* Entering critical sections is done with 1 instruction (__set_PRIMASK()) */
/* But there is high Jitter introduced within the non-OS interrupts, depending on the duration of */
/* the critical section which can happen at the task or at the scheduler level */

/* with the "sw BASEPRIO mask": */
/* only interrupts which can make OS API calls are disabled in critical sections */
/* still the interrupts need to be disabled globally - for the shortest time possible - to make the changes */


/* The user MUST define the OS specific mask in case non-Os interrupts should stay active */
/* during Os critical sections */
/* the mask should include -all- interrupt routines which can make FreeRTOS API calls */
/* to compose the mask, just OR together the interrupt numbers within the macro */
/* !!! NOTE: if a normal timer is used as SysTick, it needs to be included within configOS_INTERRUPTS !!!*/
/* the below is just an example */
#define configBITMASK_SYSCALL_IRQ  \
		(1 << UART_IRQn)			\
    |   (1 << I2C_IRQn)             \
    |   (1 << CAN_IRQn)             \
    |   (1 << ADC_IRQn)             \
    |   (1 << TIMER_32_1_IRQn)      \
    |   (1 << SSP0_IRQn)

/* define if SysTick is implemented / used on the platform, or if another timer source is used */
/* !!! NOTE: if a normal timer is used as SysTick, it needs to be included within configOS_INTERRUPTS !!!*/
#define configUSE_SYSTICK (0)

/* SysTick must be disabled in critical sections. Reading SysTick->CTRL has side effects,
 * so we can only write to this register to disable/enable the interrupt. Therefore we need
 * to know the SysTick clock source as a config parameter!
 * Set this to either 0 (external clock) or 1 (internal clock, the default)
 * Not applicable if another clock source is used 
*/
#define configSYSTICK_CLKSOURCE	(1)

/*-----------------------------------------------------------
 * Macros required to setup the timer for the OS
 *-----------------------------------------------------------*/
extern void vConfigureTimerForOS( void );
#define portCONFIGURE_TIMER_FOR_OS() vConfigureTimerForOS()

extern void vQuitOsTimerInt(void);
#define portQUIT_OS_TIMER_INT() vQuitOsTimerInt()

/*-----------------------------------------------------------
 * Macros required to setup the timer for the run time stats.
 *-----------------------------------------------------------*/
//	extern void vConfigureTimerForRunTimeStats( void );
//	#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() vConfigureTimerForRunTimeStats()
//	#define portGET_RUN_TIME_COUNTER_VALUE() TIM0->TC



#endif /* FREERTOS_CONFIG_H */
