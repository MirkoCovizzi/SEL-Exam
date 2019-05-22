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

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the ARM CM0 port.
 *----------------------------------------------------------*/

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

/*-----------------------------------------------------------*/
/* some configuration checks */
/*-----------------------------------------------------------*/
#ifndef configKERNEL_INTERRUPT_PRIORITY
    #define configKERNEL_INTERRUPT_PRIORITY     3
#endif

#if ((configUSE_SYSTICK != 1) && (configUSE_SYSTICK != 0))
	#error "configUSE_SYSTICK must be set to 1 or 0"
#endif


/* these masks are used to write to the SysTick control register */
/* cannot read out of it as it clears the timer flag */
#if (configSYSTICK_CLKSOURCE == 1)

	#define SYSTICK_ENABLE   0x7
	#define SYSTICK_DISABLE  0x5

#elif(configSYSTICK_CLKSOURCE == 0)

	#define SYSTICK_ENABLE   0x3
	#define SYSTICK_DISABLE  0x1

#else
	#error "configSYSTICK_CLKSOURCE must be set to 1 or 0"
#endif


/* Constants required to set up the initial stack. */
#define portINITIAL_XPSR            (0x01000000)

/* Each task maintains its own interrupt status in the critical nesting
variable. */
//static unsigned portBASE_TYPE uxCriticalNesting = 0xaaaaaaaa;
static unsigned portBASE_TYPE uxCriticalNesting = 0x0;

/*
 * Exception handlers.
 */
void xPortPendSVHandler (void);
void xPortSysTickHandler (void);
void vPortSVCHandler (void);

/*
 * Start first task is a separate function so it can be tested in isolation.
 */
void vPortStartFirstTask (void);

/*-----------------------------------------------------------*/

/*
 * See header file for description.
 */
portSTACK_TYPE *pxPortInitialiseStack( portSTACK_TYPE *pxTopOfStack, pdTASK_CODE pxCode, void *pvParameters )
{
    /* Simulate the stack frame as it would be created by a context switch interrupt. */
    pxTopOfStack--;                                     /* Decrement stack pointer. */
    *pxTopOfStack = portINITIAL_XPSR;                   /* Store xPSR Program Status Register. */
    pxTopOfStack--;                                     /* Decrement stack pointer. */
    *pxTopOfStack = ( portSTACK_TYPE ) pxCode;          /* Return address (new PC) */
    pxTopOfStack--;                                     /* Decrement stack pointer. */
    *pxTopOfStack = 0;                                  /* LR (R14)*/
    pxTopOfStack -= 5;                                  /* R12, R3, R2, R1. */
    *pxTopOfStack = ( portSTACK_TYPE ) pvParameters;    /* R0 */
    pxTopOfStack -= 8;                                  /* R7, R6, R5, R4, R11, R10, R9 and  R8. */

    return pxTopOfStack;
}
/*-----------------------------------------------------------*/

__asm void vPortSVCHandler (void)
{
        PRESERVE8

        ldr r3, =pxCurrentTCB           /* Restore the context. */
        ldr r1, [r3]                    /* Get address of the current TCB */
        ldr r0, [r1]                    /* The first item in pxCurrentTCB is the task top of stack. */
        ldm r0!, {r4-r7}                /* pop r8-r11 */
        mov r8, r4
        mov r9, r5
        mov r10, r6
        mov r11, r7
        ldm r0!, {r4-r7}                /* pop r4-r7 */
        msr psp, r0                     /* Update the new stack pointer for the current task */

        /* We want to return into thread mode, using PSP.
         * This requires to use 0xFFFFFFFD as the return address.
         * Doesn't have to be in LR! We compute the address in R0,
         * then simply jump with BX R0!
         */
        movs r0, #0
        subs r0, #3                     /* 0 - 3 = -3 = 0xFFFFFFFD */
        bx r0

        align 4
}
/*-----------------------------------------------------------*/

__asm void vPortStartFirstTask (void)
{
        ldr r0, =0                      /* Read vector table (always at 0) to locate the stack. */
        ldr r0, [r0]
        msr msp, r0                     /* Set the msp back to the start of the stack. */
        cpsie i                         /* Globally enable interrupts. */
        svc 0                           /* Call SVC to start the first task. */

        align 4
}
/*-----------------------------------------------------------*/

/*
 * See header file for description.
 */
portBASE_TYPE xPortStartScheduler (void)
{
    /* Start the timer that generates the tick ISR.  Interrupts are disabled here already. */
    portCONFIGURE_OS_TIMER();

    NVIC_SetPriority(PendSV_IRQn, configKERNEL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(SVCall_IRQn, configKERNEL_INTERRUPT_PRIORITY);

    /* Initialize the critical nesting count ready for the first task. */
    uxCriticalNesting = 0;

    /* Start the first task. */
    vPortStartFirstTask();

    /* Should not get here! */
    return 0;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler (void)
{
    /* It is unlikely that the CM0 port will require this function as there
    is nothing to return to.  */
}

void vPortYieldFromISR( void )
{
	/* Set a PendSV to request a context switch. */
	SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}

/*-----------------------------------------------------------*/


#define SYSTICK_CTRL_OFFSET 0x00
#define NVIC_ISER_OFFSET 	0x00
#define NVIC_ICER_OFFSET 	0x80

const struct {
#if (configUSE_SYSTICK == 1)
    unsigned portLONG SysTick_CTRL_ADDRESS;
#endif
    unsigned portLONG NVIC_ISER_ADDRESS;
    unsigned portLONG NVIC_ICER_ADDRESS;
    unsigned portLONG __syscallIRQ_Mask;

} __portIntMaskConsts = {

#if (configUSE_SYSTICK == 1)
    SysTick_BASE + SYSTICK_CTRL_OFFSET,     /* SysTick->CTRL */
#endif
    NVIC_BASE + NVIC_ISER_OFFSET,           /* NVIC->ISER */
    NVIC_BASE + NVIC_ICER_OFFSET,           /* NVIC->ICER */
    configBITMASK_SYSCALL_IRQ,
};

/* these definitions are consistent with the structure __portIntMaskConsts */
#if (configUSE_SYSTICK == 1)
    #define SYSTICK_IMMOFF 	0
	#define ISER_IMMOFF		4
	#define ICER_IMMOFF		8
	#define BITMASK_OFF		12
#elif (configUSE_SYSTICK == 0)
	#define ISER_IMMOFF		0
	#define ICER_IMMOFF		4
	#define BITMASK_OFF		8
#endif

/* used to hold the interrupt status temporarily */
volatile unsigned long __lastNVIC_Mask;

/* mask os related interrupts by emulating BASEPRI */
__asm void vPortSetInterruptMask (void)
{
	    PRESERVE8

        extern __lastNVIC_Mask
        extern __portIntMaskConsts

        ldr     r1, =__portIntMaskConsts
        ldr     r3, [r1, #ICER_IMMOFF]      ;NVIC->ICER address
        ldr     r2, [r1, #BITMASK_OFF]      ;configBITMASK_SYSCALL_IRQ

#if (configUSE_SYSTICK == 1)
        ldr     r0, [r1, #SYSTICK_IMMOFF]   ;SysTick->CTRL address
        movs    r1, #SYSTICK_DISABLE        ;Control word to disable SyStick interrupt
#endif

        ; Atomic interrupt disable
        cpsid   i
        isb
#if (configUSE_SYSTICK == 1)
        str     r1, [r0]                    ;Disable SysTick interrupt
#endif
        ldr     r0, [r3]                    ;Load currently enabled interrupts
        str     r2, [r3]                    ;Disable critical interrupts
        cpsie   i

        ldr     r1, =__lastNVIC_Mask  		; update the mask, cannot be interrupted by OS calls here
        ldr     r3, [r1]
        orrs    r0, r0, r3
        ands    r0, r0, r2
        str     r0, [r1]

        bx      lr

        ALIGN   4
}



__asm void vPortClearInterruptMask (void)
{
        PRESERVE8

        extern __lastNVIC_Mask

        ; Prepare to enable SysTick interrupts
        ldr     r1, =__lastNVIC_Mask        ;SysTick->CTRL
        ldr     r2, =__portIntMaskConsts
        ldr     r3, [r2, #ISER_IMMOFF]      ;NVIC->ISER address

#if (configUSE_SYSTICK == 1)
        ldr     r0, [r2, #SYSTICK_IMMOFF]   ;SysTick->CTRL address
        movs    r2, #SYSTICK_ENABLE         ;Control word to enable SysTick interrupt
#endif

        cpsid   i

#if (configUSE_SYSTICK == 1)
        str     r2, [r0]                    ;Enable SysTick interrupt
#endif

        ldr     r2, [r1]					;load the mask
        str     r2, [r3]                    ;Enable external interrupts
        movs    r0, #0
        str     r0, [r1]					;clear the mask
        cpsie   i

        bx      lr

        ALIGN   4
}


/*-----------------------------------------------------------*/

void vPortEnterCritical (void)
{
    portDISABLE_INTERRUPTS();
    ++uxCriticalNesting;
}


void vPortExitCritical(void)
{
    uxCriticalNesting--;
    if( uxCriticalNesting == 0 )
    {
        portENABLE_INTERRUPTS();
    }
}
/*-----------------------------------------------------------*/


#if (configEMULATE_BASEPRIO == 0)
/* in this case interrupts are always disabled globally within critical sections */
/* but interrupts might preempt the scheduler when */
/* configMAX_SYSCALL_INTERRUPT_PRIORITY != configKERNEL_INTERRUPT_PRIORITY */
__asm void xPortPendSVHandler( void )
{
        extern pxCurrentTCB;
        extern vTaskSwitchContext;

        PRESERVE8

        mrs r0, psp

        ldr r3, =pxCurrentTCB           /* Get the location of the current TCB. */
        ldr r2, [r3]

        subs r0, #16                    /* Save the remaining registers. */
        stm r0!,{r4-r7}                 /* In ascending address order: r8, r9, r10, r11, r4, r5, r6, r7 */
        subs r0, #32
        str r0, [r2]                    /* Save the new top of stack (PSP) into the first member of the TCB */
        mov r4, r8
        mov r5, r9
        mov r6, r10
        mov r7, r11
        stm r0!,{r4-r7}

        mov     r4, lr	   		/* backup in r4, caller saved register */

		/* disable globally interrupts here only if OS interrupts can preempt the scheduler */
#if (configMAX_SYSCALL_INTERRUPT_PRIORITY != configKERNEL_INTERRUPT_PRIORITY)
		cpsid i
#endif	
        bl vTaskSwitchContext

#if (configMAX_SYSCALL_INTERRUPT_PRIORITY != configKERNEL_INTERRUPT_PRIORITY)
        cpsie i
#endif

        mov lr, r4					   /* retrieve LR from r4 */

        ldr r3, =pxCurrentTCB           /* Restore context of new task */
        ldr r1, [r3]
        ldr r0, [r1]
        ldm r0!, {r4-r7}                /* pop r8-r11 */
        mov r8,  r4
        mov r9,  r5
        mov r10, r6
        mov r11, r7
        ldm r0!, {r4-r7}                /* pop r4-r7 */
        msr psp, r0
        bx lr

        align 4
}
#elif (configEMULATE_BASEPRIO == 1)
/* this applies if OS-related interrupts should preempt when possible */
/* thus have higher priority than the SysTick and PendSV */
/* SysTick / Timer can stay enabled since it has the same priority as PendSV */
/* so it will not preempt by hardware */
__asm void xPortPendSVHandler (void)
{
        extern pxCurrentTCB;
        extern vTaskSwitchContext;
        extern __lastNVIC_Mask;

        PRESERVE8

        mrs     r0, psp

        ldr     r3, =pxCurrentTCB           /* Get the location of the current TCB. */
        ldr     r2, [r3]

        subs    r0, #16                    /* Save the remaining registers. */
        stm     r0!,{r4-r7}                 /* In ascending address order: r8, r9, r10, r11, r4, r5, r6, r7 */
        subs    r0, #32
        str     r0, [r2]                    /* Save the new top of stack (PSP) into the first member of the TCB */
        mov     r4, r8
        mov     r5, r9
        mov     r6, r10
        mov     r7, r11
        stm     r0!,{r4-r7}
		 
        /* Emulate the BASEPRI behavior
		 * Disable critical external interrupts.
         * No need to disable SysTick, since it runs on same priority as PendSV.
         */
        ldr     r5, =__lastNVIC_Mask
        ldr     r7, =__portIntMaskConsts
        ldr     r1, [r7, #ICER_IMMOFF]      /* NVIC->ICER address */
        ldr     r2, [r7, #BITMASK_OFF]      /* configBITMASK_SYSCALL_IRQ */
        cpsid   i
        ldr     r0, [r1]                    /* Load currently enabled interrupts */
        str     r2, [r1]                    /* Disable all critical interrupts */
        cpsie   i

        ldr     r3, [r5]
        orrs    r0, r0, r3
        ands    r0, r0, r2
        str     r0, [r5]

        /* End of basepri emulation, task switch is protected now, perform the task switch */
        mov     r4, lr
        bl      vTaskSwitchContext
        mov     lr, r4

        /* Emulate the BASEPRI behavior
		 * Re-enable interrupts. This is necessary even if we haven't disabled
         * them prior to the task switch, because we might have entered the PendSV
         * handler in response to a portYIELD_FROM_API() call from within a 
         * critical section!
         */
#if (configUSE_SYSTICK == 1)
        ldr     r0, [r7, #SYSTICK_IMMOFF]   /* SysTick->CTRL address */
        movs    r1, #SYSTICK_ENABLE        /* Control word to enable SysTick interrupts */
#endif
        ldr     r2, [r7, #ISER_IMMOFF]      /* NVIC->ISER address */
		movs r3, #0

	    cpsid i

#if (configUSE_SYSTICK == 1)
        str     r1, [r0]                    /* Enable SysTick interrupt */
#endif
        ldr     r1, [r5]                    /* Bit mask for enabling external interrupts */
        str     r1, [r2]                    /* Enable external interrupts */
        str     r3, [r5]                    /* Clear mask */

		cpsie i

        /* Restore context of new task */
        ldr     r3, =pxCurrentTCB
        ldr     r1, [r3]
        ldr     r0, [r1]
        ldm     r0!, {r4-r7}                /* pop r8-r11 */
        mov     r8,  r4
        mov     r9,  r5
        mov     r10, r6
        mov     r11, r7
        ldm     r0!, {r4-r7}                /* pop r4-r7 */
        msr     psp, r0
        bx      lr

        align 4
}
#endif

/*-----------------------------------------------------------*/

void xPortOsTimerHandler( void )
{
    unsigned long ulDummy;

	portQUIT_OS_TIMER_INT();   // depending on the timer used, quit the interrupt
	
    /* If using preemption, also force a context switch. */
    #if configUSE_PREEMPTION == 1
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    #endif

    ulDummy = portSET_INTERRUPT_MASK_FROM_ISR();
    vTaskIncrementTick();
    portCLEAR_INTERRUPT_MASK_FROM_ISR( ulDummy );
}




