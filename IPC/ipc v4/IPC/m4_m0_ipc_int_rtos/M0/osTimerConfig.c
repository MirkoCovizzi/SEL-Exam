/* Scheduler includes. */
#include "FreeRTOS.h"

#if (configUSE_SYSTICK == 1)

void vConfigOsTimer(void) {

     /*
	 * NOTE: This CMSIS function sets SysTick exception priority to the lowest possible value.
     * However, we add an explicit initialization below to make this visible.
     */	
	SysTick_Config(configCPU_CLOCK_HZ / configTICK_RATE_HZ);

    /* Make PendSV, SysTick, and SVC the same priority as the kernel. */
    NVIC_SetPriority(SysTick_IRQn, configKERNEL_INTERRUPT_PRIORITY);
	
}

/* in systick you can read the register to clear the flag */
/* however interrupt are always generated even if flag is not cleared */
void vQuitOsTimerInt(void) {
 	return;
}


#elif (configUSE_SYSTICK == 0)

void vConfigOsTimer(void) {

	LPC_RITIMER->COMPVAL = configCPU_CLOCK_HZ / configTICK_RATE_HZ; 			// Set match value
	LPC_RITIMER->COUNTER=0;							// Set count value to 0
	LPC_RITIMER->CTRL = (1<<3)|(1<<2)|(1<<1);		// Enable, enable break, reset on mask compare, clear interrupt flag

	NVIC_EnableIRQ(RITIMER_WDT_IRQn);

	/* Make PendSV, SysTick, and SVC the same priority as the kernel. */
    NVIC_SetPriority(RITIMER_WDT_IRQn, configKERNEL_INTERRUPT_PRIORITY);
}


void vQuitOsTimerInt(void) {

	  LPC_RITIMER->CTRL |= (1<<0);	// Clear interrupt flag	for RiTimer
}


#endif

