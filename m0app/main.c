#include "chip.h"
#include "board.h"
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>

#define CPU_USAGE 50
#define TICKRATE_HZ 1000

#define LPC_UART LPC_USART0
#define UARTx_IRQn  USART0_IRQn
#define UARTx_IRQHandler UART0_IRQHandler
#define _GPDMA_CONN_UART_Tx GPDMA_CONN_UART0_Tx
#define _GPDMA_CONN_UART_Rx GPDMA_CONN_UART0_Rx

#define SHARED_MUTEX 0x10088000
#define IPC_IRQ_Priority    7
#define IPC_IRQn       M4_IRQn

static uint8_t dmaChannelNumTx, dmaChannelNumRx;
static volatile uint32_t channelTC;	/* Terminal Counter flag for Channel */
static volatile uint32_t channelTCErr;
static FunctionalState  isDMATx = ENABLE;

static uint32_t l = 0;
static uint32_t WorkingTime = 0;
static uint32_t SleepingTime = 0;
static uint32_t LastTime = 0;

static int is_prime = 0;
static uint32_t prime_test_3 = 101;
static uint32_t prime_test_4 = 1009;
static uint32_t prime_test_5 = 10007; //Using about 80%
static uint32_t prime_test_6 = 100003;
static uint32_t prime_test_7 = 1000003;

static volatile uint32_t tick_ct = 0;
static volatile uint8_t mutex = (uint8_t) SHARED_MUTEX;
#define LOCKED 1
#define UNLOCKED 0

void idle(void);

void DMA_IRQHandler(void)
{
	uint8_t dmaChannelNum;
	if (isDMATx) {
		dmaChannelNum = dmaChannelNumTx;
	}
	else {
		dmaChannelNum = dmaChannelNumRx;
	}
	if (Chip_GPDMA_Interrupt(LPC_GPDMA, dmaChannelNum) == SUCCESS) {
		channelTC++;
	}
	else {
		channelTCErr++;
	}
}

void M4_IRQHandler(void)
{
	LPC_CREG->M4TXEVENT = 0;
}

void TIMER3_IRQHandler(void)
{
	if (Chip_TIMER_MatchPending(LPC_TIMER3, 1)) {
		Chip_TIMER_ClearMatch(LPC_TIMER3, 1);
		tick_ct += 1;
	}
}

void lock_mutex(void) {
	while (mutex == LOCKED) {
		Board_LED_Set(0, 1);
		__WFE();
	}
	
	mutex = LOCKED;
	
	__DSB();
	__DMB();
}

void unlock_mutex(void) {
	__DMB();
	
	mutex = UNLOCKED;
	
	__DSB();
	__SEV();
}

static void debug(char *msg, ...)
{
	char buff[80];
	
	va_list args;
	va_start(args, msg);
	vsprintf(buff, msg, args);
	
	lock_mutex();
	
	Chip_GPDMA_Init(LPC_GPDMA);
	/* Setting GPDMA interrupt */
	NVIC_DisableIRQ(DMA_IRQn);
	NVIC_SetPriority(DMA_IRQn, ((0x01 << 3) | 0x01));
	NVIC_EnableIRQ(DMA_IRQn);
	
	dmaChannelNumTx = Chip_GPDMA_GetFreeChannel(LPC_GPDMA, _GPDMA_CONN_UART_Tx);

	isDMATx = ENABLE;
	channelTC = channelTCErr = 0;
	Chip_GPDMA_Transfer(LPC_GPDMA, dmaChannelNumTx,
					  (uint32_t) buff,
					  _GPDMA_CONN_UART_Tx,
					  GPDMA_TRANSFERTYPE_M2P_CONTROLLER_DMA,
					  strlen(buff));
	while (!channelTC) {}
		
	Chip_GPDMA_DeInit(LPC_GPDMA);
	
	unlock_mutex();

}

int check_prime(uint32_t a)
{
   int c;
 
   for (c = 2; c <= a - 1; c++)
   { 
     if ( a % c == 0 ) return 0;
   }
   return 1;
}

int main(void)
{
	uint32_t timerFreq;
	
	SystemCoreClockUpdate();
	
	NVIC_SetPriority(IPC_IRQn, IPC_IRQ_Priority);
	NVIC_EnableIRQ(IPC_IRQn);
	
	debug("M0: M0APP Core BOOT successful!\r\n");
	
	/* Enable timer 2 clock and reset it */
	Chip_TIMER_Init(LPC_TIMER2);
	Chip_RGU_TriggerReset(RGU_TIMER2_RST);
	while (Chip_RGU_InReset(RGU_TIMER2_RST)) {}

	/* Timer setup as counter incrementing TC each clock */
	Chip_TIMER_PrescaleSet(LPC_TIMER2, 0);
	Chip_TIMER_Enable(LPC_TIMER2);
		
	/* Enable timer 3 clock and reset it */
	Chip_TIMER_Init(LPC_TIMER3);
	Chip_RGU_TriggerReset(RGU_TIMER3_RST);
	while (Chip_RGU_InReset(RGU_TIMER3_RST)) {}

	/* Get timer 3 peripheral clock rate */
	timerFreq = Chip_Clock_GetRate(CLK_MX_TIMER3);

	/* Timer setup for match and interrupt at TICKRATE_HZ */
	Chip_TIMER_Reset(LPC_TIMER3);
	Chip_TIMER_MatchEnableInt(LPC_TIMER3, 1);
	Chip_TIMER_SetMatch(LPC_TIMER3, 1, (timerFreq / TICKRATE_HZ));
	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER3, 1);
	Chip_TIMER_Enable(LPC_TIMER3);

	/* Enable timer interrupt */
	NVIC_EnableIRQ(TIMER3_IRQn);
	NVIC_ClearPendingIRQ(TIMER3_IRQn);
	
	while(1) {
		is_prime = check_prime(prime_test_6);
		
		idle();
	}
}

void idle(void) {
	uint32_t t;
	
	WorkingTime += Chip_TIMER_ReadCount(LPC_TIMER2) - l;
	t = Chip_TIMER_ReadCount(LPC_TIMER2);
	
	__WFI();
	
	SleepingTime += Chip_TIMER_ReadCount(LPC_TIMER2) - t;
	
	l = Chip_TIMER_ReadCount(LPC_TIMER2);
	
	if ((tick_ct - LastTime) >= 1000) {
		LastTime = tick_ct;
		
		debug("M0:: Prime: %u; W: %u; S: %u; L: %5.2f\r\n", prime_test_6, WorkingTime, SleepingTime, ((float)WorkingTime / (float)(SleepingTime + WorkingTime) * 100));
		
		SleepingTime = 0;
		WorkingTime = 0;
	}
	
}
