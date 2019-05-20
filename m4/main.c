#include "chip.h"
#include "board.h"
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>

#define CPU_USAGE 50
#define TICKRATE_HZ 1000
#define TICKRATE_HZ_TIM1 1
#define LPC_UART LPC_USART0
#define UARTx_IRQn  USART0_IRQn
#define UARTx_IRQHandler UART0_IRQHandler
#define _GPDMA_CONN_UART_Tx GPDMA_CONN_UART0_Tx
#define _GPDMA_CONN_UART_Rx GPDMA_CONN_UART0_Rx

static uint8_t dmaChannelNumTx, dmaChannelNumRx;
static volatile uint32_t channelTC;	/* Terminal Counter flag for Channel */
static volatile uint32_t channelTCErr;
static FunctionalState  isDMATx = ENABLE;

static uint32_t l = 0;
static uint32_t WorkingTime = 0;
static uint32_t SleepingTime = 0;
static uint32_t LastTime = 0;

static uint32_t tick_ct = 0;
static float pi;
static uint32_t precision = 0;

void idle(void);

void SysTick_Handler(void) {
	tick_ct += 1;
}

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

static void debug(char *msg, ...)
{
	char buff[80];
	
	va_list args;
	va_start(args, msg);
	vsprintf(buff, msg, args);

	isDMATx = ENABLE;
	channelTC = channelTCErr = 0;
	Chip_GPDMA_Transfer(LPC_GPDMA, dmaChannelNumTx,
					  (uint32_t) buff,
					  _GPDMA_CONN_UART_Tx,
					  GPDMA_TRANSFERTYPE_M2P_CONTROLLER_DMA,
					  strlen(buff));
	while (!channelTC) {}
}

float calc_pi(int n) {
	int i;
	float fal, kal;

	fal = (2 * n - 1) + powf(n, 2);

	for( i = n - 1; i >= 1; i--) {
		kal = powf(i, 2) / fal;
		fal = (2 * i - 1) + kal;   
	}
	
	return 4 / (1 + kal);
}

int main(void) {
	uint32_t timerFreq;
	
	SystemCoreClockUpdate();
	Board_Init();
	
	SysTick_Config(SystemCoreClock / TICKRATE_HZ);
	
	Chip_UART_Init(LPC_UART);
	Chip_UART_SetBaud(LPC_UART, 115200);
	Chip_UART_ConfigData(LPC_UART, UART_LCR_WLEN8 | UART_LCR_SBS_1BIT); /* Default 8-N-1 */

	/* Enable UART Transmit */
	Chip_UART_TXEnable(LPC_UART);
	Chip_UART_SetupFIFOS(LPC_UART, (UART_FCR_FIFO_EN | UART_FCR_RX_RS | UART_FCR_TX_RS | UART_FCR_DMAMODE_SEL | UART_FCR_TRG_LEV0));
	
	Chip_GPDMA_Init(LPC_GPDMA);
	/* Setting GPDMA interrupt */
	NVIC_DisableIRQ(DMA_IRQn);
	NVIC_SetPriority(DMA_IRQn, ((0x01 << 3) | 0x01));
	NVIC_EnableIRQ(DMA_IRQn);
	
	dmaChannelNumTx = Chip_GPDMA_GetFreeChannel(LPC_GPDMA, _GPDMA_CONN_UART_Tx);
	
	/* Enable timer 1 clock and reset it */
	Chip_TIMER_Init(LPC_TIMER1);
	Chip_RGU_TriggerReset(RGU_TIMER1_RST);
	while (Chip_RGU_InReset(RGU_TIMER1_RST)) {}

	/* Get timer 1 peripheral clock rate */
	timerFreq = Chip_Clock_GetRate(CLK_MX_TIMER1);

	/* Timer setup as counter incrementing TC each clock */
	Chip_TIMER_PrescaleSet(LPC_TIMER1, 0);
	Chip_TIMER_Enable(LPC_TIMER1);
	
	while(1) {
		pi = calc_pi(precision);
		
		idle();
	}
}

void idle(void) {
	uint32_t t;
	
	__disable_irq();
	
	WorkingTime += Chip_TIMER_ReadCount(LPC_TIMER1) - l;
	t = Chip_TIMER_ReadCount(LPC_TIMER1);
	
	__WFI();
	
	SleepingTime += Chip_TIMER_ReadCount(LPC_TIMER1) - t;
	
	l = Chip_TIMER_ReadCount(LPC_TIMER1);
	
	__enable_irq();
	
	if ((tick_ct - LastTime) >= 1000) {
		LastTime = tick_ct;
		
		if (((float)WorkingTime / (float)(SleepingTime + WorkingTime) * 100) < CPU_USAGE) precision += 10;
		debug("PI: %5.15f; Precision: %u; W: %u; S: %u; L: %5.2f\r\n", pi, precision, WorkingTime, SleepingTime, ((float)WorkingTime / (float)(SleepingTime + WorkingTime) * 100));
			
		SleepingTime = 0;
		WorkingTime = 0;
	}
	
}
