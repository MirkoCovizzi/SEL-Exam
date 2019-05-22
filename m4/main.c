#include "chip.h"
#include "board.h"
#include "app_multicore_cfg.h"
#include "ipc_example.h"
#include "ipc_msg.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>

#define DEBUG 0

#define CPU_USAGE 95
#define TICKRATE_HZ 1000
#define TICKRATE_HZ_TIM1 20000

#define LPC_UART LPC_USART0
#define UARTx_IRQn  USART0_IRQn
#define UARTx_IRQHandler UART0_IRQHandler

static uint32_t l = 0;
static uint32_t WorkingTime = 0;
static uint32_t SleepingTime = 0;
static uint32_t LastTime = 0;

static float pi;
static uint32_t precision = 0;

static volatile uint32_t tick_ct = 0;
static uint16_t iteration = 0;

static volatile int printing = 0;

void idle(void);
void debug(char *, ...);

void SysTick_Handler(void) {	
	tick_ct += 1;
}

void M0APP_IRQHandler(void)
{
	uint32_t result, delta_time;
	uint16_t iter;
	
	ipcex_msg_t msg;
	Chip_CREG_ClearM0AppEvent();
	
	if (IPC_tryPopMsg(&msg) != QUEUE_VALID) {
		return;
	}
	
	result = msg.data0;
	iter = msg.id.pid;
	delta_time = Chip_TIMER_ReadCount(LPC_TIMER0) - msg.data1;
	
	if (iter % TICKRATE_HZ_TIM1 == 0) {
		debug("M4:: REQUEST PID: %d; RESULT: %u; DELTA TIME: %u;\r\n", iter, result, delta_time);
	}
}

void TIMER1_IRQHandler(void)
{
	ipcex_msg_t msg;
	
	if (Chip_TIMER_MatchPending(LPC_TIMER1, 1)) {
		Chip_TIMER_ClearMatch(LPC_TIMER1, 1);
		
		msg.id.pid = iteration;
		msg.id.cpu = (uint16_t) CPUID_M0APP;
		msg.data0 = 101; // prime test
		msg.data1 = Chip_TIMER_ReadCount(LPC_TIMER0); // timestamp
		
		IPC_tryPushMsg(msg.id.cpu, &msg);
		
		if (iteration % TICKRATE_HZ_TIM1 == 0) {
			//debug("M4:: REQUEST PID: %d; REQUEST: %u; TIMESTAMP: %u;\r\n", iteration, msg.data0, msg.data1);
		}
		
		iteration++;
	}
}

static void debug(char *msg, ...)
{
	char buff[80];
	
	if (DEBUG) {
		
		va_list args;
		va_start(args, msg);
		vsprintf(buff, msg, args);
		
		Chip_UART_SendBlocking(LPC_UART, buff, strlen(buff));

	}

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
	
	tick_ct = 0;
	
	Chip_UART_Init(LPC_UART);
	Chip_UART_SetBaud(LPC_UART, 115200);
	Chip_UART_ConfigData(LPC_UART, UART_LCR_WLEN8 | UART_LCR_SBS_1BIT); /* Default 8-N-1 */

	/* Enable UART Transmit */
	Chip_UART_TXEnable(LPC_UART);
	Chip_UART_SetupFIFOS(LPC_UART, (UART_FCR_FIFO_EN | UART_FCR_RX_RS | UART_FCR_TX_RS | UART_FCR_DMAMODE_SEL | UART_FCR_TRG_LEV0));
	
	if (M0Image_Boot(CPUID_M0APP, (uint32_t) BASE_ADDRESS_M0APP) < 0) {
		debug("M4:: Unable to BOOT M0APP Core!\r\n");
	} else {
		debug("M4:: M0APP Core BOOT successful!\r\n");
	}
	
	IPCEX_Init();
	
	NVIC_EnableIRQ(M0APP_IRQn);
	
	SysTick_Config(SystemCoreClock / TICKRATE_HZ);
	
	/* Enable timer 0 clock and reset it */
	Chip_TIMER_Init(LPC_TIMER0);
	Chip_RGU_TriggerReset(RGU_TIMER0_RST);
	while (Chip_RGU_InReset(RGU_TIMER0_RST)) {}

	/* Timer setup as counter incrementing TC each clock */
	Chip_TIMER_PrescaleSet(LPC_TIMER0, 0);
	Chip_TIMER_Enable(LPC_TIMER0);
		
	/* Enable timer 1 clock and reset it */
	Chip_TIMER_Init(LPC_TIMER1);
	Chip_RGU_TriggerReset(RGU_TIMER1_RST);
	while (Chip_RGU_InReset(RGU_TIMER1_RST)) {}

	/* Get timer 1 peripheral clock rate */
	timerFreq = Chip_Clock_GetRate(CLK_MX_TIMER1);

	/* Timer setup for match and interrupt at TICKRATE_HZ */
	Chip_TIMER_Reset(LPC_TIMER1);
	Chip_TIMER_MatchEnableInt(LPC_TIMER1, 1);
	Chip_TIMER_SetMatch(LPC_TIMER1, 1, (timerFreq / TICKRATE_HZ_TIM1));
	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER1, 1);
	Chip_TIMER_Enable(LPC_TIMER1);

	/* Enable timer interrupt */
	NVIC_EnableIRQ(TIMER1_IRQn);
	NVIC_ClearPendingIRQ(TIMER1_IRQn);
	
	while(1) {
		pi = calc_pi(precision);
		
		idle();
	}
}

void idle(void) {
	uint32_t t;
	
	WorkingTime += Chip_TIMER_ReadCount(LPC_TIMER0) - l;
	t = Chip_TIMER_ReadCount(LPC_TIMER0);
	
	__WFI();
	
	SleepingTime += Chip_TIMER_ReadCount(LPC_TIMER0) - t;
	
	l = Chip_TIMER_ReadCount(LPC_TIMER0);
	
	if ((tick_ct - LastTime) >= 1000) {
		LastTime = tick_ct;
		
		if (((float)WorkingTime / (float)(SleepingTime + WorkingTime) * 100) < CPU_USAGE) precision += 10;
		
		debug("M4:: PI: %5.15f; Precision: %u; W: %u; S: %u; L: %5.2f\r\n", pi, precision, WorkingTime, SleepingTime, ((float)WorkingTime / (float)(SleepingTime + WorkingTime) * 100));

		SleepingTime = 0;
		WorkingTime = 0;
	}
	
}
