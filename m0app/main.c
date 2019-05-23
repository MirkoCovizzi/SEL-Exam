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
#define SLAVE

#define TICKRATE_HZ 1000

#if defined(MASTER)
	#define TICKRATE_HZ_TIM0 10000
	static uint16_t iteration = 0;
#endif

#define LPC_UART LPC_USART0
#define UARTx_IRQn  USART0_IRQn
#define UARTx_IRQHandler UART0_IRQHandler

static uint32_t l = 0;
static uint32_t WorkingTime = 0;
static uint32_t SleepingTime = 0;
static uint32_t LastTime = 0;

static int is_prime = 0;
static uint32_t prime_test_3 = 101;
static uint32_t prime_test_4 = 1009;
static uint32_t prime_test_5 = 10007; //Using about 80%
static uint32_t prime_test_6 = 100003; //Using about 99%
static uint32_t prime_test_7 = 1000003;

static volatile uint32_t tick_ct = 0;

void idle(void);
int check_prime(uint32_t);
void debug(char *, ...);

void M4_IRQHandler(void)
{
	#if defined(MASTER)
		uint32_t result, delta_time;
		uint16_t iter;
		
		ipcex_msg_t msg;
		Chip_CREG_ClearM4Event();
		
		if (IPC_tryPopMsg(&msg) != QUEUE_VALID) {
			return;
		}
		
		result = msg.data0;
		iter = msg.id.pid;
		delta_time = Chip_TIMER_ReadCount(LPC_TIMER2) - msg.data1;
		
		if (iter % TICKRATE_HZ_TIM0 == 0) {
			debug("M0:: REQUEST PID: %d; RESULT: %u; DELTA TIME: %u;\r\n", iter, result, delta_time);
		}
	#elif defined(SLAVE)
		uint32_t result;
		
		ipcex_msg_t msg, msg_out;
		Chip_CREG_ClearM4Event();
		
		if (IPC_tryPopMsg(&msg) != QUEUE_VALID) {
			return;
		}
		
		result = check_prime(msg.data0);
		
		msg_out.id.pid = msg.id.pid;
		msg_out.id.cpu = (uint16_t) CPUID_M4;
		msg_out.data0 = result;
		msg_out.data1 = msg.data1;
		
		IPC_tryPushMsg(msg_out.id.cpu, &msg_out);
	#endif
}

void TIMER3_IRQHandler(void)
{
	if (Chip_TIMER_MatchPending(LPC_TIMER3, 1)) {
		Chip_TIMER_ClearMatch(LPC_TIMER3, 1);
		tick_ct += 1;
	}
}

#if defined(MASTER)
	void TIMER0_IRQHandler(void)
	{
		ipcex_msg_t msg;
		
		if (Chip_TIMER_MatchPending(LPC_TIMER0, 1)) {
			Chip_TIMER_ClearMatch(LPC_TIMER0, 1);
			
			msg.id.pid = iteration;
			msg.id.cpu = (uint16_t) CPUID_M4;
			msg.data0 = 20; // calc pi precision
			msg.data1 = Chip_TIMER_ReadCount(LPC_TIMER2); // timestamp
			
			IPC_tryPushMsg(msg.id.cpu, &msg);
			
			if (iteration % TICKRATE_HZ_TIM0 == 0) {
				//debug("M4:: REQUEST PID: %d; REQUEST: %u; TIMESTAMP: %u;\r\n", iteration, msg.data0, msg.data1);
			}
			
			iteration++;
		}
	}
#endif

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

int check_prime(uint32_t a)
{
   int c;
 
   for (c = 2; c <= a - 1; c++)
   { 
     if ( a % c == 0 ) return 0;
   }
   return 1;
}

int main(void) {
	uint32_t timerFreq;
	
	SystemCoreClockUpdate();
	
	debug("M0: M0APP Core BOOT successful!\r\n");
	
	IPCEX_Init();
	
	NVIC_EnableIRQ(M4_IRQn);
	
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
		
	#if defined(MASTER)
		/* Enable timer 0 clock and reset it */
		Chip_TIMER_Init(LPC_TIMER0);
		Chip_RGU_TriggerReset(RGU_TIMER0_RST);
		while (Chip_RGU_InReset(RGU_TIMER0_RST)) {}

		/* Get timer 0 peripheral clock rate */
		timerFreq = Chip_Clock_GetRate(CLK_MX_TIMER0);

		/* Timer setup for match and interrupt at TICKRATE_HZ_TIM0 */
		Chip_TIMER_Reset(LPC_TIMER0);
		Chip_TIMER_MatchEnableInt(LPC_TIMER0, 1);
		Chip_TIMER_SetMatch(LPC_TIMER0, 1, (timerFreq / TICKRATE_HZ_TIM0));
		Chip_TIMER_ResetOnMatchEnable(LPC_TIMER0, 1);
		Chip_TIMER_Enable(LPC_TIMER0);

		/* Enable timer interrupt */
		NVIC_EnableIRQ(TIMER0_IRQn);
		NVIC_ClearPendingIRQ(TIMER0_IRQn);
	#endif
	
	while(1) {
		is_prime = check_prime(prime_test_4);
		
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
		
		debug("M0:: Prime: %u; W: %u; S: %u; L: %5.2f\r\n", prime_test_4, WorkingTime, SleepingTime, ((float)WorkingTime / (float)(SleepingTime + WorkingTime) * 100));
		
		SleepingTime = 0;
		WorkingTime = 0;
	}
}
