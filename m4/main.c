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

#define DEBUG 1
#define MASTER
#define ASYNC 1
#define CPU 0
#define OTHER_CPU 1

#define SHARED_MUTEX 0x10086000
#define SHARED_STRUCT 0x10087000
typedef struct __shared_mutex {
	volatile uint8_t b[2];
	volatile uint8_t c[2];
	volatile uint8_t k;
} shared_mutex;
static volatile shared_mutex *mutex = (shared_mutex *) SHARED_MUTEX;
static volatile ipcex_msg_t *shared_msg = (ipcex_msg_t *) SHARED_STRUCT;
static volatile uint16_t previous_pid = 1;
static volatile uint32_t previous_data = 1;
static volatile int received = 0;

#define CPU_USAGE 1
#define TICKRATE_HZ 1000

#if defined(MASTER)
	#define TICKRATE_HZ_TIM0 200000
	static volatile uint16_t iteration = 0;
#endif

#define LPC_UART LPC_USART0
#define UARTx_IRQn  USART0_IRQn
#define UARTx_IRQHandler UART0_IRQHandler

static uint32_t l = 0;
static uint32_t WorkingTime = 0;
static uint32_t SleepingTime = 0;
static uint32_t LastTime = 0;
static uint32_t m0app_handler_LastTime = 0;
static uint32_t async_result_read_LastTime = 0;

static float pi;
static uint32_t precision = 0;

static volatile uint32_t tick_ct = 0;

void idle(void);
void debug(char *, ...);
float calc_pi(int);

static void lock_mutex(void) {
	__disable_irq();
	__DMB();
	mutex->b[CPU] = 0;
	__DSB();
	LOOP:if(mutex->k != CPU) {
		mutex->c[CPU] = 1;
		__DSB();
		if (mutex->b[mutex->k]) {
			mutex->k = CPU;
			__DSB();
		}
		goto LOOP;
	} else {
		mutex->c[CPU] = 0;
		__DSB();
		if (!mutex->c[OTHER_CPU]) goto LOOP;
	}
}

static void unlock_mutex(void) {
	mutex->b[CPU] = 1;
	__DSB();
	mutex->c[CPU] = 1;
	__DSB();
	__DMB();
	__enable_irq();
}

void TIMER1_IRQHandler(void)
{
	if (Chip_TIMER_MatchPending(LPC_TIMER1, 1)) {
		Chip_TIMER_ClearMatch(LPC_TIMER1, 1);
		tick_ct += 1;
	}
}

void M0APP_IRQHandler(void)
{
	#if defined(MASTER)
		uint32_t result, delta_time;
		uint16_t iter;
		
		ipcex_msg_t msg;
		Chip_CREG_ClearM0AppEvent();
		
		if (IPC_tryPopMsg(&msg) != QUEUE_VALID) {
			return;
		}
		
		result = msg.data0;
		iter = msg.id.pid;
		delta_time = DWT->CYCCNT - msg.data1;
		
		if ((tick_ct - m0app_handler_LastTime) >= 1000) {
			m0app_handler_LastTime = tick_ct;
			debug("M4:: REQUEST PID: %d; RESULT: %u; DELTA TIME: %u;\r\n", iter, result, delta_time);
		}
	#elif defined(SLAVE)
		uint32_t result;
		
		ipcex_msg_t msg, msg_out;
		Chip_CREG_ClearM0AppEvent();
		
		if (IPC_tryPopMsg(&msg) != QUEUE_VALID) {
			return;
		}
		
		result = calc_pi(msg.data0);
		
		msg_out.id.pid = msg.id.pid;
		msg_out.id.cpu = (uint16_t) CPUID_M0APP;
		msg_out.data0 = result;
		msg_out.data1 = msg.data1;
		
		IPC_tryPushMsg(msg_out.id.cpu, &msg_out);
	#endif
}

#if defined(MASTER)
	void TIMER0_IRQHandler(void)
	{
		ipcex_msg_t msg;
			
		if (Chip_TIMER_MatchPending(LPC_TIMER0, 1)) {
			Chip_TIMER_ClearMatch(LPC_TIMER0, 1);
			
			if (ASYNC) {
				lock_mutex();
				if (shared_msg->id.pid != previous_pid && shared_msg->data0 != previous_data) {			
					previous_pid = shared_msg->id.pid + 1;
				
					shared_msg->data0 = 101; // prime test
					previous_data = shared_msg->data0;
					shared_msg->data1 = DWT->CYCCNT;
					shared_msg->id.pid = previous_pid;
					
					received = 0;
				}
				unlock_mutex();
			} else {
				msg.id.pid = iteration;
				msg.id.cpu = (uint16_t) CPUID_M0APP;
				msg.data0 = 101; // prime test
				msg.data1 = DWT->CYCCNT; // timestamp
				
				IPC_tryPushMsg(msg.id.cpu, &msg);
				
				iteration++;
			}
				
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
	
	mutex->b[CPU] = 1;
	mutex->c[CPU] = 1;
	mutex->b[OTHER_CPU] = 1;
	mutex->c[OTHER_CPU] = 1;
	mutex->k = 0;
	tick_ct = 0;
	
	/* Enable TRC */
  CoreDebug->DEMCR &= ~0x01000000;
  CoreDebug->DEMCR |=  0x01000000;
	
	/* Enable counter */
  DWT->CTRL &= ~0x00000001;
  DWT->CTRL |=  0x00000001;
	
  /* Reset counter */
  DWT->CYCCNT = 0;
	
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
	
	/* Enable timer 1 clock and reset it */
	Chip_TIMER_Init(LPC_TIMER1);
	Chip_RGU_TriggerReset(RGU_TIMER1_RST);
	while (Chip_RGU_InReset(RGU_TIMER1_RST)) {}

	/* Get timer 1 peripheral clock rate */
	timerFreq = Chip_Clock_GetRate(CLK_MX_TIMER1);

	/* Timer setup for match and interrupt at TICKRATE_HZ */
	Chip_TIMER_Reset(LPC_TIMER1);
	Chip_TIMER_MatchEnableInt(LPC_TIMER1, 1);
	Chip_TIMER_SetMatch(LPC_TIMER1, 1, (timerFreq / TICKRATE_HZ));
	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER1, 1);
	Chip_TIMER_Enable(LPC_TIMER1);

	/* Enable timer interrupt */
	NVIC_EnableIRQ(TIMER1_IRQn);
	NVIC_ClearPendingIRQ(TIMER1_IRQn);
		
	#if defined(MASTER)
		shared_msg->id.pid = 0;
		shared_msg->data0 = 0;
		
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
		pi = calc_pi(precision);
		
		if (ASYNC) {
			lock_mutex();
			if (shared_msg->id.pid > previous_pid && !received && (tick_ct - async_result_read_LastTime) >= 1000) {
				async_result_read_LastTime = tick_ct;
				debug("M4:: REQUEST PID: %d; RESULT: %u; DELTA TIME: %u;\r\n", shared_msg->id.pid, shared_msg->data0, DWT->CYCCNT - shared_msg->data1);
				received = 1;
			}
			if (shared_msg->id.pid > previous_pid && shared_msg->data0 == 101) Board_LED_Set(0, 1);
			unlock_mutex();
		}
		
		idle();
	}
}

void idle(void) {
	uint32_t t;
	
	WorkingTime += DWT->CYCCNT - l;
	t = DWT->CYCCNT;
	
	__WFI();
	
	SleepingTime += DWT->CYCCNT - t;
	
	l = DWT->CYCCNT;
	
	if ((tick_ct - LastTime) >= 1000) {
		LastTime = tick_ct;
		
		if (((float)WorkingTime / (float)(204000000) * 100) < CPU_USAGE) precision += 10;
		
		debug("M4:: PI: %5.15f; Precision: %u; W: %u; S: %u; L: %5.2f\r\n", pi, precision, WorkingTime, SleepingTime, ((float)WorkingTime / (float)(204000000) * 100));

		SleepingTime = 0;
		WorkingTime = 0;
	}
}
