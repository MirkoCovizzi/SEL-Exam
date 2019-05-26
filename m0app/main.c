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
#define SLAVE
#define ASYNC 1
#define CPU 1
#define OTHER_CPU 0

#define SHARED_MUTEX 0x10086000
#define SHARED_STRUCT 0x10087000

typedef struct __shared_mutex {
	uint32_t b[2];
	uint32_t c[2];
	uint32_t k;
} shared_mutex;

typedef struct __ipc_msg {
	struct {
		uint32_t cpu;
		uint32_t pid;
	} id;

	uint32_t data0;
	uint32_t data1;
} ipc_msg;

static volatile shared_mutex *mutex = (shared_mutex *) SHARED_MUTEX;
static volatile ipc_msg *shared_msg = (ipc_msg *) SHARED_STRUCT;
static volatile uint32_t previous_pid = 0;

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
static uint32_t m4_handler_LastTime = 0;

static int is_prime = 0;
static uint32_t prime_test_3 = 101; //Using about 3%
static uint32_t prime_test_4 = 1009; //Using about 25%
static uint32_t prime_test_5 = 10007; //Using about 80%
static uint32_t prime_test_6 = 100003; //Using about 99%
static uint32_t prime_test_7 = 1000003;
static uint32_t prime_test_8 = 10000019;

static volatile uint32_t tick_ct = 0;

void idle(void);
int check_prime(uint32_t);
void debug(char *, ...);

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
	__DSB();
	mutex->b[CPU] = 1;
	__DSB();
	mutex->c[CPU] = 1;
	__DSB();
	__DMB();
	__enable_irq();
}

void M4_IRQHandler(void)
{
	if (ASYNC) {
		Chip_CREG_ClearM4Event();
	} else {
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
			
			if ((tick_ct - m4_handler_LastTime) >= 1000) {
				m4_handler_LastTime = tick_ct;
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

static void mutex_test(void) {
	volatile uint32_t result;
	
	lock_mutex();
	
	debug("M0:: Inside Critical Section\r\n");

	if (shared_msg->id.pid % 2 == 1) {
		debug("M0:: PID: %d; REQUEST: %u; TIMESTAMP: %u\r\n", shared_msg->id.pid, shared_msg->data0, shared_msg->data1);
		shared_msg->id.pid = shared_msg->id.pid + 1;
		result = check_prime(shared_msg->data0);
		shared_msg->data0 = result;
	}
	
	debug("M0:: Exiting Critical Section\r\n");
	unlock_mutex();
}

int main(void) {
	uint32_t timerFreq, result;
	
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
		
		if (ASYNC) {
			lock_mutex();
			debug("M0:: Entered Critical Section\r\n");
			if (shared_msg->id.pid % 2 == 1) {
				result = check_prime(shared_msg->data0);
				
				shared_msg->id.pid = shared_msg->id.pid + 1;
				shared_msg->data0 = result;
				
				debug("M0:: PID: %u; RESULT: %u; TIMESTAMP: %u;\r\n", shared_msg->id.pid, shared_msg->data0, shared_msg->data1);
			}
			debug("M0:: Exiting Critical Section\r\n");
			unlock_mutex();
		}
		
		idle();
	}
	
	/*
	//mutex test
	while(1) {
		mutex_test();
	}
	*/
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
		
		__disable_irq();
		debug("M0:: Prime: %u; W: %u; S: %u; L: %5.2f\r\n", prime_test_4, WorkingTime, SleepingTime, ((float)WorkingTime / (float)(204000000) * 100));
		__enable_irq();
		
		SleepingTime = 0;
		WorkingTime = 0;
	}
}
