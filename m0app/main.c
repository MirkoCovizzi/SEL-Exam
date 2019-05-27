#include "chip.h"
#include "board.h"
#include "app_multicore_cfg.h"
#include "ipc_example.h"
#include "ipc_msg.h"
#include "common_config.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>

#define CPU 1
#define OTHER_CPU 0

/*
	101					3%
	1009				25%
	10007
	100003
	1000003
	10000019
*/
#define PRIME_TEST 1009

#define LPC_UART LPC_USART0

typedef struct __shared_mutex {
	volatile uint32_t b[2];
	volatile uint32_t c[2];
	volatile uint32_t k;
} shared_mutex;

typedef struct __ipc_msg {
	volatile uint32_t counter;
	volatile uint32_t data0;
	volatile uint32_t data1;
	volatile uint32_t data2;
} ipc_msg;

static volatile shared_mutex *mutex = (shared_mutex *) SHARED_MUTEX;
static volatile shared_mutex *print_mutex = (shared_mutex *) SHARED_PRINT_MUTEX;
static volatile ipc_msg *shared_msg = (ipc_msg *) SHARED_STRUCT;
static volatile uint16_t iteration;

static volatile int notify = 0;
static volatile int finished = 0;
static volatile int test_number = 0;
static volatile int test_running = 0;

static volatile uint32_t l = 0;
static volatile uint32_t WorkingTime = 0;
static volatile uint32_t SleepingTime = 0;
static volatile uint32_t LastTime = 0;

static int debug_toggle = 0;
static int cpu_load_toggle = 0;

int check_prime(uint32_t);
void print(char *, ...);
void debug(char *, ...);
void idle(void);

static int is_prime = 0;

static volatile uint32_t tick_ct = 0;

void M4_IRQHandler(void)
{	
	Chip_CREG_ClearM4Event();
	notify = 1;
	
	if (test_number == 1 && test_running) {
		uint32_t result;
		int ret;
			
		ipcex_msg_t msg, msg_out;
			
		if (IPC_tryPopMsg(&msg) != QUEUE_VALID) {
			return;
		}
		
		debug("M0APP:: REQUEST COUNTER: %d; REQUEST: %u; TIMESTAMP: %u;\r\n", msg.id.pid, msg.data0, msg.data1);
		
		result = check_prime(msg.data0);
			
		msg_out.id.pid = msg.id.pid;
		msg_out.id.cpu = (uint16_t) CPUID_M4;
		msg_out.data0 = result;
		msg_out.data1 = msg.data1;
			
		if ((ret = IPC_tryPushMsg(msg_out.id.cpu, &msg_out)) == QUEUE_INSERT) iteration++;
		if (iteration >= ITERATIONS) finished = 1;
	}
}

void TIMER3_IRQHandler(void)
{
	if (Chip_TIMER_MatchPending(LPC_TIMER3, 1)) {
		Chip_TIMER_ClearMatch(LPC_TIMER3, 1);
		tick_ct += 1;
	}
}

static void wait_event(void) {
	__WFE();
	while(!notify);
	print("M0APP:: RECEIVED NOTIFY!\r\n");
	notify = 0;
}

static void notify_event(void) {
	__DSB();
	__SEV();
}

static void lock_mutex(volatile shared_mutex *mutex) {
	mutex->b[CPU] = 0;
	LOOP:if(mutex->k != CPU) {
		mutex->c[CPU] = 1;
		if (mutex->b[mutex->k]) {
			mutex->k = CPU;
		}
		goto LOOP;
	} else {
		mutex->c[CPU] = 0;
		if (!mutex->c[OTHER_CPU]) goto LOOP;
	}
}

static void unlock_mutex(volatile shared_mutex *mutex) {
	mutex->b[CPU] = 1;
	mutex->c[CPU] = 1;
}

static void print(char *msg, ...) {
	char buff[80];
	uint32_t prim;
		
	va_list args;
	va_start(args, msg);
	vsprintf(buff, msg, args);
	
	prim = __get_PRIMASK();
	__disable_irq();
	lock_mutex(print_mutex);
	
	Chip_UART_SendBlocking(LPC_UART, buff, strlen(buff));
	
	unlock_mutex(print_mutex);
	if (!prim) __enable_irq();
}

static void debug(char *msg, ...) {	
	if (debug_toggle) {	
		char buff[80];
		char out_buff[80] = "(DEBUG) ";
		uint32_t prim;
		
		va_list args;
		va_start(args, msg);
		vsprintf(buff, msg, args);
		strcat(out_buff, buff);
		
		prim = __get_PRIMASK();
		__disable_irq();
		lock_mutex(print_mutex);
		
		Chip_UART_SendBlocking(LPC_UART, out_buff, strlen(out_buff));
		
		unlock_mutex(print_mutex);
		if (!prim) __enable_irq();
	}
}

static void setup_board(void) {
	SystemCoreClockUpdate();
}

static void setup_clock_counter(void) {
	/* Enable timer 2 clock and reset it */
	Chip_TIMER_Init(LPC_TIMER2);
	Chip_RGU_TriggerReset(RGU_TIMER2_RST);
	while (Chip_RGU_InReset(RGU_TIMER2_RST)) {}

	/* Timer setup as counter incrementing TC each clock */
	Chip_TIMER_PrescaleSet(LPC_TIMER2, 0);
	Chip_TIMER_Enable(LPC_TIMER2);
}

static uint32_t clock_counter_value(void) {
	return Chip_TIMER_ReadCount(LPC_TIMER2);
}

static void setup_system_timer(void) {
	uint32_t timerFreq;
	
	/* Enable timer 3 clock and reset it */
	Chip_TIMER_Init(LPC_TIMER3);
	Chip_RGU_TriggerReset(RGU_TIMER3_RST);
	while (Chip_RGU_InReset(RGU_TIMER3_RST)) {}

	/* Get timer 3 peripheral clock rate */
	timerFreq = Chip_Clock_GetRate(CLK_MX_TIMER3);

	/* Timer setup for match and interrupt at TICKRATE_HZ */
	Chip_TIMER_Reset(LPC_TIMER3);
	Chip_TIMER_MatchEnableInt(LPC_TIMER3, 1);
	Chip_TIMER_SetMatch(LPC_TIMER3, 1, (timerFreq / 1000));
	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER3, 1);
	
	/* Enable timer interrupt */
	NVIC_EnableIRQ(TIMER3_IRQn);
	NVIC_ClearPendingIRQ(TIMER3_IRQn);
}

static void start_system_timer(void) {
	Chip_TIMER_Enable(LPC_TIMER3);
}

static void stop_system_timer(void) {
	Chip_TIMER_Disable(LPC_TIMER3);
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
	uint32_t res;
	int i;
	
	setup_board();
	setup_clock_counter();
	setup_system_timer();
	
	print("M0APP:: BOOT successful!\r\n");
	NVIC_EnableIRQ(M4_IRQn);
	
	notify_event();
	
	wait_event();
	
	res = shared_msg->data0;
	debug_toggle = shared_msg->data1;
	cpu_load_toggle = shared_msg->data2;
	
	print("M0APP:: Setting up test #%c...\r\n", res);
	
	switch(res) {
		case '1':
			IPCEX_Init();
			test_number = 1;
			iteration = 0;
			finished = 0;
			break;
		case '2':
			test_number = 2;
			iteration = 0;
			finished = 0;
			break;
		case '4':
			test_number = 4;
			break;
	}
	
	print("M0APP:: Finished setting up test #%c.\r\n", res);
	print("M0APP:: Starting test #%c...\r\n", res);
	test_running = 1;
	
	notify_event();
	
	switch(res) {
		case '1':
			while(!finished) {
				if (cpu_load_toggle) is_prime = check_prime(PRIME_TEST);
				
				__WFI();
			}
			break;
		case '2':
			while(!finished) {
				int result;
				uint32_t prim;
				
				if (cpu_load_toggle) is_prime = check_prime(PRIME_TEST);
				
				prim = __get_PRIMASK();
				__disable_irq();
				lock_mutex(mutex);
				
				debug("M0APP:: Entered Critical Section\r\n");
				
				if (shared_msg->counter % 2 == 1) {
					result = check_prime(shared_msg->data0);
					
					shared_msg->counter = shared_msg->counter + 1;
					shared_msg->data0 = result;
					
					if (shared_msg->counter >= 2 * ITERATIONS) finished = 1;
					
					debug("M0APP:: RESPONSE COUNTER: %u; RESULT: %u; TIMESTAMP: %u;\r\n", shared_msg->counter, shared_msg->data0, shared_msg->data1);
				}
				
				debug("M0APP:: Exiting Critical Section\r\n");
				
				unlock_mutex(mutex);
				if (!prim) __enable_irq();
			}
			break;
		case '4':
			start_system_timer();
			for (i = 0; i < 10000; i++) {
				if (cpu_load_toggle) is_prime = check_prime(PRIME_TEST);
					
				idle();
			}
			stop_system_timer();
			break;
	}
	
	test_running = 0;
	print("M0APP:: Finished test #%c.\r\n", res);
	
	notify_event();
}

void idle(void) {
	uint32_t t;
	
	WorkingTime += clock_counter_value() - l;
	t = clock_counter_value();
	
	__WFI();
	
	SleepingTime += clock_counter_value() - t;
	
	l = clock_counter_value();
	
	if ((tick_ct - LastTime) >= 1000) {
		LastTime = tick_ct;

		if (test_number == 4) {
			print("M0:: Prime: %u; W: %u; S: %u; L: %8.2f\r\n", PRIME_TEST, WorkingTime, SleepingTime, ((float)WorkingTime / (float)(WorkingTime + SleepingTime) * 100));
		}
		SleepingTime = 0;
		WorkingTime = 0;
	}
}
