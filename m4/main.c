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

#define CPU 0
#define OTHER_CPU 1

#define REQUEST_RATE 50000 // In DEBUG mode the request rate is 10 per second

/*
	3
	101
	1009
	10007
	100003
	1000003
	10000019
*/
#define PRIME_REQUEST 101

#define PRECISION 200

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
static volatile uint32_t total_delta_time;
static volatile uint32_t total_received;

static volatile int notify = 0;
static volatile int received = 0;
static volatile int finished = 0;
static volatile int test_number = 0;
static volatile int test_running = 0;

static volatile uint32_t l = 0;
static volatile uint32_t WorkingTime = 0;
static volatile uint32_t SleepingTime = 0;
static volatile uint32_t LastTime = 0;

static int debug_toggle = 0;
static int cpu_load_toggle = 0;

void print(char *, ...);
void debug(char *, ...);
void lock_mutex(volatile shared_mutex *);
void unlock_mutex(volatile shared_mutex *);
uint32_t clock_counter_value(void);
void idle(void);

static float pi = 0;

static volatile uint32_t tick_ct = 0;

void M0APP_IRQHandler(void)
{
	Chip_CREG_ClearM0AppEvent();
	notify = 1;
	
	if (test_number == 1 && test_running) {
		uint32_t result, delta_time;
		uint16_t iter;
		
		ipcex_msg_t msg;
		
		if (IPC_tryPopMsg(&msg) != QUEUE_VALID) {
			return;
		}
		
		result = msg.data0;
		iter = msg.id.pid;
		delta_time = clock_counter_value() - msg.data1;
		total_delta_time += delta_time;
		total_received++;
		
		debug("M4:: RESPONSE COUNTER: %d; RESULT: %u; DELTA TIME: %u;\r\n", iter, result, delta_time);
	}
}

void TIMER0_IRQHandler(void)
{
	if (Chip_TIMER_MatchPending(LPC_TIMER0, 1)) {
		Chip_TIMER_ClearMatch(LPC_TIMER0, 1);

		if (test_number == 1) {
			if (iteration < ITERATIONS) {
				ipcex_msg_t msg;
				int ret;
					
				msg.id.pid = iteration;
				msg.id.cpu = (uint16_t) CPUID_M0APP;
				msg.data0 = PRIME_REQUEST;
				msg.data1 = clock_counter_value();
							
				if (IPC_tryPushMsg(msg.id.cpu, &msg) == QUEUE_INSERT) iteration++;
			} else finished = 1;
		} else if (test_number == 2) {
			if (iteration < ITERATIONS) {
				if (!received) {
					uint32_t prim;
						
					prim = __get_PRIMASK();
					__disable_irq();
					lock_mutex(mutex);
						
					debug("M4:: Entered Critical Section\r\n");
						
					if (shared_msg->counter % 2 == 0) {			
							
						shared_msg->counter = shared_msg->counter + 1;
						shared_msg->data0 = PRIME_REQUEST;
						shared_msg->data1 = clock_counter_value();
							
						iteration++;
							
						received = 1;
							
						debug("M4:: REQUEST COUNTER: %u; REQUEST: %u; TIMESTAMP: %u;\r\n", shared_msg->counter, shared_msg->data0, shared_msg->data1);
					}
						
					debug("M4:: Exiting Critical Section\r\n");
						
					unlock_mutex(mutex);
					if (!prim) __enable_irq();
				}
			}
		}
	}
}

void TIMER1_IRQHandler(void)
{
	if (Chip_TIMER_MatchPending(LPC_TIMER1, 1)) {
		Chip_TIMER_ClearMatch(LPC_TIMER1, 1);
		tick_ct += 1;
	}
}

static void wait_event(void) {
	__WFE();
	while(!notify);
	notify = 0;
	print("M4:: RECEIVED NOTIFY!\r\n");
}

static void notify_event(void) {
	__DSB();
	__SEV();
}

static void init_mutex(volatile shared_mutex *mutex) {
	mutex->b[CPU] = 1;
	mutex->c[CPU] = 1;
	mutex->b[OTHER_CPU] = 1;
	mutex->c[OTHER_CPU] = 1;
	mutex->k = 0;
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

static void print_menu(void) {
	print("M4:: Choose one of the following tests:\r\n");
	print("  1) Message Queue Communication (using event interrupts)\r\n");
	print("  2) Mutex Message Communication (not using event interrupts)\r\n");
	print("  3) Check Core M4 PI calculation CPU usage\r\n");
	print("  4) Check Core M0APP prime check CPU usage\r\n");
	print("  5) Toggle CPU LOAD\r\n");
	print("  6) Toggle Debug\r\n");
	print("  7) Exit Program\r\n");
	print("> ");
}

static void setup_board(void) {
	SystemCoreClockUpdate();
	Board_Init();
}

static void boot_M0App(void) {
	if (M0Image_Boot(CPUID_M0APP, (uint32_t) BASE_ADDRESS_M0APP) < 0) {
		print("M4:: Unable to BOOT M0APP Core!\r\n");
	} else {
		print("M4:: M0APP Core BOOT successful!\r\n");
	}
}

static void shutdown_M0App(void) {
	Chip_Clock_Disable(CLK_M4_M0APP);
	print("M4:: M0APP Core shutdown successful!\r\n");
}

static void setup_uart(void) {
	Chip_UART_Init(LPC_UART);
	Chip_UART_SetBaud(LPC_UART, 115200);
	Chip_UART_ConfigData(LPC_UART, UART_LCR_WLEN8 | UART_LCR_SBS_1BIT); /* Default 8-N-1 */

	Chip_UART_TXEnable(LPC_UART);
	Chip_UART_SetupFIFOS(LPC_UART, (UART_FCR_FIFO_EN | UART_FCR_RX_RS | UART_FCR_TX_RS | UART_FCR_TRG_LEV0));
}

static void setup_clock_counter(void) {
	/* Enable TRC */
  CoreDebug->DEMCR &= ~0x01000000;
  CoreDebug->DEMCR |=  0x01000000;
	
	/* Enable counter */
  DWT->CTRL &= ~0x00000001;
  DWT->CTRL |=  0x00000001;
	
  /* Reset counter */
  DWT->CYCCNT = 0;
}

static uint32_t clock_counter_value(void) {
	return DWT->CYCCNT;
}

static void setup_system_timer(void) {
	uint32_t timerFreq;
	
	/* Enable timer 1 clock and reset it */
	Chip_TIMER_Init(LPC_TIMER1);
	Chip_RGU_TriggerReset(RGU_TIMER1_RST);
	while (Chip_RGU_InReset(RGU_TIMER1_RST)) {}

	/* Get timer 1 peripheral clock rate */
	timerFreq = Chip_Clock_GetRate(CLK_MX_TIMER1);

	/* Timer setup for match and interrupt at 1000 HZ */
	Chip_TIMER_Reset(LPC_TIMER1);
	Chip_TIMER_MatchEnableInt(LPC_TIMER1, 1);
	Chip_TIMER_SetMatch(LPC_TIMER1, 1, (timerFreq / 1000));
	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER1, 1);
	
	/* Enable timer interrupt */
	NVIC_EnableIRQ(TIMER1_IRQn);
	NVIC_ClearPendingIRQ(TIMER1_IRQn);
}

static void start_system_timer(void) {
	Chip_TIMER_Enable(LPC_TIMER1);
}

static void stop_system_timer(void) {
	Chip_TIMER_Disable(LPC_TIMER1);
}

static void setup_request_timer(uint32_t rate) {
	uint32_t timerFreq;
		
	/* Enable timer 0 clock and reset it */
	Chip_TIMER_Init(LPC_TIMER0);
	Chip_RGU_TriggerReset(RGU_TIMER0_RST);
	while (Chip_RGU_InReset(RGU_TIMER0_RST)) {}

	/* Get timer 0 peripheral clock rate */
	timerFreq = Chip_Clock_GetRate(CLK_MX_TIMER0);

	/* Timer setup for match and interrupt at TICKRATE_HZ_TIM0 */
	Chip_TIMER_Reset(LPC_TIMER0);
	Chip_TIMER_MatchEnableInt(LPC_TIMER0, 1);
	Chip_TIMER_SetMatch(LPC_TIMER0, 1, (timerFreq / rate));
	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER0, 1);
		
	/* Enable timer interrupt */
	NVIC_EnableIRQ(TIMER0_IRQn);
	NVIC_ClearPendingIRQ(TIMER0_IRQn);
}

static void start_request_timer(void) {
	Chip_TIMER_Enable(LPC_TIMER0);
}

static void stop_request_timer(void) {
	Chip_TIMER_Disable(LPC_TIMER0);
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
	FlagStatus exitflag;
	uint8_t buffer[10];
	int i;
	
	setup_board();
	setup_system_timer();
	
	setup_clock_counter();
	setup_uart();
	init_mutex(print_mutex);
	
	NVIC_EnableIRQ(M0APP_IRQn);
	
	print("M4:: Started M4/M0 Multicore Benchmark\r\n");
	
	exitflag = RESET;
	while (exitflag == RESET) {
		print_menu();
		
		Chip_UART_ReadBlocking(LPC_UART, buffer, 2);
		
		if ((buffer[0] != '1') && (buffer[0] != '2') && (buffer[0] != '3') && (buffer[0] != '4') && (buffer[0] != '5') && (buffer[0] != '6') && (buffer[0] != '7')) {
			print("M4:: Error: Choose one of the options provided.\r\n");
			memset(buffer, 0x00, sizeof(buffer));
			continue;
		}
		
		switch (buffer[0]) {
			case '1':
				print("M4:: Setting up test #1...\r\n");
			
				IPCEX_Init();
			
				test_number = 1;
				iteration = 0;
				finished = 0;
				total_delta_time = 0;
				total_received = 0;
			
				boot_M0App();
			
				wait_event();

				shared_msg->data0 = buffer[0];
				shared_msg->data1 = debug_toggle;
				shared_msg->data2 = cpu_load_toggle;

				notify_event();
			
				print("M4:: Waiting M0APP to finish setting up test #1...\r\n");
				
				wait_event();
				
				print("M4:: Starting test #1...\r\n");
				
				if (debug_toggle) {
					setup_request_timer(10);
					print("M4:: REQUEST RATE: 10\r\n");
				} else {
					setup_request_timer(REQUEST_RATE);
					print("M4:: REQUEST RATE: %u\r\n", REQUEST_RATE);
				}
				
				test_running = 1;
				
				start_request_timer();
				while(!finished) {
					if (cpu_load_toggle) pi = calc_pi(PRECISION);
					
					__WFI();
				}
				stop_request_timer();
				
				while (total_received < ITERATIONS) {
					notify_event();
				}
				
				wait_event();
				
				test_running = 0;
				
				print("M4:: TOTAL SENT: %u\r\n", iteration);
				print("M4:: TOTAL RECEIVED: %u\r\n", total_received);
				print("M4:: AVERAGE DELTA TIME: %10.2f\r\n", ((float) total_delta_time / (float) total_received));
				print("M4:: Finished test #1.\r\n");
				
				shutdown_M0App();

				break;
			case '2':
				print("M4:: Setting up test #2...\r\n");
			
				test_number = 2;
				iteration = 0;
				finished = 0;
				total_delta_time = 0;
				total_received = 0;
			
				init_mutex(mutex);
				shared_msg->counter = 0;
			
				boot_M0App();
			
				wait_event();

				shared_msg->data0 = buffer[0];
				shared_msg->data1 = debug_toggle;
				shared_msg->data2 = cpu_load_toggle;
				
				notify_event();
			
				print("M4:: Waiting M0APP to finish setting up test #2...\r\n");
				
				wait_event();
				
				print("M4:: Starting test #2...\r\n");
				
				if (debug_toggle) {
					setup_request_timer(10);
					print("M4:: REQUEST RATE: 10\r\n");
				} else {
					setup_request_timer(REQUEST_RATE);
					print("M4:: REQUEST RATE: %u\r\n", REQUEST_RATE);
				}
				
				start_request_timer();
				
				while(!finished) {
					if (cpu_load_toggle) pi = calc_pi(PRECISION);
					
					if (received) {
						uint32_t prim;
						
						prim = __get_PRIMASK();
						__disable_irq();
						lock_mutex(mutex);
						
						debug("M4:: Entered Critical Section (receive)\r\n");
						
						if (shared_msg->counter % 2 == 0) {
							uint32_t result, delta_time;
							
							delta_time = clock_counter_value() - shared_msg->data1;
							
							total_delta_time += delta_time;
							total_received++;
							
							received = 0;
							
							if (iteration >= ITERATIONS) finished = 1;
							
							debug("M4:: RESPONSE COUNTER: %u; RESULT: %u; DELTA TIME: %u;\r\n", shared_msg->counter, shared_msg->data0, delta_time);
						}
						
						debug("M4:: Exiting Critical Section (receive)\r\n");
						
						
						unlock_mutex(mutex);
						if (!prim) __enable_irq();
					}
				}
				
				stop_request_timer();
				
				wait_event();
				
				print("M4:: TOTAL SENT: %u\r\n", iteration);
				print("M4:: TOTAL RECEIVED: %u\r\n", total_received);
				print("M4:: AVERAGE DELTA TIME: %10.2f\r\n", ((float) total_delta_time / (float) total_received));
				print("M4:: Finished test #2.\r\n");
				
				shutdown_M0App();

				break;
			case '3':
				print("M4:: Setting up test #3...\r\n");
				test_number = 3;
			
				print("M4:: Starting test #3...\r\n");
			
				start_system_timer();
				for (i = 0; i < 10000; i++) {
					if (cpu_load_toggle) pi = calc_pi(PRECISION);
					
					idle();
				}
				stop_system_timer();
				
				print("M4:: Finished test #3.\r\n");
				
				break;
			case '4':
				print("M4:: Setting up test #4...\r\n");
			
				test_number = 4;
			
				boot_M0App();
			
				wait_event();

				shared_msg->data0 = buffer[0];
				shared_msg->data1 = debug_toggle;
				shared_msg->data2 = cpu_load_toggle;
				
				notify_event();
			
				print("M4:: Waiting M0APP to finish setting up test #4...\r\n");
				
				wait_event();
				
				wait_event();
				
				shutdown_M0App();

				break;
			case '5':
				if (cpu_load_toggle == 0) {
					cpu_load_toggle = 1;
					print("M4:: CPU LOAD is now ON.\r\n");
				} else if (cpu_load_toggle == 1) {
					cpu_load_toggle = 0;
					print("M4:: CPU LOAD is now OFF.\r\n");
				}
				break;				
			case '6':
				if (debug_toggle == 0) {
					debug_toggle = 1;
					print("M4:: DEBUG is now ON.\r\n");
				} else if (debug_toggle == 1) {
					debug_toggle = 0;
					print("M4:: DEBUG is now OFF.\r\n");
				}
				break;
			case '7':
				exitflag = SET;
				break;
		}
	}
	
	print("M4:: Program terminated.\r\n");
	
	/* Wait for current transmission complete - THR must be empty */
	while (Chip_UART_CheckBusy(LPC_UART) == SET) {}

	/* DeInitialize UART0 peripheral */
	Chip_UART_DeInit(LPC_UART);
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

		if (test_number == 3) {
			print("M4:: PI: %5.15f; Precision: %u; W: %u; S: %u; L: %8.2f\r\n", pi, PRECISION, WorkingTime, SleepingTime, ((float)WorkingTime / (float)(WorkingTime + SleepingTime) * 100));
		}
		SleepingTime = 0;
		WorkingTime = 0;
	}
}
