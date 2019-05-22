#ifndef __IPC_H__
#define __IPC_H__

#include "platform_config.h"

/* definition of interrupt status */
#define MSG_PENDING (1)
#define NO_MSG		(2)

typedef uint8_t intFlag_t;

#define LOOP_INSTRUCTION	0xE7FEE7FE

/**********************************************/
/* functions for interrupt handling */
/**********************************************/
/**********************************************/
/* callback definition */
/**********************************************/
	
typedef void (*intCallback_t) (void);

/**********************************************/
/* local interrupt functions */
/**********************************************/

/* reset the interrupt flag */
/* shortly disables interrupts */
void IPC_resetIntFlag(void);

/**********************************************/
/* remote mailbox functions 				  */
/**********************************************/

/* send a message to a specific mailbox */
/* configures the mailbox as process, triggers an interrupt to the remote cpu */
void IPC_sendInterrupt(void);

extern volatile intFlag_t intFlag;

#include "master_interrupt_callback.h"
#include "slave_interrupt_callback.h"

/**********************************************/
/* functions to initialize the framework */
/**********************************************/

/* download a processor image to the slave CPU */
void IPC_downloadSlaveImage(uint32_t SlaveRomStart, const unsigned char slaveImage[], uint32_t imageSize);

/* take processor out of reset */
void IPC_startSlave(void);

/* put the processor back in reset */
void IPC_haltSlave(void);

/* initialize the interrupt ipc framework */
void IPC_slaveInitInterrupt(intCallback_t slaveCback);
void IPC_masterInitInterrupt(intCallback_t masterCback);


#endif /* __IPC_H__ */
