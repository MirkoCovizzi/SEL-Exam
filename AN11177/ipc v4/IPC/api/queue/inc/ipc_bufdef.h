#ifndef __IPC_BUFDEF_H 
#define __IPC_BUFDEF_H


/* accomodate the required number of messages, on the biggest message type plus one for the ring buffer */
#define MSG_BUF_LEN (SLAVE_MSGBUF_SIZE + 1)

typedef struct msgBlock_tag {

	/* read and write pointers for accessing the command buffer */
	msgToken* msgQstartAddress;
	msgToken* msgQendAddress;
	msgToken* msgRdPtr;
	msgToken* msgWrPtr;
	
} msgBlock;

/* buffer space holding messages */
extern msgToken _hostMsgBufferData[MSG_BUF_LEN];


/* accomodate the required number of commands plus one for the ring buffer */
#define CMD_BUF_LEN (MASTER_CMDBUF_SIZE + 1)

typedef struct cmdBlock_tag {

	/* read and write pointers for accessing the command buffer */
	cmdToken* cmdQstartAddress;
	cmdToken* cmdQendAddress;
	cmdToken* cmdRdPtr;
	cmdToken* cmdWrPtr;
	
} cmdBlock;

/* buffer space holding commands */
extern cmdToken _hostCmdBufferData[CMD_BUF_LEN];

#endif /**/

