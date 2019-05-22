#ifndef __M4_IPC_MSG_H 
#define __M4_IPC_MSG_H

/* messages the M4 shall get */
enum ipcM4Msg_tag {

	CMD_MASTER_NONE = 0,
	RESPONSE_SLAVE_STARTED,
	RESPONSE_SLAVE_LED_BLINKED
		
};


#endif
