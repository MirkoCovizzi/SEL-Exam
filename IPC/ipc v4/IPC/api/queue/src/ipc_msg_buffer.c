#include <stdint.h>

#include "ipc_queue.h"

/* block holding registers (pointers) and data for the messages */
msgBlock hostMsgBufferControl __attribute__ ((section ("msgBuffer_section")));


/* buffer space holding messages */
msgToken _hostMsgBufferData[MSG_BUF_LEN] __attribute__ ((section ("msgBuffer_section")));

