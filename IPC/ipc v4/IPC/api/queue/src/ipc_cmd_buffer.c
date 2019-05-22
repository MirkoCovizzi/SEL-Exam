#include <stdint.h>

#include "ipc_queue.h"

/* block holding registers (pointers) and data for the commands */
cmdBlock hostCmdBufferControl __attribute__ ((section ("cmdBuffer_section")));

cmdToken _hostCmdBufferData[CMD_BUF_LEN] __attribute__ ((section ("cmdBuffer_section")));


