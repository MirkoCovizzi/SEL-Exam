#include <stdint.h>

#include "ipc_mbx.h"

/* place in a specific named section, for the linker scatter file */
Mbx Master_mbxTable[NUM_MASTER_MBX] __attribute__((section("Master_mbxTable")));

/* place in a specific named section, for the linker scatter file */ 
Mbx Slave_mbxTable[NUM_SLAVE_MBX] __attribute__((section("Slave_mbxTable")));

	






