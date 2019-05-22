
#include "LPC43xx.H"   
#include "platform_config.h"


// all defined interrupts at compile time get placed into the flash table
// application copies system + user defined interrupts
// 48 vectors in total

extern unsigned long __Vectors;
extern unsigned long __endVectors;

#if (USE_EXT_FLASH == YES)
extern unsigned int  Image$$ER_RELOCIRQ$$Base;
unsigned int relocIntVectors[48] __attribute__ ((section("ram_isr_table")));
#endif

// this routine relocates the interrupt vector table within the internal ram 
// memory at address M4_RAM_START (default 0x1000 0000)
// This area is reserved by the linker script
// it includes space for 48 effective interrupt vectors (16 system + 32 user)
void relocIrqTable(void)
{
#if (USE_EXT_FLASH == YES)
    unsigned long *pulSrc;
    unsigned long *pulRamBase = (unsigned long*) &Image$$ER_RELOCIRQ$$Base;

    unsigned long *relocBase = pulRamBase;

    // copy the table over
    for(pulSrc = &__Vectors; pulSrc < &__endVectors; )
    {
        *relocBase++ = *pulSrc++;
    }

    // point to the new table in ram
    SCB->VTOR = (uint32_t) pulRamBase;
#endif

}
