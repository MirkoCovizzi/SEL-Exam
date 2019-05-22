#ifndef __PLATFORM_CONFIG_H 
#define __PLATFORM_CONFIG_H

/****************************************************/
/* supported platforms list							*/
/* DO NOT CHANGE THESE DEFINITIONS 					*/
#define HITEX_BOARD          (1)
/****************************************************/

/****************************************************/
/* supported devices list							*/
/* DO NOT CHANGE THESE DEFINITIONS 					*/
#define LPC43xx	(1)
/****************************************************/

/****************************************************/
/* used for the configuration checks */
/* DO NOT CHANGE THESE DEFINITIONS 					*/
/****************************************************/
#define YES	(1)
#define NO	(2)

/****************************************************/
/* USER CONFIGURATION SECTION						*/
/****************************************************/

/* choose the platform you want to build against 	*/
#define PLATFORM HITEX_BOARD

/* choose the device you want to build against 	*/
#define DEVICE	LPC43xx

/* these definitions are being taken from the build rule */
#ifdef EXT_FLASH
#define	USE_EXT_FLASH	(YES)
#else 
#define USE_EXT_FLASH	(NO)
#endif

#ifdef EXT_STAT_MEM
#define USE_EXT_STATIC_MEM	(YES)
#else
#define USE_EXT_STATIC_MEM	(NO)
#endif

#ifdef EXT_DYN_MEM
#define USE_EXT_DYNAMIC_MEM	(YES)
#else
#define USE_EXT_DYNAMIC_MEM	(NO)
#endif

/* specify the filename used for the slave image */
#define SLAVE_IMAGE_FILE "CM0_image.c"

/* Note: make sure there are no interrupts pending / active at priority zero */
/* at the point vPortStartFirstTask() executes the SVC 0 call, otherwise */
/* if SVCall has a lower priority  Cortex/M3 will issue a */
/* hard fault exception. SVCall is left at default (priority zero) in xPortStartScheduler (port.c) */
/* cmsis definition, priority from 0 to 7 */
#define MASTER_INTERRUPT_PRIORITY		(6) 

/* cmsis definition, priority from 0 to 3 */
#define SLAVE_INTERRUPT_PRIORITY	(3) 


/* memory map for the application */
/* needs to be consistent with the scatter file */
/* memory map for the application */
/* !!! needs to be consistent with the scatter file !!! */
#ifdef EXT_FLASH

/************************************/
/* this is for the FLASH version 	*/
/************************************/
/*	0x1C000000	M4 ROM 4Mbytes		*/
/*	0x1C3FFFFF						*/
/*	0x10000000	M4 RAM 96K			*/
/*	0x10017FFF						*/
#define MASTER_ROM_START	0x1C000000
#define MASTER_ROM_LEN		0x400000	/* 4 Mbytes */

#define MASTER_RAM_START	0x10000000	/* 96 Kbytes */
#define MASTER_RAM_LEN		0x18000

/*	0x10080000	M0a ROM 16K	*/
/*	0x10083FFF				*/
/*	0x10084000 	M0a RAM 24K	*/
/*	0x10089FFF 				*/
#define SLAVE_ROM_START	0x10080000
#define SLAVE_ROM_LEN		0x4000

#define SLAVE_RAM_START	0x10084000
#define SLAVE_RAM_LEN		0x6000

/*	0x20000000  M4 BUF 16K	*/
/*	0x20003FFF				*/
/*	0x20004000	M0 BUF	16K	*/
/*	0x20007FFF				*/
#define MASTER_BUF_START	0x20000000
#define MASTER_BUF_LEN		0x4000

#define SLAVE_BUF_START	0x20004000
#define SLAVE_BUF_LEN		0x4000


#else 

/*******************************/
/* this is for the ram version */
/*******************************/
/*	0x10000000	M4 ROM 64K	*/
/*	0x1000FFFF				*/
/*	0x10010000	M4 RAM 32K	*/
/*	0x10017FFF				*/
#define MASTER_ROM_START	0x10000000
#define MASTER_ROM_LEN		0x10000

#define MASTER_RAM_START	0x10010000
#define MASTER_RAM_LEN		0x8000

/*	0x10080000	M0a ROM 16K	*/
/*	0x10083FFF				*/
/*	0x10084000 	M0a RAM 24K	*/
/*	0x10089FFF 				*/
#define SLAVE_ROM_START	0x10080000
#define SLAVE_ROM_LEN		0x4000

#define SLAVE_RAM_START	0x10084000
#define SLAVE_RAM_LEN		0x6000

/*	0x20000000  M4 BUF 16K	*/
/*	0x20003FFF				*/
/*	0x20004000	M0a BUF	16K	*/
/*	0x20007FFF				*/
#define MASTER_BUF_START	0x20000000
#define MASTER_BUF_LEN		0x4000

#define SLAVE_BUF_START	0x20004000
#define SLAVE_BUF_LEN		0x4000

#endif

/****************************************************/
/* END OF USER CONFIGURATION 						*/
/* DO NOT EDIT BELOW THIS LINE						*/
/****************************************************/
/* assign the roles for the devices */
#if (DEVICE==LPC43xx)

#include "LPC43xx.h"

#define MASTER_CPU CORE_M4
#define SLAVE_CPU CORE_M0

#define MASTER_IRQn (1)
#define SLAVE_IRQn 	(1)

#define MASTER_TXEV_FLAG 	((uint32_t *) 0x40043130)
#define MASTER_TXEV_QUIT() 	{ *MASTER_TXEV_FLAG = 0x0; }

#define SLAVE_TXEV_FLAG ((uint32_t *) 0x40043400)
#define SLAVE_TXEV_QUIT() { *SLAVE_TXEV_FLAG = 0x0; }

#define SLAVE_SHADOW_REG	0x40043404

#endif


/****************************************************/
/* platform wise initialization functions			*/
/****************************************************/
void platformInit(void);

#endif /* __PLATFORM_CONFIG_H */

