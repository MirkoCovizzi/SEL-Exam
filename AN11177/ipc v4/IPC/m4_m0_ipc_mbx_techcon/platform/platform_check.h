#ifndef __PLATFORM_CHECK_H
#define __PLATFORM_CHECK_H

#warning "************ PLATFORM CONFIG ************"

/* this is used to check the build platform */
#if (PLATFORM == HITEX_A2_BOARD)
	#warning "*** Building for HITEX LPC4350EVA platform ***"	
#else
	#error "!!! Platform not supported, check platform_config.h !!!"
#endif

/* this is used to check the build device */
#if (DEVICE == LPC43xx) 
	#warning "Building for LPC43xx device"	
#elif (DEVICE == newDevice)
	#warning "Building for XYZ device"	
#else
	#error "!!! Device not supported, check platform_config.h !!!"
#endif

#ifdef IPC_MASTER
/* this is to ensure memory ranges are defined for the master */
#ifndef MASTER_ROM_START
	#error "!!! M4_ROM_START not defined, check platform_config.h !!!"
#endif
#ifndef MASTER_ROM_LEN
	#error "!!! M4_ROM_LEN not defined, check platform_config.h !!!"
#endif
#ifndef MASTER_RAM_START
	#error "!!! M4_RAM_START not defined, check platform_config.h !!!"
#endif
#ifndef MASTER_RAM_LEN
	#error "!!! M4_RAM_LEN not defined, check platform_config.h !!!"
#endif
#endif

#ifdef IPC_SLAVE
/* this is to ensure memory ranges are defined for the slave */
#ifndef SLAVE_ROM_START
	#error "!!! M0_ROM_START not defined, check platform_config.h !!!"
#endif
#ifndef SLAVE_ROM_LEN
	#error "!!! M0_ROM_LEN not defined, check platform_config.h !!!"
#endif
#ifndef SLAVE_RAM_START
	#error "!!! M0_RAM_START not defined, check platform_config.h !!!"
#endif
#ifndef SLAVE_RAM_LEN
	#error "!!! M0_RAM_LEN not defined, check platform_config.h !!!"
#endif
#endif

/* feedback for mailboxes usage */

/* check the build rules */
#if (USE_EXT_FLASH == YES)
	#warning "Building with external flash support: YES"
#elif (USE_EXT_FLASH == NO)
	#warning "Building with external flash support: NO"
#endif

#if (USE_EXT_STATIC_MEM == YES)
	#warning "Building with external static memory support: YES"
#elif (USE_EXT_STATIC_MEM == NO)
	#warning "Building with external static memory support: NO"
#endif

#if (USE_EXT_DYNAMIC_MEM == YES)
	#warning "Building for external dynamic memory support: YES"
#elif (USE_EXT_DYNAMIC_MEM == NO)
	#warning "Building for external dynamic memory support: NO"
#endif

#warning "************ PLATFORM CONFIG ************"



#endif /* platform check */




