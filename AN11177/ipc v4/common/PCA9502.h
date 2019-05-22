/***********************************************************************
 * $Id:: PCA9502.h 8242 2011-10-11 15:15:25Z nxp28536                  $
 *
 * Project: LPC43xx Common
 *
 * Description:
 *     Header file for PCA9502 configuration
 *
 ***********************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
 **********************************************************************/
#ifndef __PCA9502_H 
#define __PCA9502_H

extern void PCA9502_Init(void);
extern void PCA9502_SetIO(uint8_t value);
extern uint8_t PCA9502_ReadIO(uint8_t value);
extern void PCA9502_SetReg(uint8_t reg, uint8_t value);
extern uint8_t PCA9502_ReadReg(uint8_t reg);

#endif //__PCA9502_H
