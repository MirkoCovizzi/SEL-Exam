/***********************************************************************
 * $Id:: matrix.h 8242 2011-10-11 15:15:25Z nxp28536                   $
 *
 * Project: LPC43xx Common
 *
 * Description:
 *     Header file for matrix display on the validation board.
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
#ifndef __MATRIX_H 
#define __MATRIX_H

//#define USE_DMA
//#define USE_GREEN

typedef enum COLOR 
{
	RED = 1,
	GREEN = 2,
	ORANGE = 3,
}COLOR_Type;

extern void init_matrix(void);
extern void matrix_display(void);
extern void matrix_update(void);
extern void copy_to_matrix(int8_t x, int8_t y, char sprite[8], COLOR_Type c);
extern void copy_to_buffer(char sprite[8]);
extern void matrix_scrollstr(void);
extern char reverse_bits(char *value);
extern void matrix_clr(void);
extern void matrix_setpix(char x, char y, COLOR_Type c);
extern void matrix_clrpix(char x, char y);
extern void matrix_putc(char ch, char offset, COLOR_Type c);
extern void matrix_puts(char *p, COLOR_Type c);
extern void matrix_put_int(uint8_t i, COLOR_Type c);
extern void scroll_animation(char sprite_1[], char sprite_2[]);
 

#endif /* end __MATRIX_H */
/*****************************************************************************
**                            End Of File
******************************************************************************/
