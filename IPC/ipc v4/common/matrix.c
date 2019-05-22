/***********************************************************************
 * $Id:: matrix.c 8242 2011-10-11 15:15:25Z nxp28536                   $
 *
 * Project: LPC43xx Common
 *
 * Description:
 *     This file contains functions to drive the 8x8 Matrix display 
 *     on the LPC4300 validation board
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
#include <stdio.h>
#include <string.h>
#include "LPC43xx.H"                    
#include "type.h"
#include "font.h"
#include "matrix.h"
#include "dma.h"
#include "stdint.h"
#include "PCA9502.h"


// Change these values to adjust scroll speeds and animation iterations
#define ANIMATION_SCROLL_SPEED 80  // how fast to scroll the animations
#define TEXT_SCROLL_SPEED 120      // how fast to scrill the text
#define REPEAT_ANIMATION 10        // how often to repeat the animation if in cycling mode
#define REPEAT_TEXT 5              // how often to repeat the text if in cycling mode
#define MAX_ANIMATIONS 3

typedef union _MatrixStr {

	unsigned short full;
   	struct partMatrixStr
	{
    	unsigned char   ROW;
		unsigned char   COL;
	}part;
}MatrixStr;

volatile uint32_t matrixTicks;							   
volatile char display_string = 0;
static MatrixStr matrix_buf_R[8];			// screen memory Red
static MatrixStr matrix_buf_G[8];			// screen memory Green
static char active_row;						// active row
char buffer[60];                     		// stores the active message or sprite
static char message_ptr = 0;                // points to the active char in the message
static char message_displayed = 0;          // how often has the message been displayed?
static char active_char = 0;                // stores the active char
static char message_length = 0;             // stores the length of the active message
static char char_ptr = 0;                   // points to the active col in the char
static char char_length = 0;                // stores the length of the active char
static volatile uint16_t counter = 0;       // used for delay function

// How to add a new message:
// * add the new message (only upper case, see font.h)
// * adjust MAX_MESSAGES
// * add the new message to messages
// NOTE: messages may not be longer than 59 chars. Otherwise they will not fit in the buffer.
//						 = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";
static char message_00[] = "   JBK RULEZ !!! ";
static char message_01[] = "   WWW.NXP.COM ";
static char message_02[] = "   10   9   8   7   6   5   4   3   2   1  ...  BOOM! ";

#define MAX_MESSAGES 3
static char * messages[] = 
{
  message_00
  ,message_01
  ,message_02
}; 

char sprite_00[] = 
{
	0x18,    // ___XX___ 
	0x3C,    // __XXXX__
	0x7E,    // _XXXXXX_
	0xDB,    // X_XXXX_X
	0xFF,    // XXXXXXXX
	0x24,    // __X__X__
	0x5A,    // _X_XX_X_
	0xA5     // X_X__X_X
};

char sprite_01[8] =
{
	0x18,    // ___XX___ 
	0x3C,    // __XXXX__
	0x7E,    // _XXXXXX_
	0xDB,    // X_XXXX_X
	0xFF,    // XXXXXXXX
	0x24,    // __X__X__
	0x42,    // _X____X_
	0x24     // __X__X__
};

char sprite_02[8] =
  {
    0x00,    // ________ 
    0x00,    // ________
    0x14,    // ___X_X__
    0x3E,    // __XXXXX_
    0x3E,    // __XXXXX_
    0x1C,    // ___XXX__
    0x08,    // ____X___
    0x00     // ________
  };

char sprite_03[8] =
  {
    0x00,    // ________ 
    0x66,    // _XX__XX_
    0xFF,    // XXXXXXXX
    0xFF,    // XXXXXXXX
    0xFF,    // XXXXXXXX
    0x7E,    // _XXXXXX_
    0x3C,    // __XXXX__
    0x18     // ___XX___
  };

char sprite_04[8] =
  {
    0x24,    // __X__X__
    0x7E,    // _XXXXXX_
    0xDB,    // XX_XX_XX
    0xFF,    // XXXXXXXX
    0xA5,    // X_X__X_X
    0x99,    // X__XX__X
    0x81,    // X______X
    0xC3     // XX____XX
  };

char sprite_05[8] =
  {
    0x24,    // __X__X__
    0x18,    // ___XX___
    0x7E,    // X_XXXX_X
    0xDB,    // XX_XX_XX
    0xFF,    // XXXXXXXX
    0xDB,    // X_XXXX_X
    0x99,    // X__XX__X
    0xC3     // XX____XX
  };

void init_matrix(void) 
{
	uint8_t i;

	// Red column 1tm 8
	LPC_SCU->SFSP0_0 = 0x0;		// function 2, GPIO_0
	LPC_SCU->SFSP0_1 = 0x0;		// function 2, GPIO_1
	LPC_SCU->SFSP1_15 = 0x0;	// function 2, GPIO_2
	LPC_SCU->SFSP1_16 = 0x0;	// function 2, GPIO_3
	LPC_SCU->SFSP4_9 = 0x0;		// function 3, GPIO_4	// no GPIO!!!
	LPC_SCU->SFSP6_6 = 0x0;		// function 2, GPIO_5	
#ifndef EXT_FLASH
 	LPC_SCU->SFSP6_7 = 0x0;		// function 2, GPIO_6	// no GPIO!!!
	LPC_SCU->SFSP6_8 = 0x0;		// function 2, GPIO_7	// no GPIO!!!
#endif
	
	// Row 1tm 8
#ifndef EXT_FLASH
	LPC_SCU->SFSP1_1 = 0x0;		// function 0, GPIO_8
	LPC_SCU->SFSP1_2 = 0x0;		// function 0, GPIO_9
	LPC_SCU->SFSP1_3 = 0x0;		// function 2, GPIO_10
#endif
	LPC_SCU->SFSP1_4 = 0x0;		// function 2, GPIO_11
	LPC_SCU->SFSP1_17 = 0x0;	// function 3, GPIO_12
	LPC_SCU->SFSP1_18 = 0x0;	// function 3, GPIO_13
#ifndef EXT_FLASH
	LPC_SCU->SFSP2_10 = 0x0;	// function 3, GPIO_14
#endif
	LPC_SCU->SFSP1_20 = 0x0;	// function 3, GPIO_15
	
	LPC_GPIO0->DIR = 0xFFFF;	// 1 = ouput, 0 = input

	for(i=0;i<8;i++)
	{	
		matrix_buf_R[i].part.COL = ~(1<<i)&0xFF;
		matrix_buf_G[i].part.COL = ~(1<<i)&0xFF;
	}

#ifdef USE_DMA
	DMA_Init_Matrix((uint32_t)matrix_buf_Red);
#endif
#ifdef USE_GREEN
	PCA9502_Init();
#endif
}

/*
 * delay_ms
 * Uses the counter that is incremented by the ISR.
 * Max delay is 32767ms.
 */
void delay_ms(uint16_t delay) 
{
	matrixTicks=0;
	while(matrixTicks<(delay>>1));		// systicks @ 2 ms
}

/*
 * copy_to_matrix
 * Copies sprite data to the screen memory at the given position. 
 */
void copy_to_matrix(int8_t x, int8_t y, char sprite[8], COLOR_Type c)
{
	int8_t i, t;
  	char row;
  	for (i = 0; i < 8; i++) 
	{
    	t = i-y;
    	row = ((t >= 0) && (t < 8)) ? sprite[t] : 0x00;
    	row = (x >= 0) ? (row >> x) : (row << -x);

		if(c&RED)matrix_buf_R[i].part.ROW = row;
		if(c&GREEN)matrix_buf_G[i].part.ROW = row;
    	
  	}
}

char reverse_bits(char *value)
{
    char h = 0, i;
    // loop through all the bits
    for(i = 0; i < 8; i++)
    {
          // add bit from value to 1 bit left shifted variable
        h = (h << 1) + (*value & 1);
        // right shift bits by 1
        *value >>= 1;
    }
    return h;   
}

void matrix_update(void) 
{
#ifndef USE_DMA 
	char columns;
#endif
	
	if(display_string)
	{
		if(!matrixTicks)
		{
			matrix_scrollstr();
			matrixTicks = TEXT_SCROLL_SPEED;
		}
	}
	
	active_row = (active_row+1) % 8;				// select next row

	
#ifndef USE_DMA 
	columns = matrix_buf_R[active_row].part.ROW;
		
	// Draw Red
	LPC_GPIO0->CLR = 0x00FF;						// clear rows
	LPC_GPIO0->SET = reverse_bits(&columns);		// set row

	LPC_GPIO0->SET = 0xFF00;						// clear columns
	LPC_GPIO0->CLR = 1<<(active_row+8);				// set column
#else
	//LPC_GPIO0->PIN = matrix_buf_Red[active_row].full;
#endif

#ifdef USE_GREEN
	// Draw Green
	columns = matrix_buf_G[active_row].part.ROW;
	PCA9502_SetIO(reverse_bits(&columns));//reverse_bits(&matrix_buf_R[active_row].part.ROW));
#endif
}

/*
 * matrix_scrollstr
 * Displays the actual message. 
 * Scrolls the screen to the left and draws new pixels on the right.
 */
void matrix_scrollstr(void) 
{
	char i, b;
	
	// blit the screen to the left
	for (i = 0; i < 8; i++)matrix_buf_R[i].part.ROW <<= 1; 

	// advance a char if needed
	if (char_ptr == char_length) 
	{
		message_ptr++;
		if (message_ptr == message_length) 
		{
			message_ptr = 0;
			message_displayed++;

			display_string = 0;
		}
		active_char = buffer[message_ptr] - CHAR_OFFSET;
		char_length = font[active_char * 4 + 3];
		char_ptr = 0;
		return; // this makes the space between two chars
	}
	
	// read pixels for current column of char
	b = font[active_char * 4 + char_ptr++];
	// write pixels into screen memory
	for(i=0; i<7; i++)
	{
		if (b&(1<<i))matrix_buf_R[i+1].part.ROW |= 0x01;
	}
}

void matrix_putc(char ch, char offset, COLOR_Type z)
{
	char x, y, b;
	char active_char = ch - CHAR_OFFSET;
	char char_length = font[active_char * 4 + 3];

	for(y=offset;y<(offset+char_length);y++)
	{
		// read pixels for current column of char
		b = font[active_char * 4 + ((y-offset))];	
		for(x=0; x<7; x++) 
		{
			// write pixels into screen memory
			if((z&RED) && (b&(1<<x)))matrix_buf_R[x+1].part.ROW |= (1<<(7-y));
			if((z&GREEN) && (b&(1<<x)))matrix_buf_G[x+1].part.ROW |= (1<<(7-y));
		}
	}
}

void matrix_puts(char *p, COLOR_Type c)
{
	char *dest;
	
	message_length = 0;
	dest = buffer;
	do
	{
		*dest = *p;
		message_length++;
	}
	while(*p++ != '\0');
	display_string = 1;	
}

void matrix_put_int(BYTE i, COLOR_Type c)
{
	matrix_putc((i/10)+48,1,c);
	matrix_putc((i%10)+48,5,c);
}

void matrix_clr(void) 
{
	char i;
	for(i=0; i<8; i++)matrix_buf_R[i].part.ROW = 0x00;
	for(i=0; i<8; i++)matrix_buf_G[i].part.ROW = 0x00;
}

void matrix_setpix(char x, char y, COLOR_Type c)
{
	if(c&RED)matrix_buf_R[y].part.ROW |= (1<<(7-x));
	if(c&GREEN)matrix_buf_G[y].part.ROW |= (1<<(7-x));
}

void matrix_clrpix(char x, char y)
{
	matrix_buf_R[y].part.ROW &= ~(1<<(7-x));
	matrix_buf_G[y].part.ROW &= ~(1<<(7-x));
}

void memcpy_P(char *dest, char *src, uint32_t count) 
{
	char *dst8 = (char *)dest;
	char *src8 = (char *)src;		   
	while(count--)*dst8++ = *src8++;
}


/*
 * copy_to_buffer
 * Copies the given sprite from to RAM.
 */
void copy_to_buffer(char sprite[8]) 
{
	memcpy_P(buffer, sprite, 8);
}

void scroll_animation(char sprite_1[8], char sprite_2[8]) 
{
	char i;
	int8_t x;

	for(i = 0; i < REPEAT_ANIMATION; i++) 
	{
		copy_to_buffer(sprite_1);
		for (x = -8; x <= 0; x++) 
		{
			copy_to_matrix(x, 0, buffer, RED);
			delay_ms(ANIMATION_SCROLL_SPEED);
		}
		delay_ms(200);
		copy_to_buffer(sprite_2);
		copy_to_matrix(0, 0, buffer, RED);
		delay_ms(200);
		copy_to_buffer(sprite_1);
		copy_to_matrix(0, 0, buffer, RED);
		delay_ms(200);
		copy_to_buffer(sprite_2);
		copy_to_matrix(0, 0, buffer, RED);
		delay_ms(200);
		copy_to_buffer(sprite_1);
		for (x = 0; x < 8; x++) 
		{
			copy_to_matrix(x, 0, buffer, RED);
			delay_ms(ANIMATION_SCROLL_SPEED);
		}
	}
}
  
void matrix_display(void) 
{
	char i = 0, mode = 3;

	while (1) 
	{
		switch (mode) 
		{
			case 0:
				scroll_animation(sprite_00, sprite_01);
				break;
			case 1:
				for (i = 0; i < REPEAT_ANIMATION; i++) 
				{
					copy_to_buffer(sprite_03);
					copy_to_matrix(0, 0, buffer, RED);
					delay_ms(750);
					copy_to_buffer(sprite_02);        
					copy_to_matrix(0, 0, buffer, RED);
					delay_ms(180);
				}
				break;
			case 2:
				scroll_animation(sprite_04, sprite_05);
				break;
			default:
				strcpy(buffer, messages[mode - 3]);
				message_length = strlen(buffer);
				while (message_displayed < REPEAT_TEXT) 
				{
					matrix_scrollstr();
					delay_ms(TEXT_SCROLL_SPEED);
				}
				message_displayed = 0;
				break;
		}
			
		// cycle through all modes
		mode++;
		matrix_clr();
		if (mode >= (MAX_ANIMATIONS + MAX_MESSAGES))mode = 0;
	}
}



