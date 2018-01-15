/************************************************************************/
/*																		*/
/*	IR_Board_Defs.h --	Board Customization for custom IR Board		    */

/************************************************************************/

#if !defined(IR_BOARD_DEFS_H)
#define IR_BOARD_DEFS_H

#include <inttypes.h>

/* ------------------------------------------------------------ */
/*				Public Board Declarations						*/
/* ------------------------------------------------------------ */
/* The following define symbols that can be used in a sketch to
** refer to periperhals on the board generically.
*/

#define	_BOARD_NAME_	"IRBoard Rev A"

/* ------------------------------------------------------------ */
/*					Global Variables							*/
/* ------------------------------------------------------------ */

#define TICKS_PER_SECOND 80000000
#define T3_ON 0x8000
#define Ext_Int0_mask 0x8
#define Timer1_mask 0x10
#define OC1_mask 0x40
#define Ext_Int1_mask 0x80
#define Timer2_mask 0x100
#define OC2_mask 0x400
#define Ext_Int2_mask 0x800
#define Timer3_mask 0x1000
#define OC3_mask 0x4000
#define Ext_Int3_mask 0x8000
#define Timer4_mask 0x10000
#define OC4_mask 0x40000
#define Ext_Int4_mask 0x80000
#define Timer5_mask 0x100000
#define OC5_mask 0x400000
#define identification_byte 0xF1
#define R2D2_byte1 0xB3
#define R2D2_byte2 0x0D
#define BB9E_byte1 0xA2
#define BB9E_byte2 0x83
#define LMQ_byte1 0xA8
#define LMQ_byte2 0x77

/* ------------------------------------------------------------ */
/*					Timer Pin Declarations						*/
/* ------------------------------------------------------------ */

#define PWM_OUTPUT		3
#define	OC2				5
#define	OC3				6
#define	OC4				9	


/* ------------------------------------------------------------ */
/*					Interrupt Pin Declarations					*/
/* ------------------------------------------------------------ */

#define	INT0	38
#define	INT1	2
#define INT2	7
#define	INT3	8
#define	INT4	35


/* ------------------------------------------------------------ */
/*					Other Pins									*/
/* ------------------------------------------------------------ */
/* These define the pin numbers for the various change notice
** pins.
*/

#define	VDDC		37
#define	VBATC		36
#define	D/C			34
#define	SCK2		13
#define	MISO		12
#define	MOSI		11
#define	SS2			10
#define	LED17		33
#define	LED16		32
#define	PWM5_EN		31
#define	PWM4_EN		30
#define	PWM3_EN		29
#define	PWM2_EN		28
#define	PWM1_EN		27
#define	PWM0_EN		26
#define	RF1			4
//#define	I2C_UC32_SCL	37
//#define	I2C_UC32_SDA	36


#endif	// IR_BOARD_DEFS_H

/************************************************************************/
