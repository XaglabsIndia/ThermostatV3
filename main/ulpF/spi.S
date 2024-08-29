
/* ULP assembly files are passed through C preprocessor first, so include directives
   and C macros may be used in these files 
 */

#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "stack.S"
#define esp32
#ifdef esp32
//; change RTC pins here based on the GPIO 's used  
//; Here the RTC GPIO's and GPIO's are different so make sure RTC pins should match regarding their gpio pin used 
//;  THese are GPIO pins					```|04|25|26|27|32|33|34|35|36|37|38|39|15|13|12|14|```
//;   and their respective RTC pins  		```|10|06|07|17|09|08|04|05|00|01|02|03|13|14|15|16|```
#define MISO 15
#define MOSI 14
#define CS 13
#define SCLK 16
#endif
#ifndef esp32
#define MISO 7//;7 gpio
#define MOSI 5//; 5 gpio
#define CS 4//;
#define SCLK 6
#endif

	.set bit_mask, 	0x80
	.set bit_len, 	0x08

	.set SPI_MODE_1,	1		/* Mode_1, Clock Polarity is 0 and Clock Phase is 0 */
	.set SPI_MASTER,	0		/* SPI Master */
	.set SPI_SLAVE,		1		/* SPI Slave */
	.set SPI_MODE_SET,	SPI_MODE_1
	.set SPI_TYPE_SET,	SPI_MASTER

.bss
	.global spi_mode
spi_mode:
	.long	0

	/* Code goes into .text section */
	.text

.macro spi_delay
	nop
.endm

.macro read_MISO 
	READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + MISO, 1) 
.endm

.macro clear_SCLK
	WRITE_RTC_REG(RTC_GPIO_OUT_W1TC_REG, RTC_GPIO_OUT_DATA_W1TC_S + SCLK, 1, 1)
.endm
.macro set_SCLK
	WRITE_RTC_REG(RTC_GPIO_OUT_W1TS_REG, RTC_GPIO_OUT_DATA_W1TS_S + SCLK, 1, 1)
.endm

.macro clear_MOSI
	WRITE_RTC_REG(RTC_GPIO_OUT_W1TC_REG, RTC_GPIO_OUT_DATA_W1TC_S + MOSI, 1, 1)
.endm
.macro set_MOSI
	WRITE_RTC_REG(RTC_GPIO_OUT_W1TS_REG, RTC_GPIO_OUT_DATA_W1TS_S + MOSI, 1, 1)
.endm

.macro clear_CS 
	WRITE_RTC_REG(RTC_GPIO_OUT_W1TC_REG, RTC_GPIO_OUT_DATA_W1TC_S + CS, 1, 1)
.endm
.macro set_CS 
	WRITE_RTC_REG(RTC_GPIO_OUT_W1TS_REG, RTC_GPIO_OUT_DATA_W1TS_S + CS, 1, 1)
.endm


	.global	SPI_Write_Byte 		/* r2 save the data to be sent out */
SPI_Write_Byte:
write_mode_1:					/* Clock Polarity is 0 and Clock Phase is 0 */
	stage_rst
	clear_SCLK
write_loop1:
	clear_SCLK
	and r0, r2, bit_mask
	lsh r2, r2, 1
	jumpr loop1_bit0, 1, lt
	set_MOSI
	jump loop1_bit1
loop1_bit0:
	clear_MOSI
loop1_bit1:
	spi_delay
	set_SCLK
	spi_delay
	stage_inc 1
	jumps write_loop1, bit_len, lt
	clear_SCLK
	ret


/* spi read function */
	.global	SPI_Read_Byte
SPI_Read_Byte:
	set_MOSI
read_mode_1: 					/* Clock Polarity is 0 and Clock Phase is 0 */
	clear_SCLK
	stage_rst
read_loop1:
	clear_SCLK
	spi_delay
	set_SCLK
	read_MISO
	spi_delay
	lsh r2, r2, 1
	or r2, r2, r0
	stage_inc 1
	jumps read_loop1, bit_len, lt
	clear_SCLK
	ret
	
