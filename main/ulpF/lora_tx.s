
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
#define CS 	 	 13//;15 gpio 
#define RESET 	 07//;26 gpio 
#define SCLK     16//
#define MOSI     14
#endif


#ifndef esp32
#define CS 	 	 4//;4 gpio 
#define RESET 	 8//;8 gpio 
#define SCLK     6//;6
#define MOSI     5//;5
#endif

#define DELIMITER '@'
#define print
#define DEVID 0x44

	.set	MS5611_ADDR,             0x77 // MS5611
	.set	CMD_RESET,               0x42 // ADC reset command


	/* Define variables, which go into .bss section (zero-initialized data) */
	.bss
	.global tx_counter				/* PROM read lenth counter */
tx_counter:
	.long 0
	.global tx_fifo
tx_fifo:
	.skip 420
	.global rx_counter				/* PROM read lenth counter */
rx_counter:
	.long 0
	.global FIFOEnd
FIFOEnd:
	.long 0

	.global counter				/* PROM read lenth counter */
counter:
	.long 0
    .global init_tx
init_tx:
	.long 0
    .global tx_c
tx_c:
	.long 0

	.text
		.macro clear_MOSI
		WRITE_RTC_REG(RTC_GPIO_OUT_W1TC_REG, RTC_GPIO_OUT_DATA_W1TC_S + MOSI, 1, 1)
	.endm
	.macro clear_SCLK
		WRITE_RTC_REG(RTC_GPIO_OUT_W1TC_REG, RTC_GPIO_OUT_DATA_W1TC_S + SCLK, 1, 1)
	.endm
		.macro clear_RESET 
		WRITE_RTC_REG(RTC_GPIO_OUT_W1TC_REG, RTC_GPIO_OUT_DATA_W1TC_S + RESET, 1, 1)
	.endm
	.macro set_RESET
		WRITE_RTC_REG(RTC_GPIO_OUT_W1TS_REG, RTC_GPIO_OUT_DATA_W1TS_S + RESET, 1, 1)
	.endm
	.macro clear_CS 
		WRITE_RTC_REG(RTC_GPIO_OUT_W1TC_REG, RTC_GPIO_OUT_DATA_W1TC_S + CS, 1, 1)
	.endm
	.macro set_CS 
		WRITE_RTC_REG(RTC_GPIO_OUT_W1TS_REG, RTC_GPIO_OUT_DATA_W1TS_S + CS, 1, 1)
	.endm
	.macro spi_init
		set_CS 						/* disable CS bus */
		clear_MOSI
		clear_SCLK	
		clear_CS 				/* enable cs bus */	
	.endm
	// user defined macros which is used to write to register in spi
	.macro write_to_reg regaddress,value     					
		spi_init		
		move r0,\value
		push r0
		move r2, \regaddress			
		or r2,r2,0x80		
		psr
		jump SPI_Write_Byte 					
		pop r2		
		psr
		jump SPI_Write_Byte
		set_CS 			/* disbale CS */
	.endm
	// user defined macros which is used to read from register in spi
	.macro read_from_reg reg
		spi_init									
		move r2,\reg	 					
		psr
		jump SPI_Write_Byte 					
		psr
		jump SPI_Read_Byte		
		set_CS 			/* disbale CS */
	.endm
	.global tx_entry
tx_entry:
	//;The following 3 lines of code is used to avoid multiple initialisations for the LoRa module 
	//; only one-time -lora-initialisations
//	move r0,init_tx
//	ld r0,r0,0
//	jumpr tx_initialized,0,gt
	//; here LoRa initialaisations starts	
	psr 
	jump tx_lora_init
// 	move r0,init_tx
//	move r2,1
// 	st r2,r0,0
	//; After initialiasation LoRa, it skips multiple intialisations 
tx_initialized:
	//;The below code returns if there is any packet available  for the LoRa to Receive 
	psr 
    jump lora_begin	
  //  jump wake_up


	.global tx_lora_init
tx_lora_init:
	//; The below five lines of code will be used to reduce the multiple times LoRa reset for every wake-up 
	//; so here it is reading the LoRa version which is available only after the reset operation
	read_from_reg 0x42
	move r0,r2
	jumpr skip,0x12,eq
			clear_RESET			//*
			move r2, 1			//*			
			psr					//*
			jump waitMs		//*
			set_RESET			//*
			move r2, 11 		//*********** lora version block ******************	
			psr					//*
			jump waitMs		//*
			move r2, 100 		//*			
			psr					//*
			jump waitMs		//*
	jump entry
skip:
	//; The below code is used to set the lora parameters like 
	//; 1) 433 MHZ frequecy
	//; 2) Clearing TX and RX fifo buffer pointers 
	//; 3) setting TX power and RX current for Lora
	write_to_reg 0x01,0x80
	write_to_reg 0x06,0x6C
	write_to_reg 0x07,0x40
	write_to_reg 0x08,0x00
	write_to_reg 0x0E,0x00
	write_to_reg 0x0F,0x00
	read_from_reg 0x0C
	or r2,r2,0x03
	write_to_reg 0x0C,r2
	write_to_reg 0x26,0x04
	write_to_reg 0x09,0x8F
	//; 4) Intilaising the hardware DIO_x pins here  
	write_to_reg 0x40,0x40	
	write_to_reg 0x01,0x81
	//; placing LORa into stand-by mode and returned to where the function is called

	ret

	.global lora_begin
lora_begin:
	write_to_reg 0x1E,0x74
writeloop:
	write_to_reg 0x01,0x81
	write_to_reg 0x1D,0x72
	write_to_reg 0x0D,0x00
	write_to_reg 0x22,0x00
//	write_to_reg 0x00,0x42
	write_to_reg 0x22,0x65

tx_loop:
	move r1,tx_counter
	ld r2,r1,0
	add r2,r2,1
	st r2,r1,0
	add r1,r1,r2
	ld r2,r1,0
	rsh r2,r2,8//; using same buffer for both tx and rx
	push r2
	write_to_reg 0x00,r2
	pop r0	
	jumpr tx_loop,0x00,gt
	//; temperature values
	move r1,temperature
    ld r2,r1,0
	write_to_reg 0x00,r2  
	write_to_reg 0x00,44
	move r1,humidity
    ld r2,r1,0
	write_to_reg 0x00,r2 	
//	move r1,tx_fifo
//	ld r0,r1,32// symbol indication for conditions "A","S","N" 
//	rsh r2,r2,8
//	and r0,r0,0x00ff
//	write_to_reg 0x00,r0
	write_to_reg 0x00,DELIMITER
	write_to_reg 0x00,0
	move r1,tx_counter
	ld r2,r1,0
	add r2,r2,5 // temp + delimit
	write_to_reg 0x22,r2
	move r1,tx_counter
	move r2,0
//	ld r2,r1,0
	and r2,r2,0x0000
	st r2,r1,0
	write_to_reg 0x01,0x83
intloop:
	read_from_reg 0x12
	and r0,r2,0x08
	jumpr intloop,0,eq
	write_to_reg 0x12,0x08
    ret
	halt
	/* Get ULP back to sleep */
