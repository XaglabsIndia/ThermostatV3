/*

This is the ULP program used to LORa_receive whenever there is incoming packet length from transmitter 
editor: Krishna.TL 

*/

#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "stack.S"
#define esp32 // please comment if you are using es32s3
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


//; change the payload length here
#define payload_len 12 
#define DEVID 64
//#define conditions
#define rx_timeout
#define DEVIDm (DEVID+1)
#define payloadm (payload_len+1)
	/* Define variables, which go into .bss section (zero-initialized data) */
	.bss

	.global rx_count			
rx_count:
	.long 0
	.global rx_fifo
rx_fifo:
	.skip 80
	.global rx_fifo1
rx_fifo1:
	.skip 4
	.global reg_addr	 	
reg_addr:
	.long 0
	.global pay_len			
pay_len:
	.long 0				
	.global pay_lend			
pay_lend:
	.long 0	
	.global temp
temp:
	.long 0
	.global temp1// FOR RX TIMEOUT
temp1:
	.long 0
	.global rx_c
rx_c:
	.long 0
	.global init_rx
init_rx:
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
	.macro storing
		move r1,tx_fifo
		move r2,rx_count
		ld r0,r2,0
		add r1,r1,r0
		add r0,r0,1
		st r0,r2,0
		move r0,temp 		//temperature value
		ld r0,r0,0
		ld r2,r1,0   //; these lines are used for common buffer space
		or r0,r2,r0	 //; these lines are used for common buffer space
		st r0,r1,0
	.endm


	.global rx_entry
rx_entry:

	//;The following 3 lines of code is used to avoid multiple initialisations for the LoRa module 
	//; only one-time -lora-initialisations
	move r0,init_rx
	ld r0,r0,0
	jumpr rx_initialized,0,gt
	//; here LoRa initialaisations starts	
	psr 
	jump rx_lora_init
	move r0,init_rx
	move r2,1
	st r2,r0,0
	//; After initialiasation LoRa, it skips multiple intialisations 
rx_initialized:
	//;The below code returns if there is any packet available  for the LoRa to Receive 	
	psr 
	jump receive_lora
	ret					

	//; This is used to reset the LoRa on power-up   

	.global rx_lora_init
rx_lora_init:
	//; The below five lines of code will be used to reduce the multiple times LoRa reset for every wake-up 
	//; so here it is reading the LoRa version which is available only after the reset operation
	read_from_reg 0x42
	move r0,r2
	jumpr skip,0x12,eq	
	clear_RESET			//*
	move r2, 1			//*			
	psr					//*
	jump waitMs			//*
	set_RESET			//*
	move r2, 11 		//*********** lora version block ******************	
	psr					//*
	jump waitMs			//*
	move r2, 100 		//*			
	psr					//*
	jump waitMs			//*
	halt
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

	//; This is used to hold CPU cycles delay for a period defined by r2 .For ex: if r2=1 then it waits for (1ms) delay 

	//; The following function is used to check,Is there any packet available for the  LoRa to receive by continuous polling the IRQ
	//; So if there is any IRQ then it jumps into Lora read loop. 
	.global lora_parse
lora_parse:
#ifdef rx_timeout
	move r1,temp1
	ld r2,r1,0
	and r2,r2,0x0000
	add r2,r2,1
	st r2,r1,0 	
#endif
	nop	
lora_parse1:
	//; for rx timeout function
#ifdef rx_timeout
	move r1,temp1
	ld r0,r1,0
	add r0,r0,1
	st r0,r1,0 
	jumpr retloop1,1024,ge
#endif
	//; 1) The following three lines is used to set the LoRa into explicit mode 
	read_from_reg 0x1D	
	and r2,r2,0xfe		
	write_to_reg 0x1D,r2 
	//; 2) Reading the IRQ continuously for the software interrupt 	
	read_from_reg 0x12
	write_to_reg 0x12,r2
	move r0,r2

	//; 3) Waiting for the valid header IRQ
	jumpr lora_parse1,0x40,le
//	move r2, 10                    
//	psr                                
//  jump waitMs
	//; 4) After the IRQ it clears the counter which is used to store the Lora payload length
	move r2,rx_count
	and r0,r0,0x0000//move r0,0
	st r0,r2,0
	//; 5) Jumping to reading function which reads the incoming payload
	psr
	jump lora_read_PL_len
	//; 6) Returninig back to where the Lora parse function called. 
retloop1:
	move r1,temp1
	move r0,0
	st r0,r1,0 
   ret
	
	//; This function is used to place the LoRa into the Receiving mode 
	//; After Confirming the LoRa placed into the receiving mode then it jumps to Lora_parse function  
	.global receive_lora
receive_lora:
	//; 1) Placing the LoRa into the receiving mode
	write_to_reg 0x01,0x85
	read_from_reg 0x01		
	move r0,r2
	//; 2) Confirming the LoRa placed into the receive mode, if not confirmed it again jumps to step 1.
	jumpr receive_lora,0x85,lt	
	//; 3) Then jumping back to the LoRa parse function 
	psr 
	jump lora_parse	// skipping recieve function for fixed interval
	//; Returninig back to where the receive lora function called.
	ret

	//; The following function is used for reading the incoming payload length from LoRa.
	.global lora_read_PL_len
lora_read_PL_len:
	//; 1) Reading the payload length from the incoming packets.
	read_from_reg 0x13
	move r1,pay_len
	move r0,r2
	//; 2) IF the payload length is equal to zero then it jumps back to reading the Payload length else it stores the length in two consecutive memory places 
	jumpr lora_read_PL_len,payload_len,lt
	jumpr lora_read_PL_len,payloadm,ge			//* this line gives the accurate payload filter it provides range
	st r0,r1,0
	st r0,r1,4
	//; 3) If there is non zero payload, then the follwing line will place the LoRa into idle mode 
	write_to_reg 0x01,0x81
	//; 4) Read the current FIFO pointer and placing RX FIFO pointer to current FIFO pointer position
	read_from_reg 0x10
	write_to_reg 0x0D,r2
	//; 5) Jumping back to LoRa read loop to read the 
//	read_from_reg 0x00
//	move r0,r2
//	jumpr end1,DEVID,lt
//	jumpr end1,DEVIDm,ge
//	storing
	psr 
	jump read_loop
#ifdef conditions
	psr 
	jump AddressMatch
#endif
//;	if address matches then it is jumping back to condition checks
	jump Condition_Check_T
//; 6) Returninig back to where the function lora_read_PL_len called
	ret




	//; The following function is used for reading the incoming payload from LoRa (byte by byte).	
	.global read_loop
read_loop:
	//; 1) The following  code will read the incoming packets based on the payload length.
	read_from_reg 0x00
//	push r2//£
//	read_from_reg 0x00//£
	move r1,pay_len
	ld r0,r1,0	
	sub r0,r0,1//£
	st r0,r1,0
	jumpr end1,0,eq//£
	//; 2) After reading a byte from the LoRa  then it stroes each byte into separate buffer so whenever the soc wakes-up we are able to read 
	move r1,temp
	st r2,r1,0
	storing
	//; 3) Jumping back to step 1 untill last packet receives.
	jump read_loop
end1:
	move r1,temp
	st r2,r1,0

	storing

	//; 4) Returning Back where the read_loop called
	ret


	//; This function is used to store the incoming byte into seoarate buffer addres so after soc wake up we are able to see the data 

	//; end of the ulp program

// This is used to convert the integer to string 







