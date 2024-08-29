
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "stack.S"
#define rx_timeout
.set TEMP,0x80 
	/* Define variables, which go into .bss section (zero-initialized data) */
	.bss
	.global stack
stack:
	.skip 140
	.global stackEnd
stackEnd:
	.long 0
	
	.global temperature
temperature:
	.long 0
	
	.global humidity
humidity:
	.long 0
    .text
    .global entry
entry:
    //; The Below is used as stack pointer for all the operations in the ULP
	//; note: Don't use the r3 register for any other purpose
	move r3, stackEnd

	//;1) lora receive function
    psr 
    jump rx_entry
//	jump wake_up
#ifdef rx_timeout
	psr 
	jump lora_parse
#endif
//	jump wake_up

	//;2) hdc1080 sensor function
	move r2, 200                     
 	psr                                
    jump waitMs   

	psr 
	jump Task_HDC1080
	jump entry	
//	jump wake_up
//    halt

exitloop:
	jump wake_up
	
//; This is used to hold CPU cycles delay for a period defined by r2 .For ex: if r2=1 then it waits for (1ms) delay 
	
	.global waitMs
waitMs:
	wait 8000
	sub r2,r2,1 				/* Wait for r2 milliseconds */
	jump doneWaitMs,eq
	jump waitMs
doneWaitMs:
	ret
	
	.global exit
exit:
	halt

	//; waking up the soc and closing up the ulp program  
	.global wake_up
wake_up:
	READ_RTC_REG(RTC_CNTL_DIAG0_REG, 19, 1)
	and r0, r0, 1
	
	jump exit, eq
	wake
	WRITE_RTC_FIELD(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN, 0)
	halt

/*	
	.global int_to_str
int_to_str:
	move r1,temperature 
	move r2,tx_fifo	
	ld r0,r1,0
1stnibble:
	jumpr 1stnibble100,100,lt
	sub r0,r0,100
	jumpr 1stnibble200,100,lt
	sub r0,r0,100
	move r0,50
	st r0,r2,44
	jump 2ndnibble
1stnibble200:
	move r0,49
	st r0,r2,44
	jump 2ndnibble
1stnibble100:
	move r0,48
	st r0,r2,44	

	
2ndnibble:

	jumpr 3rdnibble,10,lt
	sub r0,r0,10
	jumpr 2ndnibble900,10,lt
	sub r0,r0,10
	move r0,50
	st r0,r2,48
	jump 3rdnibble
2ndnibble200:
	move r0,49
	st r0,r2,48
	jump 3rdnibble
2ndnibble100:
	move r0,48
	st r0,r2,48	



3rdnibble:
	add r0,r0,48
	st r0,r2,52	
	ret
*/