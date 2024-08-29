/*

This is the ULP program used to LORa_receive whenever there is incoming packet length from transmitter 
editor: Krishna.TL 

*/

#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "stack.S"
#define condition1 'A'
#define condition2_0 '3'
#define condition2_1 '1'
#define condition2_2 '2'
#define condition3 'N'
#define condition4 'S'
#define CONDITION_1_RETRY 3 // change the retry numbers here  
.bss
.global tx_retry_cond_1_counter
tx_retry_cond_1_counter:
.long 0
.global AddressMatched
AddressMatched:
.long 0

.text
.global Condition_Check_T
Condition_Check_T:
//; Decoding the incoming message from HUB 
move r1,tx_fifo
ld r0,r1,32// symbol indication for conditions "A","S","N" 
and r0,r0,0x00ff
jumpr Condition_2_T,condition2_0,eq
jumpr Condition_2_T,condition2_1,eq
jumpr Condition_2_T,condition2_2,eq
jumpr Condition_3_T,condition3,eq
jumpr Condition_4_T,condition4,eq
jump entry
 


 .global Condition_1_T
Condition_1_T:
wait 10
psr 
jump tx_entry
//; initilaising the tx retry counter as zero
move r1,tx_retry_cond_1_counter
move r2,0
and r2,r2,0
st r2,r1,0
//; wait for 200 ms for tx to rx lora conversion
Condition_1_retry:
move r1,tx_fifo
ld r0,r1,32// symbol indication for conditions "A","S","N" 
and r0,r0,0xff00
st r0,r1,32

move r2, 200                     
psr                                
jump waitMs
//ret  
psr 
jump rx_lora_init
psr 
jump receive_lora 
psr 
jump lora_parse
//psr
//jump AddressMatch
//; check for the symbol from received message
move r1,tx_fifo
ld r0,r1,32// symbol indication for conditions "A","S","N" 
and r0,r0,0x00ff
jumpr Condition_1_retry_stop,condition1,eq
//; ...............
//; transmitting the message again
psr 
jump tx_entry
//; ...............
move r1,tx_retry_cond_1_counter
ld r0,r1,0
add r0,r0,1
st r0,r1,0
jumpr Condition_1_retry,CONDITION_1_RETRY,lt
Condition_1_retry_stop:
move r1,tx_retry_cond_1_counter
move r2,0
and r2,r2,0
st r2,r1,0

ret

 .global Condition_2_T
Condition_2_T:
jump wake_up
 
 .global Condition_3_T
Condition_3_T:
jump wake_up
 
 .global Condition_4_T
Condition_4_T:
   psr
   jump Condition_1_T
   jump entry

 .global Condition_Check_F
Condition_Check_F:
	jump wake_up

.global AddressMismatch
AddressMismatch:
	jump entry


.global AddressMatch
//PlaceLoraIntoRx:
	//; 1) Placing the LoRa into the receiving mode
//	write_to_reg 0x01,0x85
//	read_from_reg 0x01		
//	move r0,r2
	//; 2) Confirming the LoRa placed into the receive mode, if not confirmed it again jumps to step 1.
//	jumpr PlaceLoraIntoRx,0x85,lt
AddressMatch:
//; The below lines are used for address matching
	move r1,tx_fifo
	ld r2,r1,20
	ld r0,r1,4
	rsh r2,r2,8
	and r0,r0,0x00ff
	sub r0,r0,r2
	jumpr ndnibble,0,eq
	jump AddressMismatch		// if address fails jump to entry 
ndnibble:
	move r1,tx_fifo
	ld r2,r1,24
	ld r0,r1,8
	rsh r2,r2,8
	and r0,r0,0x00ff
	sub r0,r0,r2
	jumpr rdnibble,0,eq
	jump AddressMismatch		// if address fails jump to entry  
rdnibble:
	move r1,tx_fifo
	ld r2,r1,28
	ld r0,r1,12
	rsh r2,r2,8
	and r0,r0,0x00ff
	sub r0,r0,r2
	jumpr wakeup,0,eq
	jump AddressMismatch		// if address fails jump to entry  
wakeup:
	ret