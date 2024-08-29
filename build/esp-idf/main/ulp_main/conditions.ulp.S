       
       
       
.macro push rx
 st \rx,r3,0
 sub r3,r3,1
.endm
.macro pop rx
 add r3,r3,1
 ld \rx,r3,0
.endm
.macro psr
 .set addr,(.+16)
 move r1,addr
 push r1
.endm
.macro ret
 pop r1
 jump r1
.endm
.macro clear param
 move r1, \param
 move r0, 0
 st r0, r1, 0
.endm
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
move r1,tx_fifo
ld r0,r1,32
and r0,r0,0x00ff
jumpr Condition_2_T,'3',eq
jumpr Condition_2_T,'1',eq
jumpr Condition_2_T,'2',eq
jumpr Condition_3_T,'N',eq
jumpr Condition_4_T,'S',eq
jump entry
 .global Condition_1_T
Condition_1_T:
wait 10
psr
jump tx_entry
move r1,tx_retry_cond_1_counter
move r2,0
and r2,r2,0
st r2,r1,0
Condition_1_retry:
move r1,tx_fifo
ld r0,r1,32
and r0,r0,0xff00
st r0,r1,32
move r2, 200
psr
jump waitMs
psr
jump rx_lora_init
psr
jump receive_lora
psr
jump lora_parse
move r1,tx_fifo
ld r0,r1,32
and r0,r0,0x00ff
jumpr Condition_1_retry_stop,'A',eq
psr
jump tx_entry
move r1,tx_retry_cond_1_counter
ld r0,r1,0
add r0,r0,1
st r0,r1,0
jumpr Condition_1_retry,3,lt
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
AddressMatch:
 move r1,tx_fifo
 ld r2,r1,20
 ld r0,r1,4
 rsh r2,r2,8
 and r0,r0,0x00ff
 sub r0,r0,r2
 jumpr ndnibble,0,eq
 jump AddressMismatch
ndnibble:
 move r1,tx_fifo
 ld r2,r1,24
 ld r0,r1,8
 rsh r2,r2,8
 and r0,r0,0x00ff
 sub r0,r0,r2
 jumpr rdnibble,0,eq
 jump AddressMismatch
rdnibble:
 move r1,tx_fifo
 ld r2,r1,28
 ld r0,r1,12
 rsh r2,r2,8
 and r0,r0,0x00ff
 sub r0,r0,r2
 jumpr wakeup,0,eq
 jump AddressMismatch
wakeup:
 ret
