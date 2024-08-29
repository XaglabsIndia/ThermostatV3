       
       
       
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
.set TEMP,0x80
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
 move r3, stackEnd
    psr
    jump rx_entry
 psr
 jump lora_parse
 move r2, 200
  psr
    jump waitMs
 psr
 jump Task_HDC1080
 jump entry
exitloop:
 jump wake_up
 .global waitMs
waitMs:
 wait 8000
 sub r2,r2,1
 jump doneWaitMs,eq
 jump waitMs
doneWaitMs:
 ret
 .global exit
exit:
 halt
 .global wake_up
wake_up:
 REG_RD ((((0x3ff48000 + 0xc0)) - 0x3ff48000) / 4), ((19) + (1) - 1), (19)
 and r0, r0, 1
 jump exit, eq
 wake
 REG_WR ((((0x3ff48000 + 0x18)) - 0x3ff48000) / 4), ((24) + (((((((0x1) >> (0)) & 1) + (((0x1) >> (0 + 1)) & 1)) + ((((0x1) >> (0 + 2)) & 1) + (((0x1) >> (0 + 2 + 1)) & 1))) + (((((0x1) >> (0 + 4)) & 1) + (((0x1) >> (0 + 4 + 1)) & 1)) + ((((0x1) >> (0 + 4 + 2)) & 1) + (((0x1) >> (0 + 4 + 2 + 1)) & 1))))) - 1), (24), ((((0) & 0x1)) & 0xff)
 halt
