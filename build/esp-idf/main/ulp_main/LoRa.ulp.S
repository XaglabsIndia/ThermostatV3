       
       
       
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
 .global temp1
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
  REG_WR ((((0x3ff48400 + 0x8)) - 0x3ff48000) / 4), ((14 + 14) + (1) - 1), (14 + 14), ((1) & 0xff)
 .endm
 .macro clear_SCLK
  REG_WR ((((0x3ff48400 + 0x8)) - 0x3ff48000) / 4), ((14 + 16) + (1) - 1), (14 + 16), ((1) & 0xff)
 .endm
  .macro clear_RESET
  REG_WR ((((0x3ff48400 + 0x8)) - 0x3ff48000) / 4), ((14 + 07) + (1) - 1), (14 + 07), ((1) & 0xff)
 .endm
 .macro set_RESET
  REG_WR ((((0x3ff48400 + 0x4)) - 0x3ff48000) / 4), ((14 + 07) + (1) - 1), (14 + 07), ((1) & 0xff)
 .endm
 .macro clear_CS
  REG_WR ((((0x3ff48400 + 0x8)) - 0x3ff48000) / 4), ((14 + 13) + (1) - 1), (14 + 13), ((1) & 0xff)
 .endm
 .macro set_CS
  REG_WR ((((0x3ff48400 + 0x4)) - 0x3ff48000) / 4), ((14 + 13) + (1) - 1), (14 + 13), ((1) & 0xff)
 .endm
 .macro spi_init
  set_CS
  clear_MOSI
  clear_SCLK
  clear_CS
 .endm
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
  set_CS
 .endm
 .macro read_from_reg reg
  spi_init
  move r2,\reg
  psr
  jump SPI_Write_Byte
  psr
  jump SPI_Read_Byte
  set_CS
 .endm
 .macro storing
  move r1,tx_fifo
  move r2,rx_count
  ld r0,r2,0
  add r1,r1,r0
  add r0,r0,1
  st r0,r2,0
  move r0,temp
  ld r0,r0,0
  ld r2,r1,0
  or r0,r2,r0
  st r0,r1,0
 .endm
 .global rx_entry
rx_entry:
 move r0,init_rx
 ld r0,r0,0
 jumpr rx_initialized,0,gt
 psr
 jump rx_lora_init
 move r0,init_rx
 move r2,1
 st r2,r0,0
rx_initialized:
 psr
 jump receive_lora
 ret
 .global rx_lora_init
rx_lora_init:
 read_from_reg 0x42
 move r0,r2
 jumpr skip,0x12,eq
 clear_RESET
 move r2, 1
 psr
 jump waitMs
 set_RESET
 move r2, 11
 psr
 jump waitMs
 move r2, 100
 psr
 jump waitMs
 halt
skip:
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
 write_to_reg 0x40,0x40
 write_to_reg 0x01,0x81
 ret
 .global lora_parse
lora_parse:
 move r1,temp1
 ld r2,r1,0
 and r2,r2,0x0000
 add r2,r2,1
 st r2,r1,0
 nop
lora_parse1:
 move r1,temp1
 ld r0,r1,0
 add r0,r0,1
 st r0,r1,0
 jumpr retloop1,1024,ge
 read_from_reg 0x1D
 and r2,r2,0xfe
 write_to_reg 0x1D,r2
 read_from_reg 0x12
 write_to_reg 0x12,r2
 move r0,r2
 jumpr lora_parse1,0x40,le
 move r2,rx_count
 and r0,r0,0x0000
 st r0,r2,0
 psr
 jump lora_read_PL_len
retloop1:
 move r1,temp1
 move r0,0
 st r0,r1,0
   ret
 .global receive_lora
receive_lora:
 write_to_reg 0x01,0x85
 read_from_reg 0x01
 move r0,r2
 jumpr receive_lora,0x85,lt
 psr
 jump lora_parse
 ret
 .global lora_read_PL_len
lora_read_PL_len:
 read_from_reg 0x13
 move r1,pay_len
 move r0,r2
 jumpr lora_read_PL_len,12,lt
 jumpr lora_read_PL_len,(12 +1),ge
 st r0,r1,0
 st r0,r1,4
 write_to_reg 0x01,0x81
 read_from_reg 0x10
 write_to_reg 0x0D,r2
 psr
 jump read_loop
 jump Condition_Check_T
 ret
 .global read_loop
read_loop:
 read_from_reg 0x00
 move r1,pay_len
 ld r0,r1,0
 sub r0,r0,1
 st r0,r1,0
 jumpr end1,0,eq
 move r1,temp
 st r2,r1,0
 storing
 jump read_loop
end1:
 move r1,temp
 st r2,r1,0
 storing
 ret
