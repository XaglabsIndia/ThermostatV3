       
       
       
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
 .set MS5611_ADDR, 0x77
 .set CMD_RESET, 0x42
 .bss
 .global tx_counter
tx_counter:
 .long 0
 .global tx_fifo
tx_fifo:
 .skip 420
 .global rx_counter
rx_counter:
 .long 0
 .global FIFOEnd
FIFOEnd:
 .long 0
 .global counter
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
 .global tx_entry
tx_entry:
 psr
 jump tx_lora_init
tx_initialized:
 psr
    jump lora_begin
 .global tx_lora_init
tx_lora_init:
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
 jump entry
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
 .global lora_begin
lora_begin:
 write_to_reg 0x1E,0x74
writeloop:
 write_to_reg 0x01,0x81
 write_to_reg 0x1D,0x72
 write_to_reg 0x0D,0x00
 write_to_reg 0x22,0x00
 write_to_reg 0x22,0x65
tx_loop:
 move r1,tx_counter
 ld r2,r1,0
 add r2,r2,1
 st r2,r1,0
 add r1,r1,r2
 ld r2,r1,0
 rsh r2,r2,8
 push r2
 write_to_reg 0x00,r2
 pop r0
 jumpr tx_loop,0x00,gt
 move r1,temperature
    ld r2,r1,0
 write_to_reg 0x00,r2
 write_to_reg 0x00,44
 move r1,humidity
    ld r2,r1,0
 write_to_reg 0x00,r2
 write_to_reg 0x00,'@'
 write_to_reg 0x00,0
 move r1,tx_counter
 ld r2,r1,0
 add r2,r2,5
 write_to_reg 0x22,r2
 move r1,tx_counter
 move r2,0
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
