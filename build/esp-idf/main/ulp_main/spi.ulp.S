       
       
       
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
 .set bit_mask, 0x80
 .set bit_len, 0x08
 .set SPI_MODE_1, 1
 .set SPI_MASTER, 0
 .set SPI_SLAVE, 1
 .set SPI_MODE_SET, SPI_MODE_1
 .set SPI_TYPE_SET, SPI_MASTER
.bss
 .global spi_mode
spi_mode:
 .long 0
 .text
.macro spi_delay
 nop
.endm
.macro read_MISO
 REG_RD ((((0x3ff48400 + 0x24)) - 0x3ff48000) / 4), ((14 + 15) + (1) - 1), (14 + 15)
.endm
.macro clear_SCLK
 REG_WR ((((0x3ff48400 + 0x8)) - 0x3ff48000) / 4), ((14 + 16) + (1) - 1), (14 + 16), ((1) & 0xff)
.endm
.macro set_SCLK
 REG_WR ((((0x3ff48400 + 0x4)) - 0x3ff48000) / 4), ((14 + 16) + (1) - 1), (14 + 16), ((1) & 0xff)
.endm
.macro clear_MOSI
 REG_WR ((((0x3ff48400 + 0x8)) - 0x3ff48000) / 4), ((14 + 14) + (1) - 1), (14 + 14), ((1) & 0xff)
.endm
.macro set_MOSI
 REG_WR ((((0x3ff48400 + 0x4)) - 0x3ff48000) / 4), ((14 + 14) + (1) - 1), (14 + 14), ((1) & 0xff)
.endm
.macro clear_CS
 REG_WR ((((0x3ff48400 + 0x8)) - 0x3ff48000) / 4), ((14 + 13) + (1) - 1), (14 + 13), ((1) & 0xff)
.endm
.macro set_CS
 REG_WR ((((0x3ff48400 + 0x4)) - 0x3ff48000) / 4), ((14 + 13) + (1) - 1), (14 + 13), ((1) & 0xff)
.endm
 .global SPI_Write_Byte
SPI_Write_Byte:
write_mode_1:
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
 .global SPI_Read_Byte
SPI_Read_Byte:
 set_MOSI
read_mode_1:
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
