       
       
       
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
 .global i2c_started
i2c_started:
 .long 0
 .global i2c_didInit
i2c_didInit:
 .long 0
 .global i2c_started1
i2c_started1:
 .long 0
 .global i2c_didInit1
i2c_didInit1:
 .long 0
 .global i2c_started2
i2c_started2:
 .long 0
 .global i2c_didInit2
i2c_didInit2:
 .long 0
.text
.global i2c_start_cond2
.global i2c_stop_cond2
.global i2c_start_cond1
.global i2c_stop_cond1
.global i2c_start_cond
.global i2c_stop_cond
.global i2c_write_bit
.global i2c_read_bit
.global i2c_write_byte
.global i2c_read_byte
.macro I2C_delay
wait 50
.endm
.macro read_SCL
 REG_RD ((((0x3ff48400 + 0x24)) - 0x3ff48000) / 4), ((14 + 9) + (1) - 1), (14 + 9)
.endm
.macro read_SDA
 REG_RD ((((0x3ff48400 + 0x24)) - 0x3ff48000) / 4), ((14 + 8) + (1) - 1), (14 + 8)
.endm
.macro set_SCL
 REG_WR ((((0x3ff48400 + 0x14)) - 0x3ff48000) / 4), ((14 + 9) + (1) - 1), (14 + 9), ((1) & 0xff)
.endm
.macro clear_SCL
 REG_WR ((((0x3ff48400 + 0x10)) - 0x3ff48000) / 4), ((14 + 9) + (1) - 1), (14 + 9), ((1) & 0xff)
.endm
.macro set_SDA
 REG_WR ((((0x3ff48400 + 0x14)) - 0x3ff48000) / 4), ((14 + 8) + (1) - 1), (14 + 8), ((1) & 0xff)
.endm
.macro clear_SDA
 REG_WR ((((0x3ff48400 + 0x10)) - 0x3ff48000) / 4), ((14 + 8) + (1) - 1), (14 + 8), ((1) & 0xff)
.endm
i2c_start_cond:
 move r1,i2c_didInit
 ld r0,r1,0
 jumpr didInit,1,ge
 add r0,r0,1
 st r0,r1,0
didInit:
 move r2,i2c_started
 ld r0,r2,0
 jumpr not_started,1,lt
 set_SDA
 I2C_delay
 set_SCL
 stage_rst
clock_stretch:
 stage_inc 1
 jumps TIMEOUT,254,ge
 read_SCL
 jumpr clock_stretch,1,lt
TIMEOUT:
 I2C_delay
not_started:
 clear_SDA
 I2C_delay
 clear_SCL
 move r0,1
 st r0,r2,0
 ret
i2c_stop_cond:
 clear_SDA
 I2C_delay
 set_SCL
 stage_rst
clock_stretch_stop:
 stage_inc 1
 jumps TIMEOUT1,254,ge
 read_SCL
 jumpr clock_stretch_stop,1,lt
TIMEOUT1:
 I2C_delay
 set_SDA
 I2C_delay
  move r2,i2c_started
  move r0,0
  st r0,r2,0
  ret
i2c_write_bit:
 jumpr bit0,1,lt
 set_SDA
 jump bit1
bit0:
 clear_SDA
bit1:
 I2C_delay
 set_SCL
 I2C_delay
clock_stretch_write:
 read_SCL
 jumpr clock_stretch_write,1,lt
TIMEOUT2:
  clear_SCL
  ret
i2c_read_bit:
 set_SDA
 I2C_delay
  set_SCL
clock_stretch_read:
 read_SCL
 jumpr clock_stretch_read,1,lt
 I2C_delay
 read_SDA
 clear_SCL
 ret
i2c_write_byte:
 stage_rst
next_bit:
 and r0,r2,0x80
 psr
 jump i2c_write_bit
 lsh r2,r2,1
 stage_inc 1
 jumps next_bit,8,lt
 psr
 jump i2c_read_bit
 ret
    .global i2c_read_byte
i2c_read_byte:
 push r2
 move r2,0
 stage_rst
next_bit_read:
 psr
 jump i2c_read_bit
 lsh r2,r2,1
 or r2,r2,r0
 stage_inc 1
 jumps next_bit_read,8,lt
 pop r0
 psr
 jump i2c_write_bit
 move r0,r2
 ret
