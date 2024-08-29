       
       
       
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
.set HDC1080_ADDR_W,0x80
.set HDC1080_ADDR_R,0x81
.set HDC1080_ON,0x00
.set HDC1080_slave,0x02
.set data1, 0x15
.set data2, 0x00
.bss
  .global HDC1080_dummy
HDC1080_dummy:
   .skip 8
   .global HDC1080_dummy1
HDC1080_dummy1:
   .skip 8
   .global HDC1080_buffer
HDC1080_buffer:
   .skip 180
   .global HDC1080_humidbuffer
HDC1080_humidbuffer:
   .skip 180
   .global init
init:
 .long 0
   .global sensor_count_value
sensor_count_value:
   .long 0
.text
.macro humidconversion
   move r1,HDC1080_dummy1
   ld r0,r1,0
   rsh r0,r0,8
   lsh r2,r0,2
   lsh r0,r2,3
   add r0,r0,r2
   lsh r2,r2,4
   add r0,r0,r2
   rsh r0,r0,8
   st r0,r1,0
.endm
.macro conversion
   move r1,HDC1080_dummy
   ld r0,r1,0
   rsh r0,r0,7
   ld r2,r1,0
   add r2,r2,r0
   lsh r0,r0,2
   add r2,r2,r0
   lsh r0,r0,3
   add r2,r2,r0
   rsh r2,r2,7
   sub r2,r2,160
   st r2,r1,0
.endm
.macro storing_hdc1080
   move r2,sensor_count_value
   move r1,r0
   add r0,r0,1
   st r0,r2,0
   sub r0,r0,1
   move r1,HDC1080_buffer
   add r1,r1,r0
   move r2,HDC1080_dummy
   ld r2,r2,0
   st r2,r1,0
   move r1,HDC1080_humidbuffer
   add r1,r1,r0
   move r2,HDC1080_dummy1
   ld r2,r2,0
   st r2,r1,0
.endm
.global Read_HDC1080
Read_HDC1080:
   move r1, HDC1080_ADDR_R
   push r1
   psr
   jump i2c_start_cond
   ld r2, r3, 4
   psr
   jump i2c_write_byte
   jumpr popfail, 1, ge
   pop r1
   move r2,0
   psr
   jump i2c_read_byte
   push r0
   move r2,0
   psr
   jump i2c_read_byte
   push r0
   move r2,0
   psr
   jump i2c_read_byte
   push r0
   move r2,1
   psr
   jump i2c_read_byte
   push r0
   psr
   jump i2c_stop_cond
   pop r0
   pop r2
   lsh r2,r2,8
   or r2,r2,r0
   move r1,HDC1080_dummy1
   st r2,r1,0
   humidconversion
   pop r0
   pop r2
   lsh r2,r2,8
   or r2,r2,r0
   move r1,HDC1080_dummy
   st r2,r1,0
   conversion
   move r2,0
ret
.global Cmd_Write_HDC1080
Cmd_Write_HDC1080:
   psr
   jump i2c_start_cond
   ld r2, r3, 12
   psr
   jump i2c_write_byte
   jumpr popfail,1,ge
   ld r2, r3, 8
   psr
   jump i2c_write_byte
   jumpr popfail, 1, ge
   psr
   jump i2c_stop_cond
   ret
.global popfail
popfail:
   pop r1
   move r2,1
   ret
.global data_Write_HDC1080
data_Write_HDC1080:
   psr
   jump i2c_start_cond
   ld r2, r3, 20
   psr
   jump i2c_write_byte
   jumpr popfail,1,ge
   ld r2, r3, 16
   psr
   jump i2c_write_byte
   jumpr popfail, 1, ge
   ld r2, r3, 12
   psr
   jump i2c_write_byte
   jumpr popfail,1,ge
   ld r2, r3, 8
   psr
   jump i2c_write_byte
   jumpr popfail, 1, ge
   psr
   jump i2c_stop_cond
    ret
.global Start_HDC1080
Start_HDC1080:
   move r1, HDC1080_ADDR_W
   push r1
   move r1, HDC1080_ON
   push r1
   psr
   jump Cmd_Write_HDC1080
   pop r1
   move r1, HDC1080_slave
   push r1
   move r1,data1
   push r1
   move r1,data2
   push r1
   psr
   jump data_Write_HDC1080
   pop r1
   pop r1
   pop r1
   move r1, HDC1080_ON
   push r1
   psr
jump Cmd_Write_HDC1080
   move r2, 15
   psr
   jump waitMs
   pop r1
   pop r1
   ret
.global Task_HDC1080
Task_HDC1080:
   psr
   jump Start_HDC1080
   psr
   jump Read_HDC1080
   psr
   jump comparison_buffer_manager
   ret
   .global comparison_buffer_manager
comparison_buffer_manager:
   move r1,HDC1080_dummy
   ld r0,r1,0
   ld r2,r1,4
   sub r0,r0,r2
   jumpr retloop2,0,eq
   ld r0,r1,0
   st r0,r1,4
   move r1,temperature
   st r0,r1,0
   move r0,HDC1080_dummy1
   ld r0,r0,0
   move r1,humidity
   st r0,r1,0
   move r2,sensor_count_value
   ld r0,r2,0
   jumpr retloop,45,ge
   storing_hdc1080
   psr
   jump Condition_1_T
   jump entry
   ret
   .global retloop
retloop:
   move r2,sensor_count_value
   move r0,0
   st r0,r2,0
   ret
   .global retloop2
retloop2:
   ret
