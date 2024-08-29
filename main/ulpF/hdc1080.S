#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "stack.S"
#define I2C
//; change the buffer length here
#define BUFFER 45

.set HDC1080_ADDR_W,0x80  // 0x40<<1|0
.set HDC1080_ADDR_R,0x81  //0x40<<1|1
.set HDC1080_ON,0x00   //0xE1
.set HDC1080_slave,0x02   //0xAC
.set data1, 0x15  //0x33
.set data2, 0x00  //0x00

.bss
  .global HDC1080_dummy                          // it stores the present sensor value of HDC1080 sensor
HDC1080_dummy:
   .skip 8 
   .global HDC1080_dummy1                          // it stores the present sensor value of HDC1080 sensor
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
//; This is used to convert the raw humidity value into the real time humidity value 
.macro humidconversion  
   move r1,HDC1080_dummy1
   ld r0,r1,0
   rsh r0,r0,8
   //ld r2,r1,0
   //add r2,r2,r0
   lsh r2,r0,2
   //add r2,r2,r0
   lsh r0,r2,3
   add r0,r0,r2
   lsh r2,r2,4
   add r0,r0,r2
   rsh r0,r0,8
   st r0,r1,0
.endm
//; This is used to convert the raw temp value into the real time temp value 
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
   rsh r2,r2,7           //    rsh r2,r2,9
   sub r2,r2,160           //   sub r2,r2,40// these lines for resolution
   st r2,r1,0
.endm

// end of conversion label
.macro storing_hdc1080
   move r2,sensor_count_value
   move r1,r0                                 // copying count value into the r2 register
   add r0,r0,1                               // incrementing the count value
   st r0,r2,0                                //storing it in sensor count value variable.
   sub r0,r0,1
   move r1,HDC1080_buffer                     //Load the starting address pointer of buffer
   add r1,r1,r0                              // incrementing the buffer size pointer by using the r2 value 
   move r2,HDC1080_dummy
   ld r2,r2,0                                
   st r2,r1,0                               //Store the present sensor value into the current buffer pointer
//   move r1,temperature
//   st r2,r1,0
 // HUMID STORING  
   move r1,HDC1080_humidbuffer
   add r1,r1,r0                              // incrementing the buffer size pointer by using the r2 value 
   move r2,HDC1080_dummy1
   ld r2,r2,0                                
   st r2,r1,0
.endm

.global Read_HDC1080 //BEGINNING OF SUBROTUINE.
Read_HDC1080:
   move r1, HDC1080_ADDR_R //MOVES THE ADDRESSS OF HDC1080 SENSOR FOR READ OPERATIONS INTO REGISTER R1 AND PUSHES IT ONTO THE STACK.
   push r1
   psr 
   jump i2c_start_cond         // i2c Start
   ld r2, r3, 4                 // Address+Read
   psr
   jump i2c_write_byte
   jumpr popfail, 1, ge
   pop r1
   move r2,0                    // byte 1
   psr
   jump i2c_read_byte // tmp msb byte
   push r0
   move r2,0 // byte 2
   psr
   jump i2c_read_byte  // tmp lsb byte
   push r0
   move r2,0 // byte 3
   psr
   jump i2c_read_byte // hum msb byte
   push r0
   move r2,1 // byte 4
   psr
   jump i2c_read_byte
   push r0
   psr
   jump i2c_stop_cond 
   // humid
   pop r0// lsb 
   pop r2// msb
   lsh r2,r2,8
   or r2,r2,r0
   move r1,HDC1080_dummy1
   st r2,r1,0                 // humidity   value 
   humidconversion
   // temp 
   pop r0// lsb 
   pop r2// msb
   lsh r2,r2,8
   or r2,r2,r0
   move r1,HDC1080_dummy
   st r2,r1,0                 // temperature value
   conversion
   move r2,0 // OK
ret



.global Cmd_Write_HDC1080
Cmd_Write_HDC1080:
   psr 
   jump i2c_start_cond           // i2c Start    
   ld r2, r3, 12                 // Address+Write
   psr
   jump i2c_write_byte
   jumpr popfail,1,ge
   ld r2, r3, 8                  // Command
   psr
   jump i2c_write_byte
   jumpr popfail, 1, ge
   psr
   jump i2c_stop_cond            // i2c Stop
   ret
.global popfail
popfail:
   pop r1                        // pop caller return address
   move r2,1
   ret

.global data_Write_HDC1080
data_Write_HDC1080:
   psr 
   jump i2c_start_cond           // i2c Start
   ld r2, r3, 20                 // Address+Write
   psr
   jump i2c_write_byte// address
   jumpr popfail,1,ge
   ld r2, r3, 16                 
   psr
   jump i2c_write_byte// slave address
   jumpr popfail, 1, ge
   ld r2, r3, 12                
   psr
   jump i2c_write_byte// data byte 1
   jumpr popfail,1,ge
   ld r2, r3, 8                  
   psr
   jump i2c_write_byte// data byte 2
   jumpr popfail, 1, ge

   psr
   jump i2c_stop_cond            // i2c Stop
    ret

//;1) sensor i2c intialisations and configurations 
.global Start_HDC1080
Start_HDC1080:
   move r1, HDC1080_ADDR_W
   push r1
   move r1, HDC1080_ON
   push r1 
   psr 
   jump Cmd_Write_HDC1080
 // power on
   pop r1                        // removing ON command
   move r1, HDC1080_slave    
   push r1                        // adding slave_address
   move r1,data1
   push r1
   move r1,data2
   push r1
   psr 
   jump data_Write_HDC1080         // once H
 //  pop r1                       // device address_r
  // pop r1                         // slave address
   pop r1                           // data1
   pop r1                           // data2
   pop r1                           // slave_address 
   move r1, HDC1080_ON    
   push r1
   psr                               //..........
jump Cmd_Write_HDC1080              //...........
   move r2, 15                      //.........
   psr                                //.........// delay of 10ms conversion time.
   jump waitMs                       //..............
   pop r1                            // aht_25 on..........
   pop r1                           // device address_r//............
 //  pop r1                         // slave address//.........
   ret


//; The sensor task starts here 
.global Task_HDC1080
Task_HDC1080:   
//;1) sensor i2c intialisations and configurations 
   psr
   jump Start_HDC1080
//;2) reading the hdc sensor temp and humidity values
   psr
   jump Read_HDC1080
//;3) comparision and storing 
   psr
   jump comparison_buffer_manager
   ret

//;3) This is used to compare the present temp with  previous temp 
   .global comparison_buffer_manager
comparison_buffer_manager:
   move r1,HDC1080_dummy
   ld r0,r1,0                                   // r0 has present value
   ld r2,r1,4                                  // r2 has previous value
   sub r0,r0,r2
   jumpr retloop2,0,eq
   ld r0,r1,0
   st r0,r1,4
   move r1,temperature
   st r0,r1,0
   move r0,HDC1080_dummy1
   ld r0,r0,0
   move r1,humidity              // storing humidity here 
   st r0,r1,0
#ifdef I2C
   move r2,sensor_count_value               // sensor_count_value stores buffer value
   ld r0,r2,0 
   jumpr retloop,BUFFER,ge 
   storing_hdc1080
   psr
   jump Condition_1_T
   jump entry
#endif 
#ifndef I2C
   move r2,sensor_count_value               // sensor_count_value stores buffer value
   ld r0,r2,0 
   jumpr retloop,BUFFER,ge 
   storing_hdc1080
#endif
   ret

//; This is used to store the hdc sensor value in the buffer 

   .global retloop
retloop:
   move r2,sensor_count_value               // sensor_count_value stores buffer value
   move r0,0
   st r0,r2,0 
   ret

   
   .global retloop2
retloop2:
   ret