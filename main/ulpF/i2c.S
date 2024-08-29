/* ULP assembly files are passed through C preprocessor first, so include directives
   and C macros may be used in these files 
 */

#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "stack.S"
#define esp32
#ifdef esp32
//; change RTC pins here based on the GPIO 's used  
//; Here the RTC GPIO's and GPIO's are different so make sure RTC pins should match regarding their gpio pin used 
//;  THese are GPIO pins					```|04|25|26|27|32|33|34|35|36|37|38|39|15|13|12|14|```
//;   and their respective RTC pins  		```|10|06|07|17|09|08|04|05|00|01|02|03|13|14|15|16|```
#define SDA 8 // 33 gpio
#define SCL 9 // 32 gpio
#endif

#ifndef esp32
#define SDA 9 // 9 gpio
#define SCL 10 // 10 gpio
#endif

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
	READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + SCL, 1) 
.endm

.macro read_SDA 
	READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + SDA, 1) 
.endm

.macro set_SCL 
	WRITE_RTC_REG(RTC_GPIO_ENABLE_W1TC_REG, RTC_GPIO_ENABLE_W1TC_S + SCL, 1, 1)
.endm

.macro clear_SCL 
	WRITE_RTC_REG(RTC_GPIO_ENABLE_W1TS_REG, RTC_GPIO_ENABLE_W1TS_S + SCL, 1, 1)
.endm

.macro set_SDA 
	WRITE_RTC_REG(RTC_GPIO_ENABLE_W1TC_REG, RTC_GPIO_ENABLE_W1TC_S + SDA, 1, 1)
.endm

.macro clear_SDA 
	WRITE_RTC_REG(RTC_GPIO_ENABLE_W1TS_REG, RTC_GPIO_ENABLE_W1TS_S + SDA, 1, 1)
.endm


//; This is used to intiilaise the i2c protocol
i2c_start_cond:
	move r1,i2c_didInit
	ld r0,r1,0
	jumpr didInit,1,ge//jumpr didInit,1,ge
	add r0,r0,1//move r0,1
	st r0,r1,0
didInit:
	move r2,i2c_started
	ld r0,r2,0
	jumpr not_started,1,lt 	// if started, do a restart condition
	set_SDA					// set SDA to 1
	I2C_delay
	set_SCL
	stage_rst
clock_stretch: 				// TODO: Add timeout
	stage_inc 1
	jumps TIMEOUT,254,ge
	read_SCL
	jumpr clock_stretch,1,lt
TIMEOUT:
	I2C_delay 				// Repeated start setup time, minimum 4.7us
not_started:
	clear_SDA 				// SCL is high, set SDA from 1 to 0.
	I2C_delay
	clear_SCL
	move r0,1
	st r0,r2,0
	ret

//; This is used to end the i2c protcol
i2c_stop_cond:
	clear_SDA 				// set SDA to 0
	I2C_delay
	set_SCL
	stage_rst
clock_stretch_stop:
	stage_inc 1
	jumps TIMEOUT1,254,ge
	read_SCL
	jumpr clock_stretch_stop,1,lt
TIMEOUT1:
	I2C_delay 				// Stop bit setup time, minimum 4us
	set_SDA  				// SCL is high, set SDA from 0 to 1
	I2C_delay
 	move r2,i2c_started
 	move r0,0
 	st r0,r2,0
//	st r0,r2,4	// new line added by me.. need for multiple-slave and single-master  
 	ret

//; This is used to write a bit operation in i2c protocol
i2c_write_bit: 				// Write a bit to I2C bus
	jumpr bit0,1,lt
	set_SDA
	jump bit1
bit0:
	clear_SDA
bit1:
	I2C_delay 				// SDA change propagation delay
	set_SCL
	I2C_delay

clock_stretch_write:
	read_SCL
	jumpr clock_stretch_write,1,lt
TIMEOUT2:
 	clear_SCL
 	ret

//; This is used to read a bit operation in i2c protocol
i2c_read_bit: 				// Read a bit from I2C bus
	set_SDA 				// Let the slave drive data
	I2C_delay
 	set_SCL
clock_stretch_read:
	read_SCL
	jumpr clock_stretch_read,1,lt
	I2C_delay
	read_SDA 				// SCL is high, read out bit
	clear_SCL
	ret 					// bit in r0

//; This is used to write a byte operation in i2c protocol
i2c_write_byte: 			// Return 0 if ack by the slave.
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
	ret  					// nack
  

//; This is used to read a byte operation in i2c protocol  
    .global i2c_read_byte
i2c_read_byte: 				// Read a byte from I2C bus
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



