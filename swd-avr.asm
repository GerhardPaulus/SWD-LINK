// swd-avr.asm     SWD adapter 

/*
* Software License Agreement (BSD License)
*
* Copyright (c) 2012, Gerhard Paulus (Germany)
* Gerhard.Paulus@gmail.com
*
* Redistribution and use in source and binary forms, 
* with or without modification, are permitted 
* provided  that the following conditions are met 
* 1. Redistributions of source code must retain the 
*    above copyright notice, this list of conditions 
*    and the following disclaimer.
* 2. Redistributions in binary form must reproduce 
*    the above copyright notice, this list of 
*    conditions and the following disclaimer in the
*    documentation and/or other materials provided 
*    with the  distribution.
* 3. Neither the name of the copyright holders nor 
*    the names of its contributors may be used to 
*    endorse or promote products derived from this
*    software without specific prior written 
*    permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS 
* ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES,
* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
* PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT HOLDERS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
* DAMAGES  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
* OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
* OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED 
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
* OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
* DAMAGE.
*
*/

// hardware:
// AVR MEGA8 at VDD +5 Volt 
// (NO external pull-ups on SWD lines !) 
// STM32F4-Discovery (powered by USB)
// clock line: AVR PB0 to STM32F4 PA14 (SWDCLK)
// data  line: AVR PB1 to STM32F4 PA13 (SWDDIO)
// connect common GND 
// documentation:
// arm.com :  ARM Debug Interface version 5 (ADIv5)
// limitation: 
// does not support sticky overrun detection
// (LSB in debug port CTRL/STAT must be 0)
// (sticky overrun has a data phase even if ACK 
// is not OK)

	// adapt following lines to your MC and clock:
	.DEVICE ATmega8
	;.INCLUDE "m8def.inc"
	.EQU RAMSTART = 32 + 64   // 96
	.EQU RAMSIZE  = 1 * 1024  //  1 KB 
	.EQU RAMEND   = RAMSTART + RAMSIZE - 1; 
	; frequency oscillator:
	.EQU fosc = 16000000  ; 16 MHz
	.EQU baud = 38400
	.EQU ubrrValue = fosc / (baud * 16) - 1
	
	.equ UDR =$0c
	.equ UCSRA =$0b
	.equ UCSRB =$0a
	;.equ UCSRC =$20  ;  Note! UCSRC equals UBRRH
	.equ UBRRH =$20  ;  Note! UCSRC equals UBRRH
	.equ UBRRL =$09
	; UCSRA:
	.equ UDRE = 5 
	.equ RXC  = 7
	; UCSRB:
	.equ RXEN =4
	.equ TXEN =3
	
	// port addresses:
	.EQU PINB  = $16  // 22
	.EQU DDRB  = $17  // 23 
	.EQU PORTB = $18  // 24
	
	.EQU  SPL  = 61; // stack pointer LOW
	.EQU  SPH  = 62  // stack pointer HIGH
	.EQU  SREG = 63  // status register
	
	.EQU OUTPUT  = PORTB
	.EQU INPUT   = PINB
	.EQU DIRECTION  = DDRB
	// end of adaptation
	
	; SWD pins on Port B:  PB0 SWDCLK, PB1 SWDDIO
	.EQU CLKMASK    = 1 << 0  ; clock mask
	.EQU DIOMASK    = 1 << 1  ; data mask
	.EQU CLK  = 0  ; clock pin
	.EQU DIO  = 1  ; data pin
	.EQU JTAG2SWD = 0xE79E;  // 16 bit, LSB first 
	.EQU SWD2JTAG = 0xE73C;  // 16 bit, LSB first 
	; commands:
	.EQU STARTCMD  = 1; 
	.EQU STOPCMD   = 2; 
	.EQU FLUSHCMD  = 3; 
	.EQU PROGCMD   = 4; 
	// ack values + parity error :
	.EQU OK     = 1; 
	.EQU WAIT   = 2; 
	.EQU FAULT  = 4; 
	.EQU PARITYERROR = 8; 
	.EQU UNKNOWNCMD  = 0xFF; 
	
	
	// used registers:
	.DEF work   = r16 ; temporary stuff
	.DEF data   = r17
	.DEF count  = r18
	.DEF bytes  = r19
	.DEF parity = r20
	.DEF uart   = r21 ; software UART send/transmit
	.DEF header = r22
	.DEF flags  = r23 ; reserved
	.DEF XL     = r26
	.DEF XH     = r27
	
	
.MACRO setZ
	ldi r30, LOW(@0)
	ldi r31, HIGH(@0)
.ENDM

.MACRO setCargo
	setZ txBuffer
	ldi work, BYTE1(@0)
	st Z+, work
	ldi work, BYTE2(@0)
	st Z+, work
	ldi work, BYTE3(@0)
	st Z+, work
	ldi work, BYTE4(@0)
	st Z+, work
.ENDM

/////////////////////////////////////////////
.DSEG
	rxBuffer:  .BYTE 4  // read from target
	txBuffer:  .BYTE 4  // write to target
	
/////////////////////////////////////////////
.CSEG
interruptVectors:
	rjmp main   ; reset entry point
	// no UART RX interrupt
	
main:
	; init stack pointer:
	ldi work, LOW(RAMEND)
	out SPL, work
	ldi work, HIGH(RAMEND)
	out SPH, work
	
	// port B init:
	ldi work, 0xFF       // all pins output
	out DIRECTION, work  
	ldi work, 0          // data + clock low
	out OUTPUT, work
	
	// UART init:
	sbi UCSRB, RXEN
	sbi UCSRB, TXEN
	ldi work, LOW(ubrrValue)
	out UBRRL,  work
	ldi work, HIGH(ubrrValue)
	out UBRRH, work
	
	whileMain:
		sbis UCSRA, RXC
		rjmp whileMain
		in uart, UDR
		ifStart:
			cpi uart, STARTCMD
			brne endifStart
			rcall start  // reads IDCODE/DPIDR
			rcall swd
			rjmp whileMain
		endifStart:
		;sleep
		ldi uart, UNKNOWNCMD
		rcall putChar
		rjmp whileMain
	endwhileMain:
	
	

swd:  // loop waiting for packets/commands
	rcall ticktack
	sbis UCSRA, RXC  // check UART
	rjmp swd
	in header, UDR
	tst header
	// commands (1 to 127) have MSB cleared
	ifPlus:  
		brmi endifPlus
		ifCmd:  // answer command here !
			cpi header, STOPCMD
			brne elifFlushCmd
			rcall stop
			ldi uart, OK
			rcall putChar
			ret
			;rjmp endifCmd
		elifFlushCmd:
			cpi header, FLUSHCMD
			brne elifProgCmd
			rcall flush
			ldi uart, OK
			rjmp endifCmd
		elifProgCmd:
			cpi header, PROGCMD
			brne elseCmd
			; rcall prog  // TBD
			ldi uart, OK
			rjmp endifCmd
		elseCmd:
			ldi uart, UNKNOWNCMD
		endifCmd:
		rcall putChar
		rjmp swd  // continue
	endifPlus:
	
	// SWD headers have MSB set (park bit)
	ifReading:  // bit[2] 0 write, 1 read
		sbrs header, 2 
		rjmp elseReading
		rcall readPacket
		rjmp swd  // continue
	elseReading:  // writing ...
		setZ txBuffer 
		rcall getInt
		rcall writePacket
		rjmp swd  // continue
	endifReading:
	// end-of-loop
	
start:  // switch from JTAG to SWD
	// 53 high, JTAG2SWD, 53 high, 2 idle, read DPIDR
	setZ txBuffer
	ldi work, BYTE1(JTAG2SWD)
	st Z+, work
	ldi work, BYTE2(JTAG2SWD)
	st Z+, work
	rcall switchMode
	;rcall resetLine
	cbi OUTPUT, DIO   // 2 more LOW clocks
	rcall ticktack
	rcall ticktack
	ldi header, 0xA5  // MUST read DPIDR at once !
	rcall readPacket
	ret
	
stop:  // switch from SWD to JTAG
	// 53 high, SWD2JTAG, 53 high
	setZ txBuffer
	ldi work, BYTE1(SWD2JTAG)
	st Z+, work
	ldi work, BYTE2(SWD2JTAG)
	st Z+, work
	rcall switchMode
	ret
	
switchMode:
	ldi work, 0xFF     // all pins output
	out OUTPUT, work   // data + clock high
	rcall resetLine
	setZ txBuffer
	ldi bytes, 2  // only 16 bits 
	whileSW:
		ld data, Z+
		ldi count, 8
		rcall writeBits
		dec bytes
		brne whileSW
	endwhileSW:
	rcall resetLine
	ret

resetLine:
	sbi DIRECTION, DIO  // output
	sbi OUTPUT, DIO    // high
	ldi count, 53
	whileRL:
		rcall ticktack
		dec count
		brne whileRL
	endwhileRL:
	ret
	

ticktack:   // clock pulse
	sbi OUTPUT, CLK  // high
	nop
	nop
	cbi OUTPUT, CLK  // low
	ret
	
readPacket:
	setZ rxBuffer  // destination
	rcall writeHeader
	rcall readAck   // sets uart
	ifWaitRead:
		cpi uart, WAIT
		breq readPacket
	endifWaitRead:
	ifOkRead:
		cpi uart, OK
		brne endifOkRead
		rcall readData
		ifParityError:
			brcc endifParityError
			ldi uart, PARITYERROR
		endifParityError:
	endifOkRead:
	rcall turnAround
	cbi OUTPUT, DIO  // idle clocking
	rcall putChar  // uart holds ACK/PARITY
	ifOkRead2:
		cpi uart, OK
		brne endifOkRead2
		setZ rxBuffer  // destination
		rcall putInt
	endifOkRead2:
	ret
	
writePacket:
	setZ txBuffer  // source
	rcall writeHeader
	rcall readAck   // sets uart
	ifWaitWrite:
		cpi uart, WAIT
		breq writePacket
	endifWaitWrite:
	rcall turnAround
	ifOkWrite:
		cpi uart, OK
		brne endifOkWrite
		rcall writeData
	endifOkWrite:
	cbi OUTPUT, DIO  // idle clocking
	rcall putChar  // uart holds ACK
	ret

writeHeader:   // header phase (writing 8 bits)
	mov data, header
	ldi count, 8
	rcall writeBits
	ret
	
turnAround:     // host turns from read to write
	// host currently reads, data line is in tri-state 
	// pull-up holds line high (expected by target)
	rcall ticktack
	rcall ticktack
	sbi DIRECTION, DIO  // output
	ret
	
readAck:  // Turnaround + Acknowledge phase 
	cbi DIRECTION, DIO  // input 
	sbi OUTPUT, DIO   // internal pullup active
	// data line tri-stated, pull-up holds line high
	ldi data, 0
	ldi count, 3
	rcall readBits
	mov uart, data  // save ACK bits
	lsr uart
	swap uart
	ret
	
writeBits:   // number of bits in count
	in work, OUTPUT
	whileWB:
		andi work, ~DIOMASK    // 0
		lsr data
		ifSetWB:
			brcc endifSetWB
			ori work, DIOMASK    // 1
			inc parity
		endifSetWB:
		out OUTPUT, work
		rcall ticktack
		dec count  
		brne whileWB
	endwhileWB:
	ret
	
readBits:  // number of bits in count
	whileRB:
		rcall ticktack
		clc
		in work, INPUT
		andi work, DIOMASK
		ifSetRB:
			breq endifSetRB
			sec
			inc parity
		endifSetRB: 
		ror data  // store bit
		dec count
		brne whileRB
	endwhileRB:
	ret
	
writeData:  // write 4 bytes from address in Z
	ldi parity, 0
	ldi bytes, 4
	whileWD:
		ld data, Z+
		ldi count, 8
		rcall writeBits
		dec bytes
		brne whileWD
	endwhileWD:
	; send parity bit:
	mov data, parity
	ldi count, 1
	rcall writeBits
	ret
	
readData:    // read 4 bytes to address in Z
	ldi parity, 0
	ldi bytes, 4
	whileRD:
		ldi count, 8
		rcall readBits
		st Z+, data
		dec bytes
		brne whileRD
	endwhileRD:
	; check parity bit:
	ldi count, 1
	rcall readBits  // adds to parity (data ignored)
	lsr parity
	; lsb 1 + parity 1 -> lsb 0  (OK)
	; lsb 0 + parity 0 -> lsb 0  (OK)
	; C flag is 0 if OK
	; C flag is 1 if error
	ret
	
	
flush:  // dummy write to complete target operation
	ldi data, 0x00
	ldi count, 8
	rcall writeBits
	ret
	
// functions for RS232 connection:

putInt:    // send to PC 4 bytes from address in Z
	push count
	ldi count, 4
	whilePI:
		ld uart, Z+
		rcall putChar
		dec count
		brne whilePI
	endwhilePI:
	pop count
	ret
	
putChar:  // send one byte to PC
	rcall ticktack
	sbis UCSRA, UDRE
	rjmp putChar
	out UDR, uart
	ret

getInt:    // get from PC 4 bytes, save at address in Z
	push count
	ldi count, 4
	whileGI:
		rcall getChar
		st Z+, uart
		dec count
		brne whileGI
	endwhileGI:
	pop count
	ret
	
getChar:  // get one byte from PC
	rcall ticktack
	sbis UCSRA, RXC
	rjmp getChar
	in uart, UDR
	ret

/*
; for debugging only ! *******************
.CSEG
.EXIT
; *****************************************
test:
	ldi uart, 3
	rcall putChar
	ldi uart, 4
	rcall putChar
	setCargo 0xAAAAAAAA  
	
	rcall start  // JTAG2SWD, reads DPIDR
	ldi header, 0xA5  
	rcall readPacket
	
	ldi header, 0x8D  
	rcall readPacket
	
	ldi header, 0xA9  
	setCargo 0x50000000  
	rcall writePacket
	
	ldi header, 0x8D  
	rcall readPacket
	
	ldi header, 0xB1  
	setCargo 0x000000F0  
	rcall writePacket
	
	ldi header, 0x9F  
	rcall readPacket
	
	ldi header, 0xB7  
	rcall readPacket
	
	ldi header, 0xBD  
	rcall readPacket
	rcall stop  // SWD2JTAG
	
	;setZ rxBuffer  // destination
	;rcall putInt
	ret
*/

	