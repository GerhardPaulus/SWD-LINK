
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


// usage:    java IO <portName> <baudRate>
// defaults: /dev/ttyS1  38400
// documentation:
// arm.com :  ARM Debug Interface version 5 (ADIv5)
// limitation: 
// does not support sticky overrun detection
// (LSB in debug port CTRL/STAT must be 0)

import java.io.*;
import java.util.*;
// RS232 interface:
// import javax.comm.*  // sub-optimal
import gnu.io.*;  // RXTX preferred, you find it on 
// http://rxtx.qbang.org/wiki/index.php/Main_Page
// you may have to add in ~/.bashrc something like:
// export _JAVA_OPTIONS='-Djava.library.path=.:/usr/lib/jni'
// export CLASSPATH=.:/usr/share/java/RXTXcomm.jar
// or call with  java -cp  -D  IO ...


public class IO {
	int[] packet = new int[3]; // header,ack,data
	SerialPort port = null;
	InputStream   in = null;
	OutputStream out = null;
	String binName = null;
	
	// constants:
	final static int DP = 0;  // debug port
	final static int AP = 1;  // access port
	final static int WRITE = 0;
	final static int READ  = 1;
	final static int CMD   = 2;
	// packet elements:
	final static int HEADER = 0;
	final static int ACK    = 1;
	final static int DATA   = 2;
	// ack values:
	final static int OK     = 1;  
	final static int WAIT   = 2;
	final static int FAULT  = 4;
	final static int PARITYERROR = 8;
	final static int UNKNOWNCMD  = 0xFF;
	
	// commands:
	final static int START = 1;;
	final static int STOP  = 2;
	final static int FLUSH = 3;
	final static int PROG  = 4;
	
	// core debug registers:
	final static int DHCSR = 0xE000EDF0; // control+status
	final static int DCRSR = 0xE000EDF4; // select 
	final static int DCRDR = 0xE000EDF8; // data
	final static int DEMCR = 0xE000EDFC; // monitor
	
	
/**   **/
public static void main(String[] params) {
	System.out.println("IO.main()") ;
	String portName = "/dev/ttyS1";
	int baudRate = 38400; 
	if (params.length > 0) {
		portName = params[0];
	}
	if (params.length > 1) {
		try {
			baudRate = Integer.parseInt(params[1]);
		} catch (Exception ex) { 
			_("IO.main() problems parsing "+params[1]);
			return ;
		} 
	}
	
	IO io = new IO();
	io.open(portName, baudRate);
	io.blink();
	io.close();
	//io.listen();  // stop with Ctrl+C
}

/** opens RS232 connection */
int open(String portName, int baudRate) { 
	int openTimeout = 100;  // millis
	CommPortIdentifier cpi = null;
	//SerialPort port = null;
	
	//Enumeration enum = 
	//CommPortIdentifier.getPortIdentifiers();
	//while (enum.hasMoreElements()) {
		//cpi = (CommPortIdentifier) enum.nextElement();
		//System.out.println("IO.open() port: "+cpi.getName()) ;
	//}
	
	try {
		cpi = CommPortIdentifier.getPortIdentifier(portName);
		port = (SerialPort) cpi.open("IO", openTimeout); 
		port.setSerialPortParams(baudRate,
			SerialPort.DATABITS_8,
		  SerialPort.STOPBITS_1,
		  SerialPort.PARITY_NONE);
		//int xonOut = SerialPort.FLOWCONTROL_XONXOFF_OUT;
		//int xonIn  = SerialPort.FLOWCONTROL_XONXOFF_IN;
		//port.setFlowControlMode(xonIn | xonOut);
		_("IO.open() port: "+portName+" baud: "+baudRate);
		in   = port.getInputStream();
		out  = port.getOutputStream();
	} catch (Exception ex) {
		_("IO.open() ex: "+ex) ;
	}
	return 0;
}

/** closes  RS232 connection */
int close() {  
	try {
		port.close();
	} catch (Exception ex) { 
		_("IO.close() ex: "+ex);
	} 
	return 0;
}

/** read-request */
int r(int port, int address) { 
	//_("IO.r() "+port+"  "+address);
	//int header = genHeader(port, READ, address);
	packet[HEADER] = genHeader(port, READ, address);
	int ack = request(READ, packet);
	return ack;
}

/**  write-request */
int w(int port, int address, int data) { 
	String hex = Integer.toHexString(data);
	//_("IO.w() "+port+"  "+address+"  0x"+hex);
	packet[HEADER] = genHeader(port, WRITE, address);
	packet[DATA]   = data;
	int ack = request(WRITE, packet);
	return ack;
}
 
/** sends command to adapter */
int command(int cmd) { 
	//_("IO.command() "+cmd);
	packet[HEADER] = cmd;
	int ack = request(CMD, packet);
	return ack;
}
 
/** assemble SWD header byte */
int genHeader(int dpap, int rw, int address) {  
	address = address >> 2 & 0x3;
	int parity = 0; // odd parity over bits 4:1 
	if (address == 1 || address == 2) { 
		parity = 1;
	} else if (address == 3) { 
		parity = 2;
	} 
	parity += dpap;
	parity += rw;
	parity &= 1;
	int header = 0x81;       // park,stop,start bit
	header |= dpap << 1;     // port (DP, AP)
	header |= rw   << 2;     // type (WRITE, READ)
	header |= address  << 3; // address (3:2)
	header |= parity   << 5; // odd parity 
	//String hex = Integer.toHexString(header);
	//_("IO.genHeader() header: "+hex) ;
	return header;
}
 
/** send packet to adapter -> target */
int request(int action, int[] packet) { 
	int header = packet[HEADER];
	int data   = packet[DATA];
	// write to host -> target
	try {
		out.write(header);
		if (action == WRITE) { 
			out.write(data >>  0 & 0xFF);
			out.write(data >>  8 & 0xFF);
			out.write(data >> 16 & 0xFF);
			out.write(data >> 24 & 0xFF);
		} 
	} catch (Exception ex) { 
		_("IO.request() writing ex: "+ex) ;
		return -1;
	} 
	// read from host <- target
	int ack = 0;
	try {
		ack = in.read();
		packet[ACK] = ack;
		if (ack != OK) { 
			_("IO.request() !!! ack: "+ack) ;
		} 
		if (action == CMD && header == START) { 
			action = READ;  // reads DPIDR
		} 
		if (action == READ && ack == OK) { 
			data = in.read();
			data |= in.read() << 8;
			data |= in.read() << 16;
			data |= in.read() << 24;
			packet[DATA] = data;
			String hex = Integer.toHexString(data);
			_("IO.request() got  0x"+hex) ;
		} 
	} catch (Exception ex) { 
		_("IO.request() reading ex: "+ex) ;
		return -1;
	} 
	return ack;
}
 
/**  */
static void _(String str) { 
	System.out.println(str);
}

/**  */
int listen() {  
	_("IO.listen() ... stop with Ctrl+C ") ;
	try {
		while (true) { 
			int b = in.read();
			String hex = Integer.toHexString(b);
			_("IO.request() listen: "+hex) ;
		} 
	} catch (Exception ex) { 
		_("IO.listen() ex: "+ex) ;
	} 
	return 0;
}

/**  */
int getPacketData() {  
	return packet[DATA];
}

/**  */
int writeRegister(int reg, int value) {  
	// w(0, 0x8, 0x00000000); //  SELECT
	w(1, 0x4, DCRDR);       // TAR  DCRDR (data)
	w(1, 0xC, value);       // DRW
	w(1, 0x4, DCRSR);       // TAR  DCRSR (select)
	w(1, 0xC, 1<<16 | reg); // DRW  (start write transfer)
	w(0, 0x8, 0x00000000); //  SELECT  stalls
	//r(0, 0x4);             // CTRL write effective
	return 0;
}
 
/**  */
int readRegister(int reg) {  
	// w(0, 0x8, 0x00000000); //  SELECT
	w(1, 0x4, DCRSR);   // TAR  DCRSR (select)
	w(1, 0xC, reg);     // DRW  (starts read transfer)
	w(1, 0x4, DCRDR);   // TAR  DCRDR (data)
	r(1, 0xC);          // DRW
	r(0, 0xC);          // RDBUFF read effective
	return getPacketData();
}
 
/**  */
int blink() { 
	int data = 0;
	// start SWD session
	// (initiating bit sequence, also reads IDCODE):
	command(START);
	
	
	// note:
	// reading on 0 (Debug Port)  returns data directly
	// writing on 0 (Debug Port)  has immediate effect
	// reading on 1 (Access Port) returns data on 
	//                            following read
	// writing on 1 (Access Port) is effective one 
	//                            packet later 
	
	r(0, 0x0); //  0x2BA01477  IDCODE
	
	//command(STOP);
	//if (true) { return 0;} 
	
	
	r(0, 0x4); //  0xF0000040  CTRL/STAT
	// power up debug and system
	w(0, 0x4, 0x50000000); //  CTRL/STAT
	r(0, 0x4); //  0xF0000040  CTRL/STAT
	w(0, 0x8, 0x000000F0); //  SELECT  AP 0x00 bank 0xF
	r(1, 0xC); //  discard     FC: AP ID
	r(1, 0x8); //  0x24770011  F8: AP debug base address
	r(0, 0xC); //  0xE00FF003  RDBUFF
	// last 3 reads return for STM32F4 :
	// dummy       
	// ID of AP    0x24770011
	// debug base  0xE00FF003
	
	w(0, 0x8, 0x00000000); // SELECT  AP 0x00, bank 0x0
	// read default bus protection bits 30:24 of CSW
	// (IMPLEMENTATION DEFINED !)
	r(1, 0x0); //  discard  CSW
	r(0, 0xC); //           RDBUFF
	data = getPacketData();
	//String hex = Integer.toHexString(getPacketData());
	//_("IO.blink()  prot CSW: "+hex);
	
	data |= 0x2;  // 32-bit access  (0b0010)
	w(1, 0x0, data); //  CSW   
	
	// Halt the processor:
	w(1, 0x4, DHCSR);      //  TAR  
	w(1, 0xC, 0xA05F0003); //  DRW  C_HALT + C_DEBUGEN 
	r(0, 0x4); //  0xF0000040  CTRL/STAT
	
	// read MCU IDCODE on private peripheral bus
	w(1, 0x4, 0xE0042000); //  TAR
	r(1, 0xC); //  discard     DRW
	r(0, 0xC); //  0x10016413  RDBUFF 
	// 0x10016413 is identifier for STM32F4
	// REV_ID 0x1001   DEV_ID 0x413
	
	// activate port D in RCC_AHB1ENR:
	w(1, 0x4, 0x40023800 + 0x30); // TAR  RCC_AHB1ENR
	w(1, 0xC, 1 << 3);     // DRW  port D activated
	w(0, 0x8, 0x00000000); // SELECT  stalls
	
	// PD12, PD13, PD14, PD15 output:
	w(1, 0x4, 0x40020C00); // TAR  port D base
	w(1, 0xC, 0x55000000); // DRW  PD15:PD12 output 
	w(0, 0x8, 0x00000000); // SELECT  stalls
	
	// blink iterations: 
	w(1, 0x4, 0x40020C00 + 0x14); // TAR  port D output
	for (int i=0; i < 7; i++) { 
		int set = 0;
		if ((i & 1) == 0) { // even
			// PD12, PD13 set:
			set = 1 << 13 | 1 << 12;
		} else { // odd
			// PD14, PD15 set:
			set = 1 << 15 | 1 << 14;
		} 
		w(1, 0xC, set);        // DRW   
		try {
			Thread.sleep(500);  // 500 ms
		} catch (Exception ex) { 
			//
		} 
	} 
	
	// read reset handler location at FLASH 0x08000004
	w(1, 0x4, 0x08000004);  //  TAR  
	r(1, 0xC); //  discard   DRW
	r(0, 0xC); //            RDBUFF
	int resetHandler = getPacketData();
	
	// prepare a soft reset:
	// write reset handler location into PC
	// (program counter is register 15)
	writeRegister(15, resetHandler);
	
	// start processor:
	// (PC points to start of reset handler)
	w(1, 0x4, DHCSR);      //  TAR  
	w(1, 0xC, 0xA05F0001); //  DRW  DBGEN, no HALT
	w(0, 0x8, 0x00000000); //  SELECT  stalls
	//r(0, 0x4); //  0xF0000040  CTRL/STAT
	
	// stop SWD session:
	command(STOP);
	return 0;
}
 
}  // end-of-class IO.java



   
//MEMINI.ORG  cursor:378,19  bookmarks:61,61,61,61,61,61,61,61,61,