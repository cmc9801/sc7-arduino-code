/* This library can be used to add debug interactivity to the MCP2515 Arduino CAN bus. It works with a serial interface, allowing 
the user to send commands to control the MCP2515. The types of operations currently supported are:

 r - Read specific memory address.
 b - Read the status buffer.
 x - Read the RXstatus buffer.
 w - Write to a specific memory address.
 L - Load a Can packet.
 S - Send a Can Packet.
 R - Read an RX buffer.
 m - Set mode ([n]ormal, [c]onfig, [l]oopback, L[i]sten only)

 To read a register
 r[REGISTERNAME]. (the final period is very important)

 To load a CAN packet, use the following format
 L[buffer number],[id],[8x data char].
 L0,A5,12345678. 
 */

 #include "sc7-can-libinclude.h"

 #define ERROR "COMMAND ERROR";

 namespace CANinteract
 {

 	void mode(MCP2515& c)
 	{
 		if (Serial.available())
 			char mode = Serial.Read(); // Read the next char after the command.
 		else throw ERROR;

 		byte actualmode = 0;
 		switch (mode)
 		{
 			case 'n': actualmode = MODE_NORMAL; break;
 			case 'c': actualmode = MODE_CONFIG; break;
 			case 'l': actualmode = MODE_LOOPBACK; break;
 			case 'i': actualmode = MODE_LISTEN; break;
 			default : throw "INVALID MODE"; break;
 		}
 		bool result = c.Mode(actualmode);
 		if (result)
 		{
 			Serial.println("Mode set to " + mode + " successfully");
 		} else {
 			Serial.println("Failed to set mode to " + mode);
 		}
 	}

 	void status(MCP2515& c)
 	{
 		byte statusbuffer = c.Status();
 		Serial.print("The status buffer contains: ");
 		Serial.println(statusbuffer,BIN);
 	}

 	void rxstatus(MCP2515& c)
 	{
 		byte statusbuffer = c.RXStatus();
 		Serial.print("The RXstatus buffer contains: ");
 		Serial.println(statusbuffer,BIN);
 		if (statusbuffer & 0x80)
 			Serial.println("RX Buffer 0 is full");
 		if (statusbuffer & 0x40)
 			Serial.println("RX Buffer 1 is full");
 	}

 	Frame getframe()
 	{
 		Frame f;
 		if (Serial.available())
			f.id = Serial.Read();
		else throw ERROR;
 		f.id << 8;
 		if (Serial.available())
 			f.id |= Serial.Read();
 		else throw ERROR;

 		if (Serial.available()
 			  && Serial.Read() == ',')
 		{
 			for (byte i = 0; i < 8; i++)
 			{
 				if (Serial.available())
 					f.data[i] = Serial.Read();
 			}
 		}
 		else throw ERROR;

 		Serial.readStringUntil('.');

 		return f;
 	}

 	void load(MCP2515& c)
 	{
 		if (Serial.available()
 			char buffernum = Serial.Read();
 		else throw "MISSING BUFFER NUMBER";

 		byte buf;
 		switch (buffernum)
 		{
 			case '0': buf = TXB0; break;
 			case '1': buf = TXB1; break;
 			case '2': buf = TXB2; break;
 			default: throw "INVALID TX BUFFER";
 		}

 		c.LoadBuffer(getframe(),buf);
 	}

 	void send(MCP2515& c)
 	{
 		if (Serial.available()
 			char buffernum = Serial.Read();
 		else throw "MISSING BUFFER NUMBER";

 		byte buf;
 		switch (buffernum)
 		{
 			case '0': buf = TXB0; break;
 			case '1': buf = TXB1; break;
 			case '2': buf = TXB2; break;
 			default: throw "INVALID TX BUFFER";
 		}

 		c.SendBuffer(buf);
 	}

 	void read(MCP2515& c)
 	{
 		String regName;
 		regName = Serial.readStringUntil('.');

 		switch (regName)
 		{
 			case ""
 		}
 	}
 }
