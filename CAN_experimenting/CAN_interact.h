/* This library can be used to add debug interactivity to the MCP2515 Arduino CAN bus. It works with a serial interface, allowing 
the user to send commands to control the MCP2515. The types of operations currently supported are:

 r - read specific memory address.
 b - read the status buffer.
 x - read the RXstatus buffer.
 w - Write to a specific memory address.
 L - Load a Can packet.
 S - Send a Can Packet.
 R - read an RX buffer.
 m - Set mode ([n]ormal, [c]onfig, [l]oopback, L[i]sten only)

 To read a register
 r[REGISTERNAME]. (the final period is very important)

 To load a CAN packet, use the following format
 L[buffer number],[id],[8x data char].
 L0,A5,12345678. 
 */

 #include "sc7-can-libinclude.h"
 #define EOC "UNEXPECTED END OF COMMAND"

 namespace CANinteract
 {
        inline void _error(String str)
        {
          Serial.println(str);
        }
        
 	void mode(MCP2515& c)
 	{
                char mmode;
 		if (Serial.available())
 		   mmode = Serial.read(); // read the next char after the command.
 		else _error(EOC);
 		byte actualmode = 0;
 		switch (mmode)
 		{
 			case 'n': actualmode = MODE_NORMAL; break;
 			case 'c': actualmode = MODE_CONFIG; break;
 			case 'l': actualmode = MODE_LOOPBACK; break;
 			case 'i': actualmode = MODE_LISTEN; break;
 			default : _error("INVALID MODE"); return;
 		}
 		bool result = c.Mode(actualmode);
 		if (result)
 		{
 			Serial.print("Mode set to ");
                        Serial.print(mmode);
                        Serial.println(" successfully");
 		} else {
 			Serial.print("Failed to set mode to ");
                        Serial.println(mmode);
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
			f.id = Serial.read();
		else _error(EOC);
 		f.id << 8;
 		if (Serial.available())
 			f.id |= Serial.read();
 		else _error(EOC);

 		if (Serial.available()
 			  && Serial.read() == ',')
 		{
 			for (byte i = 0; i < 8; i++)
 			{
 				if (Serial.available())
 					f.data[i] = Serial.read();
 			}
 		}
 		else _error(EOC);

 		Serial.readStringUntil('.');

 		return f;
 	}

 	void load(MCP2515& c)
 	{
 		char buffernum;
 		if (Serial.available())
 			 buffernum = Serial.read();
 		else _error("MISSING BUFFER NUMBER");
                 
                if (Serial.available() && Serial.read() != ',')
                {
                  _error("INVALID BUFFER NUMBER");
                }

 		byte buf;
 		switch (buffernum)
 		{
 			case '0': buf = TXB0; break;
 			case '1': buf = TXB1; break;
 			case '2': buf = TXB2; break;
 			default: _error("INVALID TX BUFFER"); return;
 		}

 		c.LoadBuffer(buf,getframe());
 	}

 	void send(MCP2515& c)
 	{
 		char buffernum;
 		if (Serial.available())
 			 buffernum = Serial.read();
 		else _error("MISSING BUFFER NUMBER");

 		byte buf;
 		switch (buffernum)
 		{
 			case '0': buf = TXB0; break;
 			case '1': buf = TXB1; break;
 			case '2': buf = TXB2; break;
 			default: _error("INVALID TX BUFFER"); return;
 		}

 		c.SendBuffer(buf);
 	}

 	void readrx(MCP2515& c)
 	{
                char buffernum;
 		if (Serial.available())
 			 buffernum = Serial.read();
 		else _error("MISSING BUFFER NUMBER");

 		byte buf;
 		switch (buffernum)
 		{
 			case '0': buf = RXB0; break;
 			case '1': buf = RXB1; break;
 			default: _error("INVALID RX BUFFER"); return;
 		}

 		c.ReadBuffer(buf);
 	}
          
 	void read(MCP2515& c)
 	{
 		String regName;
 		byte buf;
 		bool readRXbuffer = false;
 		regName = Serial.readStringUntil('.');

 		if (regName == "CANSTAT")
                  buf = CANSTAT;
                else if (regName == "CANCTRL")
                  buf = CANCTRL;
                else if (regName == "CANINTF")
                  buf = CANINTF;
                else if (regName == "CANINTE")
                  buf = CANINTE;
                else if (regName == "EFLG")
                  buf = EFLG;
                else _error("INVALID REGISTER");

 		Serial.print(regName + " contains ");
 		Serial.println(c.Read(buf),BIN);
 	}

 	void runCommands(MCP2515& c)
 	{
 		byte command;
 		do ; while (!Serial.available());
 		{
 			command = Serial.read();
                        delay(10);

 				switch (command)
	 			{
	 				case 'r': read(c); 		break;
	 				case 'b': status(c); 	break;
	 				case 'x': rxstatus(c);  break;
	 				case 'w': Serial.println("Not Implemented Yet"); break;
	 				case 'L': load(c);		break;
	 				case 'S': send(c);		break;
	 				case 'm': mode(c);		break;
	 				case 'R': readrx(c);	break;
                                        default : Serial.println("INVALID COMMAND");
                                }
                                Serial.readStringUntil('.');
                                
 		}
 	}
 }
