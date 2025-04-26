#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/fcntl.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <errno.h>
#include <pthread.h>  //for thread
#include <cstring>
#include <wiringPi.h>  //for GPIO access

#include "CNCSerial.h"
#include "Timers.h"
#include "XMLParser.h"


//#define DEBUG 1


unsigned char inbuf[CNC_BUF_SIZE];
unsigned char inbuf2[CNC_BUF_SIZE];  //parsed and sanitized version of inbuf
unsigned char outbuf[CNC_BUF_SIZE];
unsigned char inputbuf[CNC_BUF_SIZE];



CNCSerial::CNCSerial()
{
	serial_fd = 0;
	log_fd = 0;
	
	pReopen = 0;
	
	pSendingFile = 0; 
	pReceivingFile = 0;
		
	pStartSendingFile = 0;
	pStopSendingFile = 0;
		
	pStartReceivingFile = 0;
	pStopReceivingFile = 0;
		
	pLogMode = LOG_MODE_FILE|LOG_MODE_PRINT;
	
	pLogRx = 1;	//default to logging the RX and TX data
	pLogTx = 1;
	
	pCurrentLine = 0;
	pNumLines = 0;

	pIgnoreNULL = 1; //CNC will send lots of 0x00 when delaying. ignore them.

	pRTS = 0;
	pDTR = 0;
	pDSR = 0;
	pDone = 0;

	pNewRTS = 0;
	pNewDTR = 0;
	pNewDSR = 0;
	pNewFlowControl = FLOW_CONTROL_HARDWARE;  //default to HW
	pClearToSend = 0; //default to flow control stopping any sends.
	pUserPaused = 0;	//user paused
	pMachinePaused = 0;
	
	serialthread = 0;  //this is a pointer, so NULL it out initially
	iothread = 0;  //this is a pointer, so NULL it out initially
	
	pIOThreadRunning = 0;
	pSerialThreadRunning = 0;
	//flow_control;
	//stop_bits;
	
	//TODO: put default serial params here:
	pXOFFByte = XOFF;  //or XOFF2
	pUseRxFlowControl = 0;
	pUseStartStopChar = '%';
	pStartStopChar = 0;
	
	
	pSingleStep = 0; //Always start up with this off
	pDoStep = 0;
	
	pPacketDelay = 0;
	pPacketLength = 0;
	pflushRequested = 1;
	
	strcpy( pSerialName, SERIAL_PORT);  //this needs to be something
	
	pFileSize = 0;
	
	rxData.reset();
	txData.reset();
}

CNCSerial::~CNCSerial()
{
	stopThreads();
}

void CNCSerial::setSerialName(char *name)
{
	strcpy(pSerialName, name);
}

char *CNCSerial::getSerialName()
{
	return pSerialName;
}
		
int CNCSerial::getCurrentLine()
{
	return pCurrentLine;
}

int CNCSerial::getNumLines()
{
	return pNumLines;
}

void CNCSerial::setLogRx( int value )
{
	pLogRx = value & 0x01;
}

void CNCSerial::setLogTx( int value )
{
	pLogTx = value & 0x01;
}

int CNCSerial::getLogRx()
{
	return pLogRx;
}

int CNCSerial::getLogTx()
{
	return pLogTx;
}

void CNCSerial::setPacketDelayMs(unsigned int microseconds)
{
	pPacketDelay = microseconds;
}

unsigned int CNCSerial::getPacketDelayMs()
{
	return pPacketDelay;
}

void CNCSerial::setPacketLength(unsigned int bytes)
{
	pPacketLength = bytes;
}
unsigned int CNCSerial::getPacketLength()
{
	return pPacketLength;
}

ParityValues CNCSerial::getParity()
{
	return pParity;
}
void CNCSerial::setParity(ParityValues value)
{
	pParity = value;
}

StopBitValues CNCSerial::getStopBits()
{
	return pStopBits;
}
void CNCSerial::setStopBits(StopBitValues value)
{
	pStopBits = value;
}


void CNCSerial::setDataBits(DataBitValues value)
{
	pDataBits = value;
}
DataBitValues CNCSerial::getDataBits()
{
	return pDataBits;
}

void CNCSerial::setBaud(int value)
{
	pBaudRate = value;
}
int CNCSerial::getBaud()
{
	return pBaudRate;
}
		
void CNCSerial::setXOFFByte( char byte )
{
	if(byte == XOFF2)  //0x93
		pXOFFByte = byte;
	else
		pXOFFByte = XOFF;  //default
}
unsigned char CNCSerial::getXOFFByte()
{
	return pXOFFByte;
}

void CNCSerial::StartPauseButtonPress()
{
	pStartPauseButtonFlag = 1;
}

void CNCSerial::StopButtonPress()
{
	pStopButtonFlag = 1;
}

int CNCSerial::getSingleStep()
{
	return pSingleStep;
}

void CNCSerial::setSingleStep(int value)
{
	pSingleStep = (value & 0x01);
	if(pSingleStep == 0)
	{
		pUserPaused = 1; //set to paused when turning off the single step mode
	}
	else
	{
		pUserPaused = 0; //turn off paused when entering single step mode
		pDoStep = 0;
	}
	log("Setting Single Step: %d\n", pSingleStep);
}
		

FlowControlValues CNCSerial::getFlowControl()
{
	return pFlowControl;
}


void CNCSerial::setUserPaused(int value)
{
	pUserPaused = 0;
	if(value)
		pUserPaused = 1;
}

int CNCSerial::getUserPaused()
{
	return pUserPaused;
}

void CNCSerial::setFlowControl(FlowControlValues value)  //FLOW_CONTROL_SOFTWARE, FLOW_CONTROL_GRBL, FLOW_CONTROL_HARDWARE, or FLOW_CONTROL_NONE
{
	pNewFlowControl = value;
}

int CNCSerial::getMachinePaused()
{
	return pMachinePaused;
}


int CNCSerial::isFileSending()
{
	return pSendingFile;
}

int CNCSerial::isFileReceiving()
{
	return pReceivingFile;
}

int CNCSerial::isSerialThreadRunning()
{
	return pSerialThreadRunning;
}

// DTR is a modem handshake line
// Not necessary for this application

void CNCSerial::setDTR( unsigned short level)  //0 or non-zero
{
	//log("Requesting DTR: %d\n", level);
	pNewDTR = level;
}

void CNCSerial::setDSR( unsigned short level)
{
	pNewDSR = level;
}


int CNCSerial::getDTR()
{
	return pDTR;
}

int CNCSerial::getDSR()
{
	return pDSR;
}

// RTS output lets the other device (CNC) know that we are ready to
// receive data.
void CNCSerial::setRTS(unsigned short level)  //Public
{
	log("Requesting RTS: %d\n", level);
	pNewRTS = level;
}



int CNCSerial::getRTS()
{
	return pRTS;
}


int CNCSerial::getUseRxFlowControl()
{
	return pUseRxFlowControl;
}

void CNCSerial::setUseRxFlowControl( int val )
{
	pUseRxFlowControl = (val & 0x01);
}


int CNCSerial::getUseStartStopChar()
{
	return pUseStartStopChar;
}

void CNCSerial::setUseStartStopChar( int val )
{
	pUseStartStopChar = val & 0x01;
}

unsigned char CNCSerial::getStartStopChar()
{
	return pStartStopChar;
}

void CNCSerial::setStartStopChar( unsigned char val )
{
	pStartStopChar = val;
}
		
		


unsigned long CNCSerial::getFileSize()
{
	return pFileSize;
}


unsigned long CNCSerial::getNumBytesSent()
{
	return pBytesSent;
}
		



void CNCSerial::setFlush(int value)
{
	pflushRequested = value & 0x01;
}

int CNCSerial::getFlush()
{
	return pflushRequested;
}



void CNCSerial::resetStats()
{
	pBytesReceived = 0;
	pBytesSent = 0;
}

int CNCSerial::isStatusNew()
{
	return pStatusNew;
}

void CNCSerial::setStatus(char *text)
{
	sprintf(pStatus, "%s", text);
	pStatusNew = 1;
}

char *CNCSerial::getStatus()
{
	pStatusNew = 0;
	return pStatus;
}
	
int CNCSerial::saveSettings(char *settingsFileName)
{
	//FIXME: add prefix to this as well
	
	//Save the settings to a XML file.
	XMLParser savefile;
	savefile.parseFile( settingsFileName );  //load and parse the existing file.
	savefile.setValue("CNCSerial.Port.Name", pSerialName);
	savefile.setValue("CNCSerial.Port.BaudRate", pBaudRate);
	
	switch(pDataBits)
	{
		case DATA_BITS_5: savefile.setValue("CNCSerial.Port.DataBits", "5"); break;
		case DATA_BITS_6: savefile.setValue("CNCSerial.Port.DataBits", "6"); break;
		case DATA_BITS_8: savefile.setValue("CNCSerial.Port.DataBits", "8"); break;
		case DATA_BITS_7: 
		default:
						savefile.setValue("CNCSerial.Port.DataBits", "7"); break;
	};
	switch(pStopBits)
	{
		case STOP_BITS_1: savefile.setValue("CNCSerial.Port.StopBits", "1"); break;
		case STOP_BITS_2: 
		default:
						savefile.setValue("CNCSerial.Port.StopBits", "2"); break;
	};	
	switch(pParity)
	{
		case PARITY_NONE:  savefile.setValue("CNCSerial.Port.Parity", "NONE"); break;
		case PARITY_ODD:   savefile.setValue("CNCSerial.Port.Parity", "ODD"); break;
		case PARITY_EVEN:  savefile.setValue("CNCSerial.Port.Parity", "EVEN"); break;
	};
	switch(pFlowControl)
	{
		case FLOW_CONTROL_NONE: 	savefile.setValue("CNCSerial.Port.FlowControl", "NONE"); break;
		case FLOW_CONTROL_SOFTWARE: savefile.setValue("CNCSerial.Port.FlowControl", "SOFTWARE"); break;
		case FLOW_CONTROL_HARDWARE: savefile.setValue("CNCSerial.Port.FlowControl", "HARDWARE"); break;
		case FLOW_CONTROL_GRBL:		savefile.setValue("CNCSerial.Port.FlowControl", "GRBL"); break;
	};
	
	savefile.setValue("CNCSerial.PacketLength", (int)pPacketLength);
	savefile.setValue("CNCSerial.PacketDelay",  (int)pPacketDelay);
	savefile.setValue("CNCSerial.UseRxFlowControl", (int)pUseRxFlowControl);
	savefile.setValue("CNCSerial.UseStartStopChar", (int)pUseStartStopChar);
	
	char string[3];
	sprintf(string, "%c", (char )pStartStopChar);
	savefile.setValue("CNCSerial.StartStopChar", string);
	
	
	switch(pXOFFByte)
	{
		case 0x93: savefile.setValue("CNCSerial.XOFFByte", "0x93"); break;
		case 0x13:
		default:
			savefile.setValue("CNCSerial.XOFFByte", "0x13"); break;
	};
	savefile.save(settingsFileName);

	return CNC_OK;
}

int CNCSerial::loadSettings(char *settingsFileName, char *prefix)
{
	//Read the settings from XML
	std::string value;
	char valuestr[32];	
	char namestr[64];
	XMLParser settingsfile;
	settingsfile.parseFile(settingsFileName);
	
	settingsfile.printAll();
	//settingsfile.save((char *)"./Files/temp.xml");//this is only for testing!

	sprintf(namestr, "%s.Port.Name", prefix);
	
	value = settingsfile.findValue((char *)namestr);
	if(value.length() > 0)
	{
		strcpy(pSerialName, value.c_str());
		log("Loaded Port: %s\n", pSerialName);
	}
	
	sprintf(namestr, "%s.Port.BaudRate", prefix);
	value = settingsfile.findValue((char *)namestr);
	if(value.length() > 0)
	{
		
		strcpy(valuestr, value.c_str());
		pBaudRate = atoi(valuestr);
		if(pBaudRate < 50)
			pBaudRate = 50;
		if(pBaudRate > 4000000)
			pBaudRate = 4000000;
		log("Loaded Baud: %d\n", pBaudRate);
	}	
	
	sprintf(namestr, "%s.Port.DataBits", prefix);
	value = settingsfile.findValue((char *)namestr);
	if(value.length() > 0)
	{
		strcpy(valuestr, value.c_str());
		int val = atoi(valuestr);
		switch(val)
		{
			case 5: pDataBits = DATA_BITS_5; break;
			case 6: pDataBits = DATA_BITS_6; break;
			case 7: pDataBits = DATA_BITS_7; break;
			case 8: pDataBits = DATA_BITS_8; break;
			default:
				pDataBits = DATA_BITS_7; break;
		};			
		log("Loaded DataBits: %d\n", val);
	}
	
	sprintf(namestr, "%s.Port.StopBits", prefix);
	value = settingsfile.findValue((char *)namestr);
	if(value.length() > 0)
	{
		strcpy(valuestr, value.c_str());
		int val = atoi(valuestr);
		switch(val)
		{
			case 1: pStopBits = STOP_BITS_1; break;
			case 2: pStopBits = STOP_BITS_2; break;
			default:
				pStopBits = STOP_BITS_2; break;
		};			
		log("Loaded StopBits: %d\n", val);
	}
	
	sprintf(namestr, "%s.Port.Parity", prefix);
	value = settingsfile.findValue((char *)namestr);
	if(value.length() > 0)
	{
		if(value.find("NONE") != std::string::npos)
			pParity = PARITY_NONE;
		if(value.find("ODD") != std::string::npos)
			pParity = PARITY_ODD;
		if(value.find("EVEN") != std::string::npos)
			pParity = PARITY_EVEN;		
		log("Loaded Parity: %s\n", value.c_str());
	}	
	
	sprintf(namestr, "%s.Port.FlowControl", prefix);
	value = settingsfile.findValue((char *)namestr);
	if(value.length() > 0)
	{
		if(value.find("NONE") != std::string::npos)
		{
			pFlowControl  = pNewFlowControl = FLOW_CONTROL_NONE;
		}
		if(value.find("SOFT") != std::string::npos)
		{
			pFlowControl = pNewFlowControl = FLOW_CONTROL_SOFTWARE;
		}
		if(value.find("HARD") != std::string::npos)
		{
			pFlowControl = pNewFlowControl = FLOW_CONTROL_HARDWARE;
		}
		if(value.find("GRBL") != std::string::npos)
		{
			pFlowControl = pNewFlowControl = FLOW_CONTROL_GRBL;
		}
		log("Loaded FlowControl: %s\n", value.c_str());		
	}	
	
	sprintf(namestr, "%s.PacketLength", prefix);
	value = settingsfile.findValue((char *)namestr);
	if(value.length() > 0)
	{
		strcpy(valuestr, value.c_str());
		pPacketLength = atoi(valuestr);
		if(pPacketLength < 0)
			pPacketLength = 0;
		if(pPacketLength > 10000)
			pPacketLength = 10000;
		log("Loaded PacketLength: %d\n", pPacketLength);
	}	

	sprintf(namestr, "%s.PacketDelay", prefix);
	value = settingsfile.findValue((char *)namestr);
	if(value.length() > 0)
	{
		strcpy(valuestr, value.c_str());
		pPacketDelay = atoi(valuestr);
		if(pPacketDelay < 0)
			pPacketDelay = 0;
		if(pPacketDelay > 10000)
			pPacketDelay = 10000;
		log("Loaded PacketDelay: %d\n", pPacketDelay);
	}		
	
	sprintf(namestr, "%s.XOFFByte", prefix);
	value = settingsfile.findValue((char *)namestr);
	{
		if(value.find("93") != std::string::npos)
			pXOFFByte = XOFF2;  //0x93
		else
			pXOFFByte = XOFF;  //0x13
	}
	
	sprintf(namestr, "%s.UseRxFlowControl", prefix);
	value = settingsfile.findValue((char *)namestr);
	strcpy(valuestr, value.c_str());
	pUseRxFlowControl = atoi(valuestr) & 0x01;
	
	sprintf(namestr, "%s.UseStartStopChar", prefix);
	value = settingsfile.findValue((char *)namestr);
	strcpy(valuestr, value.c_str());
	pUseStartStopChar = atoi(valuestr) & 0x01;
	
	sprintf(namestr, "%s.StartStopChar", prefix);
	value = settingsfile.findValue((char *)namestr);
	pStartStopChar = (unsigned char)value[0];
	
	return CNC_OK;
}



void CNCSerial::printStatus()
{
	if( (serial_fd > 0))
	{
		log("Serial Port %s Open. FD=%d\n", SERIAL_PORT);
	}
	else
	{
		log("Serial Port is Closed.");
	}
	
	if(pFlowControl == FLOW_CONTROL_SOFTWARE)
		log("Flow Control: Software\r\n");
	else if(pFlowControl == FLOW_CONTROL_HARDWARE)
		log("Flow Control: Hardware\r\n");
	else if(pFlowControl == FLOW_CONTROL_GRBL)
		log("Flow Control: GRBL\r\n");
	else if(pFlowControl == FLOW_CONTROL_NONE)
		log("Flow Control: None\n");
			
	log("RTS: %d (%d)\n", pRTS, pNewRTS);
	log("DTR: %d (%d)\n", pDTR, pNewDTR);
	log("CTS: %d\n", pClearToSend);
	log("DSR: %d (%d)\n", pDSR, pNewDSR);

	log("Bytes RX    : %u\r\n", pBytesReceived);
	log("Bytes TX    : %u\r\n", pBytesSent);
	
	log("rxData Size : %d  max: %d\n", rxData.getSize(), rxData.getMaxSize());
	log("txData Size : %d  max: %d\n", txData.getSize(), txData.getMaxSize());

	if(pUserPaused || pMachinePaused)
	{
			log(" ** Paused ** \r\n");
	}
						
}

void CNCSerial::clearQueues()
{
	txData.reset();
	rxData.reset();
	pBytesSent = 0;
	pBytesReceived = 0;
}

int CNCSerial::send(unsigned char *txdata, int length)  //Public
{
	if(txData.putData(txdata, length) != CIRC_QUEUE_OK)
	{
		log("Error adding data to the TX Queue.\n");
		return CNC_ERROR;
	}
	if(pFlowControl == FLOW_CONTROL_HARDWARE)
		setRTS( 1 );
		
	return CNC_OK;
}

int CNCSerial::sendByte( unsigned char byte )
{
	if(txData.putData(&byte, 1) != CIRC_QUEUE_OK)
	{
		log("Error adding byte to the TX Queue.\n");
		return CNC_ERROR;
	}
	return CNC_OK;
}

int CNCSerial::rxBytesWaiting()
{
	if(serial_fd > 0)
	{
		int num_bytes;
		ioctl(serial_fd, FIONREAD, &num_bytes);  //are there bytes to be read?
		return num_bytes;
	}
	return 0;  //port is closed, no data
}


int CNCSerial::getRxQueueSize()  //from the queue
{
	return rxData.getSize();
}

unsigned char CNCSerial::getRxQueueByte()  //from the queue
{
	//get the data off the queue
	unsigned char byte;
	int n = rxData.getData( &byte, 1 );
	if(n == 1)
		return byte;
	return 0;
}

int CNCSerial::getCTS()
{
	return pClearToSend;
}



int CNCSerial::getStopButton()
{
	return pStopButton;
}

int CNCSerial::getPauseButton()
{
	return pStartPauseButton;
}
	
int CNCSerial::startThreads(int useButtonThread)
{
	Timers timeout;
	
	if(serialthread != 0)
	{
		log("Error: calling startThreads() but the serial thread already exists.\n");
		return CNC_ERROR;   //the thread already exists.
	}
	if((useButtonThread != 0) && (iothread != 0))
	{
		log("Error: calling startThreads() but the button thread already exists.\n");
		return CNC_ERROR;
	}
    // Create and execute the thread
    serialthread = new std::thread(&CNCSerial::serialPollingThread, this); 
    
    //wait for it to get up and running
    timeout.Start();
    while(!pSerialThreadRunning)
	{
		usleep(1000); //delay a ms
		//delay here, but include a timeout
		if(timeout.Expired(TIMER_5S_TIMEOUT))
		{
			log("Error: Timeout (%dms) waiting for serial thread to start.\n", TIMER_5S_TIMEOUT);				
			return CNC_TIMEOUT;
		}
	}
	
	if(useButtonThread)
	{
		iothread = new std::thread(&CNCSerial::ioPollingThread, this);
		timeout.Start();
		while(!pIOThreadRunning)
		{
			usleep(1000);
			//delay here, but include a timeout
			if(timeout.Expired(TIMER_5S_TIMEOUT))
			{
				log("Error: Timeout (%dms) waiting for button thread to start.\n", TIMER_5S_TIMEOUT);
				return CNC_TIMEOUT;
			}
		}
	}
    
    
	return CNC_OK;
}

int CNCSerial::stopThreads()
{
	try 
	{
		if(!pDone)
		{
			log("Stopping Serial thread.\n");
			pDone = 1;
			while(pSerialThreadRunning)
			{
				usleep(1000);  //wait 1 millisecond
			}
			log("Stopping Button thread.\n");
			while(pIOThreadRunning)
			{
				usleep(1000); //wait 1 millisecond
			}
		}
		if(serialthread)
		{
			serialthread->join();  //wait for it to complete.
			delete serialthread;
		}
		
		if(iothread)
		{
			iothread->join();  //wait for it to complete.
			delete iothread;
		}
		
		serialthread = 0;
		iothread = 0;
	}
	catch (const std::exception& e)
	{
		log("Error. stopThread() Caught exception: %s\n", e.what());
	}
	
	log("Threads stopped.\n");
	return CNC_OK;
}

//Note: This function will block for up to 1 second while it waits for the thread to open the file.
int CNCSerial::receiveFile(char *filename)
{
	Timers timeout;
	
	if(!pReceivingFile )
	{
		strcpy(inFileName, filename);
		pStartReceivingFile = 1;
		pStopReceivingFile = 0;
		if(!pSerialThreadRunning)
		{
			log("Warning: set to recieve file, but serial thread is not running. Call startThread();\n");
		}		
		else
		{
			timeout.Start();
			//wait for the serial thread to open the file and start recieving.
			while(!pReceivingFile)
			{
				//delay here, but include a timeout
				usleep(1000); //wait a ms
				if(timeout.Expired(TIMER_1S_TIMEOUT))
				{
					log("Error: Timeout (%dms) waiting for serial thread to start receiving file.\n", TIMER_1S_TIMEOUT);
					
					return CNC_TIMEOUT;
				}
			}
		}
		return CNC_OK;
	}
	else
	{
		log("Error: receiveFile() called, but a file is already being received. Call stopReceiveFile() first.\n");
	}

	return CNC_ERROR;
}


//Note: This function will block for up to 1 second while it waits for the thread to close the file.
void CNCSerial::stopReceiveFile()
{
	Timers timeout;
	if(pReceivingFile)
	{
		
		log("Stopping file receive.\n");
		pStopReceivingFile = 1;
					
		timeout.Start();
		//wait for the serial thread to stop recieving.
		while(pReceivingFile)
		{
			usleep(1000); //wait a ms
			//delay here, but include a timeout
			if(timeout.Expired(TIMER_1S_TIMEOUT))
			{
				log("Error: Timeout (%dms) waiting for serial thread to stop receiving file.\n", TIMER_1S_TIMEOUT);
				return;
			}
		}
	}
	else
	{
		log("Warning: stopReceivFile() called, but a file is not being received.\n");
	}

}

//Note: This function will block for up to 1 second while it waits for the thread to close the file.
void CNCSerial::stopSendFile()
{
	Timers timeout;
	if(pSendingFile)
	{
		log("Stopping file send.\n");
		pStopSendingFile = 1;
					
		timeout.Start();
		//wait for the serial thread to open the file and start recieving.
		while(pSendingFile)
		{
			//delay here, but include a timeout
			usleep(1000); //wait a ms
			if(timeout.Expired(TIMER_1S_TIMEOUT))
			{
				log("Error: Timeout (%dms) waiting for serial thread to stop sending file.\n", TIMER_1S_TIMEOUT);
				return;
			}
		}
	}
	else
	{
		log("Warning: stopSendFile() called, but a file is not being sent.\n");
	}
}

//Note: This function will block for up to 1 second while it waits for the thread to open the file.
int CNCSerial::sendFile(char *filename)
{		
	Timers timeout;
	if(!pSendingFile )
	{

		log("Send File: %s\n", filename);
		strcpy(outFileName, filename);
		
		pStartSendingFile = 1;
		pStopSendingFile = 0;
		
		if(!pSerialThreadRunning)
		{
			log("Warning: set to send file, but serial thread is not running. Call startThread() first.\n");
		}		
		else
		{
			timeout.Start();
			//wait for the serial thread to open the file and start recieving.
			while(!pSendingFile)
			{
				//delay here, but include a timeout
				usleep(1000); //wait a ms
				if(timeout.Expired(TIMER_1S_TIMEOUT))
				{
					log("Error: Timeout (%dms) waiting for serial thread to start sending file.\n", TIMER_1S_TIMEOUT);				
					return CNC_TIMEOUT;
				}
			}
		}
		return CNC_OK;
	}

	log("Error: sendFile() called, but a file is already being sent. Call stopSendFile() first.\n");
	return CNC_ERROR;  //file is already sending..

}

int CNCSerial::sendString(char *str)
{		
	Timers timeout;
	if(!pSendingFile )
	{
		log("Send String: [%s]\n", str);

		if(!pSerialThreadRunning)
		{
			log("Warning: set to send string, but serial thread is not running. Call startThread() first.\n");
		}		
		else
		{
			//Simply add the string to the output queue
			if(txData.putData((unsigned char *)str, strlen(str)) == CIRC_QUEUE_FULL)  //however many bytes we have, put them on the queue.
			{
				log("Error. sendString() filled up TX queue.\n");
			}
		}
		return CNC_OK;
	}
	log("Error: sendString() called, but a file is already being sent. Call stopSendFile() first.\n");
	return CNC_ERROR;  //file is already sending..
}



/*
void set_blocking (int fd, int should_block)  //if true, disables the port from reading input bytes
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                error ("error %d getting term settings set_blocking", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = should_block ? 5 : 0; // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                error ("error setting term %sblocking", should_block ? "" : "no");
}
* */

void CNCSerial::setLogMode( int value )
{
	pLogMode = value;
}

void CNCSerial::reopen()
{
	pReopen = 1;
}

//*****************************************************************************************************
//									Private Functions
//*****************************************************************************************************
int CNCSerial::serialOpen() // Open the serial port in read-write mode
{
	speed_t myBaud ;
	struct termios options;
	//close(serial_fd);  //just in case the user calls open twice
	
	if(strstr(pSerialName, "/dev/") == 0)
	{
		log("Serial Name is not valid. Defaulting.\n");
		strcpy( pSerialName, SERIAL_PORT);		
	}
	
	if(serial_fd != 0)
	{
		log("serialOpen: Closing Serial Port.\n");
		
		close(serial_fd);
		serial_fd = 0;
		usleep(100);  //wait 10uS
	}
		
    serial_fd = open(pSerialName, O_RDWR | O_NOCTTY | O_NONBLOCK);// | O_NDELAY
    if (serial_fd < 0) 
	{
        //perror("Error opening serial port");
		log("Error opening serial port.\n");
        //exit(EXIT_FAILURE);
		//open = 0;
		return CNC_ERROR;
    }

	pUserPaused = 0;
	pMachinePaused = 0;
	
	//now set up the port
	
	// Acquire non-blocking exclusive lock
    //if(flock(serial_fd, LOCK_EX | LOCK_NB) == -1) {
        //throw std::runtime_error("Serial port with file descriptor " + 
         //   std::to_string(fd) + " is already locked by another process.");
	//		log("Serial port is already locked by another process.\n");
    //}

	log("Starting Serial [%s]: %d baud, ", pSerialName, pBaudRate);
	
	if(pParity == PARITY_EVEN)
	{
		log("EVEN Parity, ");
	}
	else if(pParity == PARITY_ODD)
	{
		log("ODD Parity, ");
	}
	else if(pParity == PARITY_NONE)
	{
		log("NO Parity, ");
	}
	
	if(pStopBits == STOP_BITS_1)
	{
		log("One Stop bit, ");
	}
	else if(pStopBits == STOP_BITS_2)
	{
		log("Two Stop bits, ");
	}
	
	if(pFlowControl == FLOW_CONTROL_SOFTWARE)
	{
		log("Software Flow Control\n");
	}
	else if(pFlowControl == FLOW_CONTROL_HARDWARE)
	{
		log("Hardware Flow Control\n");
	}
	else if(pFlowControl == FLOW_CONTROL_GRBL)
	{
		log("GRBL Flow Control\n");
	}
	else if(pFlowControl == FLOW_CONTROL_NONE)
	{
		log("No Flow Control\n");
	}
		
	 
	// Set the serial port parameters
	tcgetattr(serial_fd, &options);
	
	//log("the end-of-file character is x'%02x'\n", options.c_cc[VEOF]);
	
	switch (pBaudRate)
	  {
		case      50:	myBaud =      B50 ; break ;
		case      75:	myBaud =      B75 ; break ;
		case     110:	myBaud =     B110 ; break ;
		case     134:	myBaud =     B134 ; break ;
		case     150:	myBaud =     B150 ; break ;
		case     200:	myBaud =     B200 ; break ;
		case     300:	myBaud =     B300 ; break ;
		case     600:	myBaud =     B600 ; break ;
		case    1200:	myBaud =    B1200 ; break ;
		case    1800:	myBaud =    B1800 ; break ;
		case    2400:	myBaud =    B2400 ; break ;
		case    4800:	myBaud =    B4800 ; break ;
		case    9600:	myBaud =    B9600 ; break ;
		case   19200:	myBaud =   B19200 ; break ;
		case   38400:	myBaud =   B38400 ; break ;
		case   57600:	myBaud =   B57600 ; break ;
		case  115200:	myBaud =  B115200 ; break ;
		case  230400:	myBaud =  B230400 ; break ;
		case  460800:	myBaud =  B460800 ; break ;
		case  500000:	myBaud =  B500000 ; break ;
		case  576000:	myBaud =  B576000 ; break ;
		case  921600:	myBaud =  B921600 ; break ;
		case 1000000:	myBaud = B1000000 ; break ;
		case 1152000:	myBaud = B1152000 ; break ;
		case 1500000:	myBaud = B1500000 ; break ;
		case 2000000:	myBaud = B2000000 ; break ;
		case 2500000:	myBaud = B2500000 ; break ;
		case 3000000:	myBaud = B3000000 ; break ;
		case 3500000:	myBaud = B3500000 ; break ;
		case 4000000:	myBaud = B4000000 ; break ;

		default:
		{
		  myBaud =    B9600 ;
		  log("Error. Bad Baud Rate: %d. Defaulting to 9600.\n", pBaudRate); 
		}break ;
	  }	
	
	
	cfsetispeed(&options, myBaud);   // Set the baud rate
	cfsetospeed(&options, myBaud);		
	

	
	options.c_cflag |= (CLOCAL | CREAD);  //CLOCAL: Ignore modem control lines.  CREAD enables RX 
	options.c_cflag &= ~CSIZE;		//zero out the size mask
	
	if(pDataBits == 5)
	{
		options.c_cflag |= CS5;
	}
	else if(pDataBits == 6)
	{
		options.c_cflag |= CS6;
	}
	else if(pDataBits == 7)
	{
		options.c_cflag |= CS7;			//set the number of data bits (Values are CS5, CS6, CS7, or CS8)
	}
	else //if(pDataBits == 8)
	{
		options.c_cflag |= CS8;
	}

		
	if(pParity)
	{
		options.c_cflag |= PARENB;		//Enable parity generation on output and parity checking for input
		if(pParity == PARITY_ODD)
		{
			options.c_cflag |= PARODD;		//Set flag is Odd Partiy
		}
		else if(pParity == PARITY_EVEN)
		{
			options.c_cflag &= ~PARODD;		//Clear flag is Even Parity
		}
	}
	else
	{
		options.c_cflag &= ~PARENB;		//turn off parity checking		
	}
	
	
	if(pStopBits == STOP_BITS_2)
	{
		options.c_cflag |= CSTOPB;		//Set flag means two stop bits, rather than one.
	}
	else if(pStopBits == STOP_BITS_1)
	{
		options.c_cflag &= ~CSTOPB;		//clear flag means 1 stop bit
	}
	else
	{
		log("Error: Stop bits are not set before calling open()");
		options.c_cflag &= ~CSTOPB;		//clear flag means 1 stop bit
	}
	
	if(pNewFlowControl != pFlowControl)
		pFlowControl = pNewFlowControl;
		
	//FIXME: even in SW flow control, we need to be able to set RTS...

    if(pFlowControl == FLOW_CONTROL_HARDWARE)
	{
		options.c_cflag |= CRTSCTS;		//Enable RTS/CTS (hardware) flow control.
	}
	else //if((pFlowControl == FLOW_CONTROL_SOFTWARE) || (pFlowControl == FLOW_CONTROL_NONE) || (pFlowControl == FLOW_CONTROL_GRBL))
	{
		options.c_cflag &= ~CRTSCTS;     // Disable hardware flow control
	}

	//configure for no blocking reads
	options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
	
	//inputs
	options.c_iflag &= ~(IXON | IXOFF | IXANY);  //disable XON/XOFF on output
	options.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
	
	//
	options.c_lflag &= ~ICANON;  //disable conical mode (waits for newline to send or receive)
	options.c_lflag &= ~ECHO; // Disable echo
	options.c_lflag &= ~ECHOE; // Disable erasure
	options.c_lflag &= ~ECHONL; // Disable new-line echo
	options.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

	//outputs
	options.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	options.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
// options.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
// options.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)

	
	tcsetattr(serial_fd, TCSANOW, &options);  //set the new options immediately (don't wait for TX/RX completion)	
	
	resetStats();
	return CNC_OK;
}



int CNCSerial::isSerialWriteable()	//returns 1 when file descriptor is writeable
{
	// select() on the serial port file descriptor to wait for it to be writeable.
    // It never does become writeable. Removing this section does not change
    // the behavior of the following call to write() 
    fd_set wfds;
    struct timeval tv;
    int select_retval;
    FD_ZERO(&wfds);
    FD_SET(serial_fd, &wfds);
    tv.tv_sec = 5;
    tv.tv_usec = 0;
    select_retval = select(serial_fd + 1, NULL, &wfds, NULL, &tv);
    if (select_retval == -1)
    {
        log("select: error");
        return CNC_ERROR;
    }
    else if (select_retval)
    {
        //log("select: fd can be written now\n");
		return CNC_OK;
    }
    else
    {
        //log("select: fd did not become writeable within 5 seconds\n");
		return CNC_TIMEOUT;
    }
	return CNC_ERROR;  //just here to keep the compiler happy
	
}    


//FIXME: should this return a value?  No.
int CNCSerial::actualGetCTS()
{
	int status;
	if(serial_fd > 0)
	{
		ioctl(serial_fd, TIOCMGET, &status);
		if (status & TIOCM_CTS) 
		{
			//log("CTS is set.\r\n");
			pClearToSend = 1;
		}
		else    //CTS is clear, don't bother sending more bytes
		{
			pClearToSend = 0;
		}
	}
	else
		pClearToSend = 0;
	
	return pClearToSend;
}

int CNCSerial::actualSetRTS(unsigned short level)  //Private
{
	int status;
	
	if (serial_fd < 0) 
	{
		log("Error: Invalid File descriptor in actualSetRTS().\n");
		return CNC_ERROR;
	}

	if (ioctl(serial_fd, TIOCMGET, &status) == -1) 
	{
		log("Error: actualSetRTS(): TIOCMGET\n");
		return CNC_ERROR;
	}

	if (level) 
		status |= TIOCM_RTS;
	else 
		status &= ~TIOCM_RTS;

	if (ioctl(serial_fd, TIOCMSET, &status) == -1) 
	{
		log("Error: set_RTS(): TIOCMSET\n");
		return CNC_ERROR;
	}
	log("Setting RTS to: %d\n", level);
	pRTS = level;
	
	return CNC_OK;
}

int CNCSerial::actualSetDSR(unsigned short level)  //Private
{
	int status;
	
	if (serial_fd < 0) 
	{
		log("Error: Invalid File descriptor in actualSetDSR().\n");
		return CNC_ERROR;
	}

	if (ioctl(serial_fd, TIOCMGET, &status) == -1) 
	{
		log("Error: actualSetRTS(): TIOCMGET\n");
		return CNC_ERROR;
	}

	if (level) 
	{
		status |= TIOCM_DSR;
		status |= TIOCM_LE;  //line enable is DSR
	}
	else 
	{
		status &= ~TIOCM_DSR;
		status &= ~TIOCM_LE;
	}

	if (ioctl(serial_fd, TIOCMSET, &status) == -1) 
	{
		log("Error: set_DSR(): TIOCMSET\n");
		return CNC_ERROR;
	}
	log("Setting DSR to: %d\n", level);
	pDSR = level;
	
	return CNC_OK;
}


int CNCSerial::actualSetFlowControl(FlowControlValues value)  //FLOW_CONTROL_SOFTWARE, FLOW_CONTROL_HARDWARE, or FLOW_CONTROL_NONE
{
	struct termios options;
	// Set the serial port parameters
	tcgetattr(serial_fd, &options);

	//FIXME: add FLOW_CONTROL_NONE
	if(value == FLOW_CONTROL_SOFTWARE)  //software set
	{
		//FIXME: Test with new board to ensure we still have control of RTS in SW mode
		//options.c_cflag &= ~CRTSCTS;     // Disable hardware flow control
		//options.c_iflag |= (IXON | IXOFF | IXANY);  //enable XON/XOFF on output  FIXME: may not need IXOFF or IXANY
	}
	else if(value == FLOW_CONTROL_GRBL)
	{
		//Do nothing.  We only receive flow control 'ok\r\n' or 'error\r\n'
	}
	else if(value == FLOW_CONTROL_HARDWARE)
	{
		options.c_cflag |= CRTSCTS;		//Enable RTS/CTS (hardware) flow control
		options.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
	}
	else if(value == FLOW_CONTROL_NONE)
	{
		//Do nothing
	}
	pFlowControl = value;
    // Get the serial port parameters
	tcsetattr(serial_fd, TCSANOW, &options);  //set the new options immediately (don't wait for TX/RX completion)
	return CNC_OK;
}

int CNCSerial::actualSetDTR( unsigned short level)  //0 or non-zero
{
	int status;


	if (serial_fd < 0) 
	{
		log("Error: SetDTR(): Invalid File descriptor\n");
		return CNC_ERROR;
	}

	if (ioctl(serial_fd, TIOCMGET, &status) == -1) 
	{
		log("Error: setDTR(): TIOCMGET\n");
		return CNC_ERROR;
	}

	if (level) 
		status |= TIOCM_DTR;
	else 
		status &= ~TIOCM_DTR;

	if (ioctl(serial_fd, TIOCMSET, &status) == -1) 
	{
		log("Error: setDTR(): TIOCMSET\n");
		return CNC_ERROR;
	}	
	
	log("Setting DTR to: %d\n", level);
	pDTR = level;
	return CNC_OK;

}


__attribute__((weak)) int CNCSerial::ioPollingThread()
{
	SerialLEDStates LED_state;
	Timers led_timer;
	int led_toggle;
	
	int start_pause_counter = 0;
	pStartPauseButton = 1;
	int last_start_pause_button = 1;
	
	int stop_counter = 0;
	pStopButton = 1;
	int last_stop_button = 1;

	led_timer.Start();
	
	try
	{
		
		//initialize the GPIO
		/////////
		wiringPiSetup();  //this sets up the GPIO driver using pin numbers, not GPIO numbers.
		pinMode(PAUSE_LED_PIN, OUTPUT);
		pinMode(SERIAL_STARTPAUSE_PIN, INPUT);
		pinMode(SERIAL_STOP_PIN, INPUT);
		//////////

		pIOThreadRunning = 1;
		while(!pDone)
		{
			usleep(10);  //wait 10uS
			
			if(pUserPaused)
				LED_state = SERIAL_LED_BLINKING;
			else if(!pClearToSend)
				LED_state = SERIAL_LED_ON;
			else
				LED_state = SERIAL_LED_OFF;
			
			
			//*************************************************************************************
			//manage button inputs
			//*************************************************************************************
			
			//These inputs are pulled up and ground activated.
			if(!digitalRead(SERIAL_STARTPAUSE_PIN))
			{
				start_pause_counter++;
				if(start_pause_counter >= DEBOUNCE_MAX_COUNT)
				{
					start_pause_counter = DEBOUNCE_MAX_COUNT;
					pStartPauseButton = 1;
				}
			}
			else
			{
				start_pause_counter--;
				if(start_pause_counter <= 0)
				{
					start_pause_counter = 0;
					pStartPauseButton = 0;
				}
			}
			if((pStartPauseButton == 1) && (last_start_pause_button == 0))
			{

				log("SingleStep: %d\n", pSingleStep);
				if(pSingleStep)
				{
					pDoStep = 1;
					pUserPaused = 0;  //we are doing single step, so shut off the paused.
				}
				else
				{
					//active edge!  toggle the pUserPaused flag
					pUserPaused++;
					pUserPaused &= 0x01;
					
					log("Start/Pause button pressed!  pUserPaused=%d\n", pUserPaused);
					
					pStartPauseButtonFlag = 1;  //handle this in the Serial Thread
					
				}
			}
			else if((pStartPauseButton == 0) && (last_start_pause_button == 1))
			{
				//start pause button was released
			}

			
			if(!digitalRead(SERIAL_STOP_PIN))
			{
				stop_counter++;
				if(stop_counter >= DEBOUNCE_MAX_COUNT)
				{
					stop_counter = DEBOUNCE_MAX_COUNT;
					pStopButton = 1;
				}
			}
			else
			{
				stop_counter--;
				if(stop_counter <= 0)
				{
					stop_counter = 0;
					pStopButton = 0;
				}
			}
			if((pStopButton == 1) && (last_stop_button == 0))
			{
				//active edge!  set flags to stop all transmission and reception
				pStopButtonFlag = 1;		
				//handle this in the Serial Thread
			}
			else if((pStopButton == 0) && (last_stop_button == 1))
			{
				//stop button was released.
			}

			last_start_pause_button = pStartPauseButton;
			last_stop_button = pStopButton;
			
			//*************************************************************************************
			//LED output
			//*************************************************************************************		
			
			if(LED_state == SERIAL_LED_BLINKING)
			{
				if(led_timer.Expired(SERIAL_BLINK_DELAY))  //ms
				{
					led_timer.Start();
					
					//pinMode(PAUSE_LED_PIN,OUTPUT);
					if((led_toggle++ & 0x01) == 0x01)
					{
						digitalWrite(PAUSE_LED_PIN, LOW);
					}
					else
					{
						digitalWrite(PAUSE_LED_PIN, HIGH);
					}
				}
			}
			else if(LED_state == SERIAL_LED_ON)	//LED is pulled up. GPIO high = LED ON
			{
				pinMode(PAUSE_LED_PIN,OUTPUT);
				digitalWrite(PAUSE_LED_PIN, HIGH);
			}
			else  //LED_state == SERIAL_LED_OFF
			{
				pinMode(PAUSE_LED_PIN,OUTPUT);
				digitalWrite(PAUSE_LED_PIN, LOW);
			}
			
			
		}
	}
	catch (const std::exception& e)
	{
		log("Error. Button Thread caught exception: %s\n", e.what());
	}	
		
	pIOThreadRunning = 0;
	return CNC_OK;
}

int CNCSerial::serialPollingThread()
{
	//Note: this is the only function in the entire class that has the ability to manipulate the serial port.
	//there are other functions that change the port settings, but they are only called by this function.
	//No public function can actually change anything on the port.
	
	//This is also the only function that can manipulate a file for reading or saving data.
	int in_file_fd = 0;  //for receiving a file
	int out_file_fd = 0;  //for transmitting file
	Timers delay_timer;
	Timers GRBLtimeout;

	
	int end_of_file = 0;
	
	int rx_data_active = 0;  //gets set when we receive a 0x12 (Start of Data) or when  we get StartStopChar
	
	

	int tx_log_fd = 0;
	if(pLogTx)
	{
		tx_log_fd = open("./Logs/TXlog.txt", O_CREAT|O_RDWR, 0666);  //read and write permissions (octal)
		if (tx_log_fd <= 0) 
		{
			log("Error opening %s to log. Does it exist already?\n", "TXlog.txt");
			log("Error %d, description is : %s\n",errno, strerror(errno));
		}
	}
	
	int rx_log_fd = 0;
	if(pLogRx)
	{
		rx_log_fd = open("./Logs/RXlog.txt", O_CREAT|O_RDWR, 0666);  //read and write permissions (octal)
		if (rx_log_fd <= 0) 
		{
			log("Error opening %s to log. Does it exist already?\n", "RXlog.txt");
			log("Error %d, description is : %s\n",errno, strerror(errno));
		}	
		//else if(rx_log_fd > 0)
		//{
		//	write(rx_log_fd, "RXLog\n", 6);  //write something so we know the log is working.
		//}
	}
		


		
	int num_bytes;
	
	try
	{
			
		if(serialOpen() == CNC_ERROR)
		{
			log("Error: Unable to open serial port.\n");
			return CNC_ERROR;
		}
		tcflush(serial_fd, TCIFLUSH); //read any input bytes off the port and discard them
		
		actualSetFlowControl(pFlowControl);
		
		delay_timer.Start();
		GRBLtimeout.Start();

		actualSetFlowControl( pNewFlowControl );
		actualSetDTR( pNewDTR );
		actualSetRTS( pNewRTS );
		actualSetDSR( pNewDSR );
		
		pGRBLReplyIndex = 0;
		pGRBLNextFlag = 1;  //make it OK to send a GRBL command.
				
		
		pSerialThreadRunning = 1;
		while(!pDone)
		{

			usleep(10);  //wait 10uS
			//log("*");
			
			if(pReopen == 1)
			{
				pReopen = 0;
				if(serialOpen() == CNC_ERROR)
				{
					log("Error: Unable to open serial port.\n");					
				}
			}
			if(pFlowControl == FLOW_CONTROL_GRBL)
			{
				if(GRBLtimeout.Expired(10000))  //10 seconds
				{
					log("GRBL 10s timeout. Enabling TX again.\n");
					pGRBLNextFlag = 1;  //make it OK to send a GRBL command. again.
					GRBLtimeout.Start();
				}
			}
			//*************************************************************************************
			//manage Port settings 
			//*************************************************************************************
			if(pNewFlowControl != pFlowControl)
			{
				actualSetFlowControl( pNewFlowControl );
			}
			else if(pNewDTR != pDTR)
			{
				actualSetDTR( pNewDTR );
			}
			else if(pNewRTS != pRTS)
			{
				actualSetRTS( pNewRTS );
			}
			else if(pNewDSR != pDSR)
			{
				actualSetDSR( pNewDSR);
			}
			
				
			if(serial_fd == 0)
				continue;
		
			if(pStopButtonFlag)
			{
				pStopButtonFlag = 0;
				
				pStopSendingFile = 1;
				pStopReceivingFile = 1; 
				log("Stop button pressed! Stopping sending and receiving.\n");
			}
			if(pStartPauseButtonFlag)
			{
				pStartPauseButtonFlag = 0;
				
				if(pSingleStep)
				{
					pDoStep = 1;
				}
				else
				{
					if(pUserPaused == 0)
						pUserPaused = 1;
					else
						pUserPaused = 0;
				
					
					if(!pSendingFile && (pFlowControl == FLOW_CONTROL_SOFTWARE))  
					{
						unsigned char fc[3] = { XON, XOFF, 0x00 };
						printf("Sending Flow control: %x\n", fc[pUserPaused]);

						if(pUserPaused == 1)
							txData.putData(&fc[1], 1);
						else
							txData.putData(&fc[0], 1); 
					}
					if(pFlowControl == FLOW_CONTROL_HARDWARE)
					{
						if(pUserPaused == 1)
							setRTS(0);
						else
							setRTS(1);
					}				
				}	
			}
			
			//*************************************************************************************
			//manage RX 
			//*************************************************************************************
			num_bytes = rxBytesWaiting();
			if(num_bytes > 0)
			{
				//num_bytes = receive( inbuf, sizeof(inbuf));  //this already handles SW flow control
				num_bytes = read(serial_fd, inbuf, sizeof(inbuf));
				
				pBytesReceived += num_bytes;
				
				char str[50];
				
				//Handle all Flow Control Bytes before queuing up the data
				int count = 0;
				for(int n = 0; n < num_bytes; n++)
				{
					
					if(pLogRx && (rx_log_fd != 0))
					{
						sprintf(str, "RX: %02x\n", inbuf[n]);
						write(rx_log_fd, str, 8);
					}
					
					switch(inbuf[n])
					{
						case 0x00: //only add 0x00 to queue if ignoreNULL is 0.
						{
							if((!pIgnoreNULL) && (rx_data_active))
								inbuf2[count++] = inbuf[n];
						}break;
						case XON:  //DC1 (sent by CNC to start data xfer)
						{

							if(pFlowControl == FLOW_CONTROL_SOFTWARE)
							{
								pClearToSend = 1;
							} 
							else if(pFlowControl == FLOW_CONTROL_HARDWARE)
							{
								pMachinePaused = 0;	
							}
							if((pUseRxFlowControl == 0) && (pReceivingFile == 1))
							{
								log("Warning: Use RX Flow control disabled, but the machine sent SW flow control bytes");
							}
							
						}break;
						case XOFF:  //DC3 (sent by CNC to pause data xfer)
						case XOFF2: 
						{
							if(pFlowControl == FLOW_CONTROL_SOFTWARE)
							{
								pClearToSend = 0;
							}
							else if(pFlowControl == FLOW_CONTROL_HARDWARE)  //FIXME: does this make sense?
							{
								pMachinePaused = 1;
							}
							if((pUseRxFlowControl == 0) && (pReceivingFile == 1))
							{
								log("Warning: Use RX Flow control disabled, but the machine sent SW flow control bytes");
							}
						}break;
						case DC2:  //start of data
						{
							if(pReceivingFile && !pUseStartStopChar)
							{
								rx_data_active = 1;
								log("Recieved SW Start byte. Receiving File.");
							}
							
						}break;
						case DC4:  //end of data
						{
							if(pReceivingFile)
							{
								rx_data_active = 0;
								pStopReceivingFile = 1;
							}
						}break;
						//NAK is sent when there is an Alarm during the transfer
						case NAK:
						case NAK2: 
						{
							pStopSendingFile = 1; 
						}break;
						//SYN is sent when there is a Reset during the transfer
						case SYN:
						case SYN2: 
						{
							pStopSendingFile = 1; 
						}break;
						default:  //everything else
						{
							if(pUseStartStopChar)
							{
								if(inbuf[n] == pStartStopChar)
								{
									if(rx_data_active == 1)
									{
										rx_data_active = 0;
										inbuf2[count++] = inbuf[n];  //save the last char to the file, typically '%'
										pStopReceivingFile = 1;
										log("Got second StartStopChar. Stopping RX.");
									}
									else
									{
										rx_data_active = 1;
										log("Received start char. Receiving File.");
									}
								}
							}
							if(pFlowControl == FLOW_CONTROL_GRBL)
							{
								printf("%c", inbuf[n]);
								if(inbuf[n] == '\n')
								{
									if(pGRBLReplyIndex >= 2)
									{
										//check for 'ok\r\n'
										if((pGRBLReply[pGRBLReplyIndex-2] == 'o') && (pGRBLReply[pGRBLReplyIndex-1] == 'k'))
										{											
											pGRBLNextFlag = 1;
											printf("OK!\n");
										}
										//check for 'ok\n'
										else if((pGRBLReply[pGRBLReplyIndex-1] == 'o') && (pGRBLReply[pGRBLReplyIndex] == 'k'))
										{											
											pGRBLNextFlag = 1;
											printf("OK!\n");
										}
										else
										{
											printf("ERROR\n");
											//we must have gotten an error..
											pGRBLReply[pGRBLReplyIndex++] = '\0';
											log(pGRBLReply);  //
										}
									}
									//print this out for initial developement testing
									pGRBLReply[pGRBLReplyIndex++] = '\0';
									log("GRBL Reply: %s\n", pGRBLReply);  //
									pGRBLReplyIndex = 0;
								}
								else
								{
									pGRBLReply[pGRBLReplyIndex++] = inbuf[n];
								}
							}
							
							if(rx_data_active)
								inbuf2[count++] = inbuf[n];

						}break;
					}
				}//end for	
				//log("Received: %d\n", num_bytes);
				//add inbuf data to the circular queue
				if(count > 0)
				{
					if(rxData.putData(inbuf2, count) == CIRC_QUEUE_FULL)
					{
						log("Error. RX Queue full.\n");
					}
				}
				
				//TODO:  if Software flow control and our buffers filled up, send stop byte (pXOFFByte)
				//TODO:  if software flow control and stopped and buffers empty, send start byte (DC1)
			}
			
			//*************************************************************************************
			//Handle Receiving File Queue if needed
			//*************************************************************************************

			if(pStartReceivingFile)  //we would like to open a file and start sending it.
			{
				pStartReceivingFile = 0; //turn off the flag since we are servicing it.
				in_file_fd = open(inFileName, O_CREAT|O_RDWR, 0666);  //read and write permissions (octal)
				if (in_file_fd <= 0) 
				{
					log("Error opening %s to receive. Does it exist already?\n", inFileName);
					log("Error %d, description is : %s\n",errno, strerror(errno));
				}
				else
				{
					setRTS( 0 );  //FIXME: why 0? tell the machine we are ready to receive data.
					log("Starting Receiving.\r\n");
					//write(in_file_fd, "FirstLine.\n", 11);  //write this to test the file
					pReceivingFile = 1;
					
					if(pUseRxFlowControl)
					{
						if(pFlowControl == FLOW_CONTROL_SOFTWARE)
						{
							sendByte( XON );
							//unsigned char byte = XON;
							//write(serial_fd, &byte, 1);
							//tcdrain(serial_fd);
							log("Receiving file with Software Flow Control.");
						}
						else if(pFlowControl == FLOW_CONTROL_GRBL)
						{
							if(!pUseStartStopChar)
							{
								rx_data_active = 1;
							}
							log("Receiving file with GRBL Flow Control.");
						}
						else if(pFlowControl == FLOW_CONTROL_HARDWARE)
						{
							if(!pUseStartStopChar)  
							{
								rx_data_active = 1;
							}
							log("Receiving with Hardware Flow Control.");
							
						}
						else //FLOW_CONTROL_NONE
						{
							if(!pUseStartStopChar)
							{
								rx_data_active = 1;
							}
							log("Receiving file using no Flow Control.");
							
						}
					}
					else
					{
						if(!pUseStartStopChar)  //if not using the character
						{
							rx_data_active = 1;
							log("Not using RX Flow Control. Receiving now.");
						}
						//if we are using the start stop char, the rx_data_active will get set when it is received.
					}
					
				}			
			}
	
			if(pReceivingFile)
			{
				if(!rxData.isEmpty())
				{
					if(in_file_fd > 0)
					{
						//get the data off the queue
						int num_to_write = rxData.getData( inbuf, sizeof( inbuf ) );

						//write it to the file
						if(num_to_write > 0)
						{
							int i = write(in_file_fd, inbuf, num_to_write);
							if(i != num_to_write)
							{
								log("Error. Tried to write %d bytes from Rx queue to the file, but only %d bytes written.\n", num_to_write, i);
							}
						}
						else
						{
							log("Error. Rx Queue was not empty but we could not get the data.\n");
							log("rxData: "); rxData.printStatus();
							log("Error %d, description is : %s\n",errno, strerror(errno));
						}
						
					}
					else
					{
						log("Warning: received %d bytes with nowhere to put it.\n", rxData.getSize());
					}
				}
			}//end if receiving data
						
			
			if(pStopReceivingFile)
			{
				//be sure to only close the file after we save everything in the queue. 
				if(rxData.isEmpty())
				{
					pStopReceivingFile = 0;
					log("Stopping recieve file.\n");
					if(in_file_fd > 0)
					{
						close(in_file_fd);
						in_file_fd = 0;
					}
					pReceivingFile = 0;
				}
			}
				
			
			//*************************************************************************************
			//Handle Sending File TX Queue if needed
			//*************************************************************************************
			
			if(pStartSendingFile)  //we would like to open a file and start sending it.
			{
				pStartSendingFile = 0; //turn off the flag since we are servicing it.
				
					//first, find the file size
				//FILE *fileforsize;
				//fileforsize = fopen(outFileName, O_RDONLY);
				//fseek(fileforsize, 0L, SEEK_END);
				//pFileSize = ftell(fileforsize);
				//if(fileforsize)
				//	fclose(fileforsize);
					
				log("Opening: %s\n", outFileName);
				out_file_fd = open(outFileName, O_RDONLY);
				
				//struct stat stat_buf;
				//int rc = fstat(out_file_fd, &stat_buf);
				//if(rc == 0)
				//	pFileSize = stat_buf.st_size;
				//else
				//	pFileSize = 0;
					
				//count the number of lines in the file by iterating through the file and counting the '\n' characters.
				pNumLines = 0;
				pFileSize = 0;
				unsigned char c;
				int ret = read(out_file_fd, &c, 1);
				while(c != EOF)
				{
					if(c == '\n')
						pNumLines++;
					pFileSize++;
					
					ret = read(out_file_fd, &c, 1);
					if(ret == 0)
					   break;
				}
				lseek(out_file_fd, 0L, SEEK_SET);  //go back to the beginning.
				
				
				log("File Descriptor: %d, Size:%d, Lines:%d\n", out_file_fd, pFileSize);
				if (out_file_fd <= 0) 
				{
					log("Error opening %s to send. Does it exist and have read permissions?\n", outFileName);
				}
				else
				{
					pBytesSent = 0;
					pCurrentLine = 0;
		
					setRTS( 1 );  //tell the machine we are ready to send data.
					log("Starting Transfer.\r\n");
					pSendingFile = 1;
					
					if(pFlowControl == FLOW_CONTROL_SOFTWARE)
					{
						pClearToSend = 1;  //default to sending if sw handshaking
						//SW handshaking requires a control byte at the start and end
						sendByte( DC2 );  //DC2
					}
				}			
			}
			
			if(pStopSendingFile)  //this is set by the user
			{
				pStopSendingFile = 0;
				
				log("Stopping send file.\n");
				if(out_file_fd > 0)
				{
					close(out_file_fd);
					out_file_fd = 0;
				}
				pUserPaused = 0;
				//wait for TX queue to empty before clearing sending flag
				txData.reset();  //immediately stop sending data by clearing the TX queue
				end_of_file = 1;
			}
			
			if(end_of_file && txData.isEmpty())
			{
				if(pFlowControl == FLOW_CONTROL_SOFTWARE)
				{
					sendByte( DC4 );
				}
				end_of_file = 0;
				log("File Sent Completely.\r\n");
								
				if(out_file_fd > 0)
				{
					close(out_file_fd);
					out_file_fd = 0;
				}
				pSendingFile = 0;
			}
			
			if(pSendingFile && pClearToSend && (!pUserPaused) && (!pMachinePaused) )  //if we are sending a file and not held up by flow control
			{
				//if(!file_fd)
				//{
				//	log("Error. Trying to send, but file_fd is zero.\r\n");
				//}
				
				if((out_file_fd > 0) && (txData.spaceLeft() > sizeof(outbuf)))	//if we have room in the circular queue
				{
					int i = read(out_file_fd, outbuf, sizeof(outbuf));
					if(i > 0)
					{
						if(txData.putData(outbuf, i) == CIRC_QUEUE_FULL)  //however many bytes we read, put them on the queue.
						{
							log("Error. filled up TX queue.\n");
						}
						
						if((unsigned int)i < sizeof(outbuf))
						{
							//we read to the end of the file.
							end_of_file = 1;
						}
						
						//#ifdef DEBUG
						//write(tx_log_fd, outbuf, i);
						//#endif
					}
					else if(i == -1)
					{
						log("Error %d, description is : %s\n",errno, strerror(errno));
					}
					else if(i == 0) //when it returns zero, that is the end of the file.
					{
						end_of_file = 1;
					}
					else
					{
						log("Error reading file to send: %d, description is : %s\n",errno, strerror(errno));
					}
				}
				//wait until we empty the tx queue before we mark the file as done sending.
				if(txData.isEmpty())
				{
					pSendingFile = 0;
					//pStopSendingFile = 1;
				}
			}
			
			//*************************************************************************************
			//this is where we manage the TX data (based on flow control)
			//*************************************************************************************
			if(pFlowControl == FLOW_CONTROL_HARDWARE)
			{
				//wait for the HW flow control to signal to us that it is ready
				actualGetCTS();
			}
			else if(pFlowControl == FLOW_CONTROL_SOFTWARE)
			{
				//pClearToSend is managed inside of receive parsing
			}
			else if(pFlowControl == FLOW_CONTROL_GRBL)
			{
				//if we received 'ok\r\n' then it is OK to send the next line.
				if(pGRBLNextFlag == 1)  
				{
					//reset the timeout timer..
					GRBLtimeout.Start();
					pClearToSend = 1;
					pGRBLNextFlag = 0;
				}
				else
				{
					pClearToSend = 0;
				}
			}
			else  //no flow control
			{
				pClearToSend = 1;
			}
			
			if(pClearToSend && (!pUserPaused) && (!pMachinePaused) && !txData.isEmpty() && delay_timer.Expired(pPacketDelay))
			{
				
				int num_tx_bytes = 0;
				// read out byte by byte until we get to a '\n'
				if(pSingleStep )  //this is the machine mode. User needs to press the start button for each line.
				{
					
					if(pDoStep == 1)
					{
						pDoStep = 0;
					
						for(int n = 0; n < CNC_BUF_SIZE; n++)
						{
							if(txData.getData(&outbuf[n], 1) == 1)  //
							{
								num_tx_bytes++;
								if(outbuf[n] == '\n')
								{
									pCurrentLine++;
									n = CNC_BUF_SIZE;  //get out of the for loop
								}
							}
							else
							{
								n = CNC_BUF_SIZE;  //get out of the for loop
							}
						}
					}
					//otherwise, do nothing, effectively waiting for a button press..

				}
				else if((pPacketLength == 0) && (pPacketDelay != 0))
				{
					//if length = 0 and delay != 0, send 1 line of gCode then delay
					for(int n = 0; n < CNC_BUF_SIZE; n++)
					{
						if(txData.getData(&outbuf[n], 1) == 1)  //
						{
							num_tx_bytes++;
							if(outbuf[n] == '\n')
							{
								pCurrentLine++;
								n = CNC_BUF_SIZE;  //get out of the for loop
							}
						}
						else
						{
							n = CNC_BUF_SIZE;  //get out of the for loop
						}
					}
				}
				else  //pPacketDelay == 0 || pPacketLength != 0
				{
					int max_tx_bytes = pPacketLength;
					if(max_tx_bytes == 0)
						max_tx_bytes = CNC_BUF_SIZE;  //set to max
					//num_tx_bytes = txData.getData(outbuf, max_tx_bytes);  //request the max
					
					for(int n = 0; n < max_tx_bytes; n++)
					{
						if(txData.getData(&outbuf[n], 1) == 1)  //
						{
							num_tx_bytes++;
							if(outbuf[n] == '\n')
							{
								pCurrentLine++;
							}
						}
						else
						{
							n = CNC_BUF_SIZE;  //get out of the for loop
						}
					}

				}
				
				if(num_tx_bytes > 0)
				{
					//be careful, if we overflow the TX buffer, there is no real indication!!
					int num_written = write(serial_fd, outbuf, num_tx_bytes);
					pBytesSent += num_written;	
					
					log("%d [%s]\n", num_tx_bytes, outbuf);
					
					if(pLogTx)
						write(tx_log_fd, outbuf, num_written);
				
					if(num_written > 0) 
					{
						if(pflushRequested)
						{
							tcdrain(serial_fd);
						}
						
						if(pPacketDelay)
						{
							delay_timer.Start();
						}
					}
				}
			} 

			
			
		}  //end while !done
		
		log("Serial Thread Complete. Closing Serial Port.\n");
		
		close(serial_fd);
		serial_fd = 0;
	}
	catch (const std::exception& e)
	{
		log("Error. Serial Thread caught exception: %s\n", e.what());
	}
	

	if(tx_log_fd != 0) 
	{
		close(tx_log_fd);
		tx_log_fd = 0;
	}
	if(rx_log_fd != 0)
	{
		close(rx_log_fd);
		rx_log_fd = 0;
	}



	pSerialThreadRunning = 0;
	return CNC_OK;
	
}



void CNCSerial::log(const char *message, ...)
{
	va_list argp ;
	char buffer [1024] ;

	va_start (argp, message) ;
	vsnprintf (buffer, 1023, message, argp) ;
	va_end (argp) ;
	
	setStatus(buffer);
	

	//serialPuts (fd, buffer) ;
	if((pLogMode & LOG_MODE_PRINT) == LOG_MODE_PRINT)
	{
		printf(buffer);
		//printf("[%s]\n", buffer);
	}
	if((pLogMode & LOG_MODE_FILE) == LOG_MODE_FILE)
	{
		//
		if(log_fd <= 0)
		{
			log_fd = open("./Logs/CNCSerial.log", O_CREAT|O_RDWR, 0666);  //read and write permissions (octal)
			if (log_fd <= 0) 
			{
				printf("Error opening %s to log. Does the folder exist?\n", "CNCSerial.log");
				printf("Error %d, description is : %s\n",errno, strerror(errno));
				printf("Setting log mode to printing.\n");
				pLogMode = LOG_MODE_PRINT;
			}
		}
		if(log_fd > 0)
		{
			write(log_fd, buffer, strlen(buffer));  //write this to test the file
		}
		
	}
	if(pLogMode == LOG_MODE_NONE)
	{
		//do nothing..
	}
	
}



