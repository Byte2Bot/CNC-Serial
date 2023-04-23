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


//#define DEBUG 1

#define CNC_BUF_SIZE 256
unsigned char inbuf[CNC_BUF_SIZE];
unsigned char inbuf2[CNC_BUF_SIZE];  //parsed and sanitized version of inbuf
unsigned char outbuf[CNC_BUF_SIZE];
unsigned char inputbuf[CNC_BUF_SIZE];



CNCSerial::CNCSerial()
{
	serial_fd = 0;
	log_fd = 0;
	
	pSendingFile = 0; 
	pReceivingFile = 0;
		
	pStartSendingFile = 0;
	pStopSendingFile = 0;
		
	pStartReceivingFile = 0;
	pStopReceivingFile = 0;
		
	pLogMode = LOG_MODE_FILE|LOG_MODE_PRINT;
	
	pLogRx = 1;	//default to logging the RX and TX data
	pLogTx = 1;

	pIgnoreNULL = 1; //CNC will send lots of 0x00 when delaying. ignore them.

	pRTS = 0;
	pDTR = 0;
	pDone = 0;

	pNewRTS = 0;
	pNewDTR = 0;
	pNewFlowControl = FLOW_CONTROL_HARDWARE;  //default to HW
	pClearToSend = 0; //default to flow control stopping any sends.
	pUserPaused = 0;	//user paused
	pMachinePaused = 0;
	
	serialthread = 0;  //this is a pointer, so NULL it out initially
	buttonthread = 0;  //this is a pointer, so NULL it out initially
	
	pButtonThreadRunning = 0;
	pSerialThreadRunning = 0;
	//flow_control;
	//stop_bits;
	
	pPacketDelay = 0;
	pPacketLength = 0;
	pflushRequested = 1;
	
	rxData.reset();
	txData.reset();
}

CNCSerial::~CNCSerial()
{
	stopThreads();
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

void CNCSerial::setFlowControl(FlowControlValues value)  //FLOW_CONTROL_SOFTWARE, FLOW_CONTROL_HARDWARE, or FLOW_CONTROL_NONE
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

// DTR is a modem handshake line
// Not necessary for this application

void CNCSerial::setDTR( unsigned short level)  //0 or non-zero
{
	//log("Requesting DTR: %d\n", level);
	pNewDTR = level;
}


int CNCSerial::getDTR()
{
	return pDTR;
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
	else if(pFlowControl == FLOW_CONTROL_NONE)
		log("Flow Control: None\n");
			
	log("RTS: %d (%d)\n", pRTS, pNewRTS);
	log("DTR: %d (%d)\n", pDTR, pNewDTR);
	log("CTS: %d\n", pClearToSend);

	log("Bytes RX    : %u\r\n", pBytesReceived);
	log("Bytes TX    : %u\r\n", pBytesSent);
	
	log("rxData Size : %d  max: %d\n", rxData.getSize(), rxData.getMaxSize());
	log("txData Size : %d  max: %d\n", txData.getSize(), txData.getMaxSize());

	if(pUserPaused || pMachinePaused)
	{
			log(" ** Paused ** \r\n");
	}
						
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
	
int CNCSerial::startThreads()
{
	Timers timeout;
	
	if(serialthread != 0)
	{
		log("Error: calling startThreads() but the serial thread already exists.\n");
		return CNC_ERROR;   //the thread already exists.
	}
	if(buttonthread != 0)
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
		if(timeout.Expired(SERIAL_1S_TIMEOUT))
		{
			log("Error: Timeout (%dms) waiting for serial thread to start.\n", SERIAL_1S_TIMEOUT);				
			return CNC_TIMEOUT;
		}
	}
	
	buttonthread = new std::thread(&CNCSerial::buttonPollingThread, this);
	timeout.Start();
	while(!pButtonThreadRunning)
	{
		usleep(1000);
		//delay here, but include a timeout
		if(timeout.Expired(SERIAL_1S_TIMEOUT))
		{
			log("Error: Timeout (%dms) waiting for button thread to start.\n", SERIAL_1S_TIMEOUT);
			return CNC_TIMEOUT;
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
			while(pButtonThreadRunning)
			{
				usleep(1000); //wait 1 millisecond
			}
		}
		if(serialthread)
		{
			serialthread->join();  //wait for it to complete.
			delete serialthread;
		}
		
		if(buttonthread)
		{
			buttonthread->join();  //wait for it to complete.
			delete buttonthread;
		}
		
		serialthread = 0;
		buttonthread = 0;
	}
	catch (const std::exception& e)
	{
		log("Error. stopThread() Caught exception: %s\n", e.what());
	}
	
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
				if(timeout.Expired(SERIAL_1S_TIMEOUT))
				{
					log("Error: Timeout (%dms) waiting for serial thread to start receiving file.\n", SERIAL_1S_TIMEOUT);
					
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
			if(timeout.Expired(SERIAL_1S_TIMEOUT))
			{
				log("Error: Timeout (%dms) waiting for serial thread to stop receiving file.\n", SERIAL_1S_TIMEOUT);
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
			if(timeout.Expired(SERIAL_1S_TIMEOUT))
			{
				log("Error: Timeout (%dms) waiting for serial thread to stop sending file.\n", SERIAL_1S_TIMEOUT);
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
				if(timeout.Expired(SERIAL_1S_TIMEOUT))
				{
					log("Error: Timeout (%dms) waiting for serial thread to start sending file.\n", SERIAL_1S_TIMEOUT);				
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
		log("Send String: %s\n", filename);
		strcpy(outFileName, filename);
		//pStartSendingFile = 1;
		if(!pSerialThreadRunning)
		{
			log("Warning: set to send string, but serial thread is not running. Call startThread() first.\n");
		}		
		else
		{
			//Simply add the string to the output queue
			if(txData.putData(str, strlen(str)) == CIRC_QUEUE_FULL)  //however many bytes we have, put them on the queue.
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

//*****************************************************************************************************
//									Private Functions
//*****************************************************************************************************
int CNCSerial::serialOpen() // Open the serial port in read-write mode
{
	speed_t myBaud ;
	struct termios options;
	//close(serial_fd);  //just in case the user calls open twice
	
    serial_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NONBLOCK);// | O_NDELAY
    if (serial_fd < 0) 
	{
        //perror("Error opening serial port");
		log("Error opening serial port.\r\n");
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

	log("Starting Serial [%s]: %d baud, ", SERIAL_PORT, pBaudRate);
	
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
	options.c_cflag |= CS7;			//set the number of data bits (Values are CS5, CS6, CS7, or CS8)
    
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
	
	
	if(pStopBits == STOP_BITS_1)
	{
		options.c_cflag &= ~CSTOPB;		//clear flag means 1 stop bit
	}
	else if(pStopBits == STOP_BITS_2)
	{
		options.c_cflag |= CSTOPB;		//Set flag means two stop bits, rather than one.
	}
	else
	{
		//error("Stop bits are not set before calling open()");
	}
	
	if(pNewFlowControl != pFlowControl)
		pFlowControl = pNewFlowControl;
		
	//FIXME: even in SW flow control, we need to be able to set RTS...

    if(pFlowControl == FLOW_CONTROL_HARDWARE)
	{
		options.c_cflag |= CRTSCTS;		//Enable RTS/CTS (hardware) flow control.
	}
	else if((pFlowControl == FLOW_CONTROL_SOFTWARE) || (pFlowControl == FLOW_CONTROL_NONE))
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
	else if(value == FLOW_CONTROL_HARDWARE)
	{
		options.c_cflag |= CRTSCTS;		//Enable RTS/CTS (hardware) flow control
		options.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
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


int CNCSerial::buttonPollingThread()
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

		pButtonThreadRunning = 1;
		while(!pDone)
		{
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
				//active edge!  toggle the pUserPaused flag
				pUserPaused++;
				pUserPaused &= 0x01;
				
				log("Start/Pause button pressed!  pUserPaused=%d\n", pUserPaused);

				if(!pSendingFile && pFlowControl == FLOW_CONTROL_SOFTWARE)
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
				pStopSendingFile = 1;
				pStopReceivingFile = 1; 
				log("Stop button pressed! Stopping sending and receiving.\n");
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
		
	pButtonThreadRunning = 0;
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

	
	int end_of_file = 0;
	
	int rx_data_active = 0;  //gets set when we receive a 0x12 (Start of Data)
	
	

	int tx_log_fd = 0;
	if(pLogTx)
	{
		tx_log_fd = open("TXlog.txt", O_CREAT|O_RDWR, 0666);  //read and write permissions (octal)
		if (tx_log_fd <= 0) 
		{
			log("Error opening %s to log. Does it exist already?\n", "TXlog.txt");
			log("Error %d, description is : %s\n",errno, strerror(errno));
		}
	}
	
	int rx_log_fd = 0;
	if(pLogRx)
	{
		rx_log_fd = open("RXlog.txt", O_CREAT|O_RDWR, 0666);  //read and write permissions (octal)
		if (rx_log_fd <= 0) 
		{
			log("Error opening %s to log. Does it exist already?\n", "RXlog.txt");
			log("Error %d, description is : %s\n",errno, strerror(errno));
		}	
		else if(rx_log_fd > 0)
		{
			write(rx_log_fd, "RXLog\n", 6);  //write something so we know the log is working.
		}
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

		actualSetFlowControl( pNewFlowControl );
		actualSetDTR( pNewDTR );
		actualSetRTS( pNewRTS );
				
		
		pSerialThreadRunning = 1;
		while(!pDone)
		{

			usleep(10);  //wait 10uS
			//log("*");
			
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
							write(rx_log_fd, str, 7);
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
							}break;
							case XOFF:  //DC2 (sent by CNC to pause data xfer)
							case XOFF2: 
							{
								if(pFlowControl == FLOW_CONTROL_SOFTWARE)
								{
									pClearToSend = 0;
								}
								else if(pFlowControl == FLOW_CONTROL_HARDWARE)
								{
									pMachinePaused = 1;
								}
							}break;
							case DC2:  //start of data
							{
								rx_data_active = 1;
							}break;
							case DC4:  //end of data
							{
								rx_data_active = 0;
								pStopReceivingFile = 1;
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
					
					if(pFlowControl == FLOW_CONTROL_SOFTWARE)
					{
						sendByte( XON );
						//unsigned char byte = XON;
						//write(serial_fd, &byte, 1);
						//tcdrain(serial_fd);
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
						
						
						//TODO: if we time-out or if CTS goes low, then close the file
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
				log("Opening: %s\n", outFileName);
				out_file_fd = open(outFileName, O_RDONLY);
				if (out_file_fd <= 0) 
				{
					log("Error opening %s to send. Does it exist and have read permissions?\n", outFileName);
				}
				else
				{
					setRTS( 1 );  //tell the machine we are ready to send data.
					log("Starting Transfer.\r\n");
					pSendingFile = 1;
					
					if(pFlowControl == FLOW_CONTROL_SOFTWARE)
						pClearToSend = 1;  //default to sending if sw handshaking
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
				//wait for TX queue to empty before clearing sending flag
				txData.reset();  //immediately stop sending data by clearing the TX queue
				end_of_file = 1;
			}
			
			if(end_of_file && txData.isEmpty())
			{
				end_of_file = 0;
				log("File Sent Completely.\r\n");
								
				if(out_file_fd > 0)
				{
					close(out_file_fd);
					out_file_fd = 0;
				}
				pSendingFile = 0;
			}
			
			if(pSendingFile && pClearToSend && (!pUserPaused) && (!pMachinePaused))  //if we are sending a file and not held up by flow control
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
			else  //no flow control
			{
				pClearToSend = 1;
			}
			
			if(pClearToSend && (!pUserPaused) && (!pMachinePaused) && !txData.isEmpty() && delay_timer.Expired(pPacketDelay))
			{
				int max_tx_bytes = pPacketLength;
				if(max_tx_bytes == 0)
					max_tx_bytes = CNC_BUF_SIZE;  //set to max
				
				int num_tx_bytes = txData.getData(outbuf, max_tx_bytes);  //request the max
				
				//be careful, if we overflow the TX buffer, there is no real indication!!
				int num_written = write(serial_fd, outbuf, num_tx_bytes);
				pBytesSent += num_written;	
				
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
			log_fd = open("CNCSerial.log", O_CREAT|O_RDWR, 0666);  //read and write permissions (octal)
			if (log_fd <= 0) 
			{
				printf("Error opening %s to log. Does it exist already?\n", "CNCSerial.log");
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



