//CNCSerial.h
#ifndef __CNCSERIAL_H__
#define __CNCSERIAL_H__

#include <iostream>
#include <thread>
#include <stdarg.h> //for va_start/va_end

#include "CircularQueue.h"

//#define SERIAL_PORT "/dev/ttyS0" //mini UART, does not have flow control
#define SERIAL_PORT "/dev/ttyAMA0" //first PL011 (UART0). This does have flow control.
// ttyAMA0 requires the following after boot-up: 
//sudo systemctl stop getty.target
//sudo systemctl disable hciuart

#define USB_PORT "/dev/ttyUSB0"
#define FILE_NAME "1001.nc"     // Change this to the name of the file you want to send

//These are the de-facto standard flow control bytes
#define XON 0x11		//Device Control 1 (Start)
#define DC2 0x12		//Device Control 2 (Start of Data)
#define XOFF 0x13		//Device Control 3 (Stop) - when parameter 0055#0 = 1
#define DC4 0x14		//Device Control 4 (End of Data)
#define XOFF2 0x93		//Stop - when parameter 0055 = 0
#define NAK 0x15		//Sent by CNC when there is an Alarm, parameter 0055#0 = 1
#define NAK2 0x95		//parameter 0055#0 = 0
#define SYN 0x16		//Sent by CNC when there is an NC Reset, parameter 0055#0 = 1
#define SYN2 0x96		//parameter 0055#0 = 0


#ifndef TRUE
#define TRUE 1
#endif

//function return values
#define CNC_ERROR 0
#define CNC_OK 1
#define CNC_TIMEOUT 2

#define SERIAL_1S_TIMEOUT  1000  //ms

extern int errno;	//in errno.h


//questions about pins?  use "gpio readall" on the commandline. 
//wPi column contains #define values 

//Raspberry Pi CNC Serial pinout is as follows:
//Start/Pause: 	GPIOx 	(pin 3)  pulled up in HW, ground activated
//Stop:			GPIOx 	(pin 5)  pulled up in HW, ground activated
//Pause LED:	GPIO1   (pin 12) output high to turn LED on
//TX: 			GPIO14  (pin 8)
//RX: 			GPIO15  (pin 10)
//CTS: 			GPIO16  (pin 36)
//RTS: 			GPIO17  (pin 11)

#define PAUSE_LED_PIN 			1  //GPIO1
#define SERIAL_STARTPAUSE_PIN 	8	//SDA.1
#define SERIAL_STOP_PIN  		9	//SCL.1



#define SERIAL_BLINK_DELAY		200  //ms
#define DEBOUNCE_MAX_COUNT		16	//#loops of the same input will determine button state.


//For a Fanuc O-M:
//during receive, hold RTS high, CNC will pull CTS high when it is sending.
//it will send:
//0x1B (ESC)
//0x26   (&)
//0x48   (H)
//0x45   (E)
//0x3A   (:)
//0x12 (DC2)
//There will be lots of 0x00 being sent, ignore these.
//The file will end with (%), a small delay, then 0x14 (DC4)



//DC3 transmission condition
//Free Bloch\f19 Buffer space 1024 characters
//DC1 transmission condition
//Free buffer space  2048 characters
//Allowable overrun
//Less than 1024 characters

//If the CNC enters the alarm or reset condition, the remote buffer transmits
//the DC3 code, then clears the entire contents of the buffer.





//USB UART uses ioctl for HW handshake, so this software defaults to that method.
enum FlowControlValues
{
	FLOW_CONTROL_NONE = 0,
	FLOW_CONTROL_HARDWARE = 1,
	FLOW_CONTROL_SOFTWARE = 2,	
	FLOW_CONTROL_MASK = 0x03
};

enum StopBitValues
{
	STOP_BITS_1 = 1,
	STOP_BITS_2 = 2,
	STOP_BIT_MASK = 0x03
};

enum DataBitValues
{
	DATA_BITS_6 = 6,
	DATA_BITS_7 = 7,
	DATA_BITS_8 = 8,
	DATA_BITS_9 = 9,
	DATA_BITS_MASK = 0x0F
	
};

enum ParityValues
{
	PARITY_NONE = 0,
	PARITY_ODD = 1,
	PARITY_EVEN = 2
};

enum SerialLEDStates
{
	SERIAL_LED_OFF = 0,
	SERIAL_LED_ON = 1,
	SERIAL_LED_BLINKING = 2
};	

//SerialLogMode is used as a bit mask, so be careful when setting specific values.

#define LOG_MODE_NONE 0
#define LOG_MODE_PRINT (1<<0)
#define LOG_MODE_FILE (1<<1)		//Setting this will log to "CNCSerial.log"



class CNCSerial
{
	private:
	    int serial_fd;	//for the serial port
	    int log_fd;

		
		char inFileName[128];
		char outFileName[128];
		
		//flags for the threads
		int pDone;
		int pSendingFile; 
		int pReceivingFile;
		
		int pStartSendingFile;
		int pStopSendingFile;
		
		int pStartReceivingFile;
		int pStopReceivingFile;
		
		int pSerialThreadRunning;
		int pButtonThreadRunning;
		
		int pStartPauseButton;
		int pStopButton;		
		
		FlowControlValues pFlowControl;  	//actual. default to HW
		FlowControlValues pNewFlowControl;	//requested.	
		
		//Serial port values. Once the port is opened, these cannot be changed.
		StopBitValues pStopBits;
		ParityValues pParity;
		DataBitValues pDataBits;
		int pBaudRate;
		
		int pLogMode;
		int pLogRx;
		int pLogTx;
		
		int pRTS;		//actual
		int pNewRTS;	//requested
		
		int pDTR;		//actual
		int pNewDTR;	//requested
		
	
		CircularQueue rxData;
		CircularQueue txData;		
		
		int pClearToSend; //default to flow control stopping any sends.
		int pMachinePaused;
		int pUserPaused;
		
		
		//variables to control serial flow
		int pflushRequested;
		int pPacketLength;  //bytes
		int pPacketDelay;  //mS
		int pIgnoreNULL;
		
 
		//Statistics
		unsigned long pBytesSent;
		unsigned long pBytesReceived;
		

		//private functions
		int isSerialWriteable();
		int serialOpen();		
		
		int actualSetRTS(unsigned short level);
		int actualSetFlowControl(FlowControlValues value);
		int actualSetDTR( unsigned short level);
		int actualGetCTS();
		
		int serialPollingThread();
		int buttonPollingThread();
		
		void log(const char *message, ...);
		
		int receive(unsigned char *rxdata, int maxlength);
		std::thread *serialthread;
		std::thread *buttonthread;
		
	public:
		CNCSerial();
		~CNCSerial();
		
		//these are used before startThread()
		void setStopBits(StopBitValues value);
		StopBitValues getStopBits();
		
		void setParity(ParityValues value);
		ParityValues getParity();
		
		void setDataBits(DataBitValues value);
		DataBitValues getDataBits();
		
		void setBaud(int value);
		int getBaud();
		
		
		//These can be used at runtime
		void setFlowControl(FlowControlValues value);
		FlowControlValues getFlowControl();	
		
		void setDTR( unsigned short level);
		int getDTR();
		
		void setRTS(unsigned short level);
		int getRTS();
		
		void setFlush(int value);
		int getFlush();
		
		int getCTS();
		
		void resetStats();
		void printStatus();
		
		void setUserPaused(int value);
		int getUserPaused();
		int getMachinePaused();
		
		
		int getStopButton();
		
		int getPauseButton();
		
		void setLogMode( int value );
		void setLogRx( int value );
		void setLogTx( int value );
		int getLogRx();
		int getLogTx();
		
		
		
		int send(unsigned char *txdata, int length);
		int sendByte( unsigned char byte );  //mostly used for flow control bytes
		
		
		//Note: Filenames are limited to 128 bytes
		int sendFile(char *filename);
		int receiveFile(char *filename);
		
		int isFileSending();  //1=YES, 0=NO
		int isFileReceiving();  //1=YES, 0=NO
		
		void stopReceiveFile();
		void stopSendFile();

		int rxBytesWaiting();
		
		void setPacketLength(unsigned int bytes);
		unsigned int getPacketLength();
		
		void setPacketDelayMs(unsigned int microseconds);
		unsigned int getPacketDelayMs();
		
		int startThreads();
		int stopThreads();
};

#endif  //__CNCSERIAL_H__
