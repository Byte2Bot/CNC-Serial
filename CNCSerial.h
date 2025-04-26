//CNCSerial.h
#ifndef __CNCSERIAL_H__
#define __CNCSERIAL_H__

#include <iostream>
#include <thread>
#include "Log.h"
#include "CircularQueue.h"

//#define SERIAL_PORT "/dev/ttyS0" //mini UART, does not have flow control
#define SERIAL_PORT "/dev/ttyAMA0" //first PL011 (UART0). This does have flow control.
// ttyAMA0 requires the following after boot-up: 
//sudo systemctl stop getty.target
//sudo systemctl disable hciuart

#define USB_PORT "/dev/ttyUSB0"
#define FILE_NAME "1001.nc"     // Change this to the name of the file you want to send

//These are the de-facto standard flow control bytes
#define XON 0x11		//Device Control 1 (Start) (DC1)
#define DC2 0x12		//Device Control 2 (Start of Data) sent by sender before sending data
#define XOFF 0x13		//Device Control 3 (Stop) (DC3) - when parameter 0055#0 = 1
#define DC4 0x14		//Device Control 4 (End of Data) sent by sender after sending data
#define XOFF2 0x93		//Stop - when Fanuc parameter 0055 = 0, also default for Mitsubishi
#define NAK 0x15		//Sent by CNC when there is an Alarm, parameter 0055#0 = 1
#define NAK2 0x95		//parameter 0055#0 = 0
#define SYN 0x16		//Sent by CNC when there is an NC Reset, parameter 0055#0 = 1
#define SYN2 0x96		//parameter 0055#0 = 0


#define CNC_BUF_SIZE 256		//our internal memory buffer for TX.

#ifndef TRUE
#define TRUE 1
#endif

//function return values
#define CNC_ERROR 0
#define CNC_OK 1
#define CNC_TIMEOUT 2


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
	FLOW_CONTROL_GRBL = 3,
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
	DATA_BITS_5 = 5,
	DATA_BITS_6 = 6,
	DATA_BITS_7 = 7,
	DATA_BITS_8 = 8,
	//DATA_BITS_9 = 9,
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



class CNCSerial
{
	private:
	    int serial_fd;	//for the serial port
	    int log_fd;

		
		char inFileName[128];
		char outFileName[128];
		
		char pStatus[128];
		int pStatusNew;
		
		char pSerialName[64];
		
		
		//flags for the threads
		int pDone;
		int pSendingFile; 
		int pReceivingFile;
		
		int pStartSendingFile;
		int pStopSendingFile;
		
		int pStartReceivingFile;
		int pStopReceivingFile;
		
		int pSerialThreadRunning;
		int pIOThreadRunning;
		
		int pStartPauseButton;		//This keeps track of the button status
		int pStopButton;			//This keeps track of the button status
		int pStartPauseButtonFlag;  //This tells the SerialThread to handle the press
		int pStopButtonFlag;		//This tells the SerialThread to handle the press
		
		int pUseRxFlowControl;
		int pUseStartStopChar;
		unsigned char pStartStopChar;
		
		FlowControlValues pFlowControl;  	//actual. default to HW
		FlowControlValues pNewFlowControl;	//requested.	
		
		//Serial port values. Once the port is opened, these cannot be changed.
		StopBitValues pStopBits;
		ParityValues pParity;
		DataBitValues pDataBits;
		int pBaudRate;
		
		unsigned char pXOFFByte;
		
		int pCurrentLine;
		int pNumLines;
		
		int pLogMode;
		int pLogRx;
		int pLogTx;
		
		int pRTS;		//actual
		int pNewRTS;	//requested
		
		int pDSR;
		int pNewDSR;
		
		int pDTR;		//actual
		int pNewDTR;	//requested
		
	
		CircularQueue rxData;
		CircularQueue txData;		
		
		int pClearToSend; //default to flow control stopping any sends.
		int pMachinePaused;
		int pUserPaused;
		
		int pReopen;
		int pSingleStep;
		int pDoStep;
		//int pNewSingleStep;
		//int pNewDoStep;
		
		
		//variables to control serial flow
		int pflushRequested;
		int pPacketLength;  //bytes
		int pPacketDelay;  //mS
		int pIgnoreNULL;
		
		int pGRBLReplyIndex;
		int pGRBLNextFlag;
		char pGRBLReply[128]; 
		
 
		//Statistics
		unsigned long pBytesSent;
		unsigned long pBytesReceived;
		unsigned long pFileSize;
		

		//private functions
		int isSerialWriteable();
		int serialOpen();		
		
		int actualSetRTS(unsigned short level);
		int actualSetFlowControl(FlowControlValues value);
		int actualSetDTR( unsigned short level);
		int actualGetCTS();
		int actualSetDSR( unsigned short level);
		
		int serialPollingThread();
		int ioPollingThread();  //define this in app code if using a different hat.
		
		void setStatus(char *text);
		
		void log(const char *message, ...);
		
		int receive(unsigned char *rxdata, int maxlength);
		std::thread *serialthread;
		std::thread *iothread;
		
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
		
		void setSerialName(char *name);
		char *getSerialName();
		
		void setXOFFByte( char byte );
		unsigned char getXOFFByte();
		
		int getUseRxFlowControl();
		void setUseRxFlowControl( int val );
		
		int getUseStartStopChar();
		void setUseStartStopChar( int val );
		
		unsigned char getStartStopChar();
		void setStartStopChar( unsigned char val );
		
		
		int saveSettings(char *settingsFileName);
		int loadSettings(char *settingsFileName, char *prefix);
		
		//These can be used at runtime
		void reopen();
		
		void setFlowControl(FlowControlValues value);
		FlowControlValues getFlowControl();	
		
		void setDTR( unsigned short level);
		int getDTR();
		
		void setRTS(unsigned short level);
		int getRTS();
		
		void setDSR(unsigned short level);
		int getDSR();
		
		void setFlush(int value);
		int getFlush();
		
		int getCTS();
		
		void resetStats();
		void printStatus();
		void clearQueues();
		
		void setUserPaused(int value);
		int getUserPaused();
		int getMachinePaused();
		
		int getSingleStep();
		void setSingleStep(int value);
		
		int getStopButton();
		
		int getPauseButton();
		
		void setLogMode( int value );
		void setLogRx( int value );
		void setLogTx( int value );
		int getLogRx();
		int getLogTx();
		
		char *getStatus();
		int isStatusNew();// {return pStatusNew; }
		
		
		unsigned long getFileSize();
		unsigned long getNumBytesSent();
		
		int getRxQueueSize(); //returns the size of the RX queue
		unsigned char getRxQueueByte();  //returns a byte from the RX queue, 0 if not available
		
		int getCurrentLine();
		int getNumLines();
		
		
		int send(unsigned char *txdata, int length);
		int sendByte( unsigned char byte );  //mostly used for flow control bytes
		
		
		//Note: Filenames are limited to 128 bytes
		int sendFile(char *filename);
		int receiveFile(char *filename);
		
		int sendString(char *str);
		
		int isFileSending();  //1=YES, 0=NO
		int isFileReceiving();  //1=YES, 0=NO
		
		int isSerialThreadRunning();
		
		void stopReceiveFile();
		void stopSendFile();

		int rxBytesWaiting();
		
		void setPacketLength(unsigned int bytes);
		unsigned int getPacketLength();
		
		void setPacketDelayMs(unsigned int microseconds);
		unsigned int getPacketDelayMs();
		
		int startThreads(int useButtonThread); //button thread is only useful for CNCFeeder
		int stopThreads();
		
		void StopButtonPress();
		void StartPauseButtonPress();
		
		unsigned long getBytesSent() {return pBytesSent; }
		unsigned long getBytesReceived() {return pBytesReceived; }
};

#endif  //__CNCSERIAL_H__
