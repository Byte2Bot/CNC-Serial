#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

//#include <sys/fcntl.h>
//#include <sys/stat.h>
//#include <sys/ioctl.h>

//#include <termios.h>


#include <string.h>  //for string compare

#include "CNCSerial.h"


void printInstructions()
{
	printf("Usage: ./CNCSerial [args]\n");
	printf(" -s FileToSend\n");
	printf(" -r FileToReceive\n");
	printf("\n");
}	



int main(int argc, char **argv) 
{
	CNCSerial CNC;
	
	if(argc < 3)
	{
		printInstructions();
		return 0;
	}
	
	//use printf to the console and the CNCSerial.log file
	CNC.setLogMode(LOG_MODE_PRINT | LOG_MODE_FILE);  
	//If the CNC needs smaller packets of serial data with delays in between
	//use these functions to adjust the packet size and delay in ms between them.
	CNC.setFlush(1);  //
	CNC.setPacketDelayMs(0);  //zero means no delay
	CNC.setPacketLength(0);   //zero means no packet length limit
	//Set up the serial port parameters
	CNC.setBaud(9600);
	CNC.setParity(PARITY_EVEN);
	CNC.setStopBits(STOP_BITS_2);
	CNC.setDataBits(DATA_BITS_7);
	CNC.setFlowControl(FLOW_CONTROL_HARDWARE);

	//When the thread starts, it will open the serial port.
	if(CNC.startThreads() != CNC_OK)  //this will open the port and handle all transfers
	{
		printf("Main: Error starting the serial port thread.\n");
		return 0;  //exit the application
	}

	//start at i=1 to start looking after "./CNCSerial"
	//this only looks for -r... or -s.../-t..., so assume a filename comes after it
	//there is no need to search the last argument because it is a file name
	for(int i = 1; i < (argc - 1); i++)	
	{
		if((strncmp(argv[i], "-s", 2) == 0) || (strncmp(argv[i], "-t", 2) == 0))
		{
			if(CNC.sendFile(argv[i+1]) == CNC_OK)
				printf("Main: Sending: %s\n", argv[i+1]);
		}
		if(strncmp(argv[i], "-r", 2) == 0)
		{
			if(CNC.receiveFile(argv[i+1]) == CNC_OK)
				printf("Main: Receiving: %s\n", argv[i+1]);
		}
	}
	

	
	if(CNC.isFileSending())
	{
		//wait until complete if we are sending
		while(CNC.isFileSending())
		{
			sleep(2);  			//allow other threads to process
			CNC.printStatus();  //mostly for debugging if something isn't going right.			
		}
		printf("Main: File sent successfully.\n");
	}

	if(CNC.isFileReceiving())
	{
		while(CNC.isFileReceiving())
		{
			sleep(2);  			//allow other threads to process
			CNC.printStatus();	//optional: Output something so we can see the progress
		}
		printf("Main: Done Receiving.\n");
	}

	CNC.stopThreads();

    return 0;
}
