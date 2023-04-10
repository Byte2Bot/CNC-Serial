# CNC-Serial
This class is used to communicate with CNC machines over the serial interface

This software is designed to be used with the CNC Serial Hat board from byte2bot.com.  Hardware handshaking is supported.

The core of the CNCSerial class uses two threads to operate.  The first thread is used to handle Start/Stop button inputs using the WiringPi library.  It also manages the red LED.

The second thread is used to control the serial port.  When the thread starts, the serial port is opened.  The serial port is closed when the thread stops.  All serial port operations occur inside this thread.

The main application is able to request actions of the serial thread.  Flags are used to manage the operations that occur inside the serial thread. 
