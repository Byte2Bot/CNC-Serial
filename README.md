# CNC-Serial
This class is used to communicate with CNC machines using a Raspberry Pi serial interface.

This software is designed to be used with the CNC Serial Hat board from byte2bot.com.  

# Features: 
<li>Hardware or Software handshaking</li>
<li>Serial Port is configurable: Baud, Data bits, Stop Bits, Parity, and Flow Control.</li>
<li>Packet size is configurable</li>
<li>Delay time between packets is configurable</li>
<li>Verbose logging for troubleshooting</li>
<li>remote Start/Pause and Stop buttons</li>
<li>Handling of extra ASCII characters send by the CNC machine</li>

  
# Overview
The core of the CNCSerial class uses two threads to operate.  The first thread is used to handle Start/Stop button inputs using the WiringPi library.  It also manages the red LED.

The second thread is used to control the serial port.  When the thread starts, the serial port is opened.  The serial port is closed when the thread stops.  All serial port operations occur inside this thread.  There are two Circular Queues, one for TX and one for RX.  The port is polled continuously for RX data and immediately handled. 

The main application is able to request actions of the serial thread.  Flags are used to manage the operations that occur inside the serial thread. 

# Usage
To get this working, follow these steps:
1. Build and Install the WiringPi library.

<code>cd ~
gh repo clone WiringPi/WiringPi
cd WiringPi
./build
sudo ldconfig
</code>

2. Download this project.

<code>cd ~
gh repo clone Byte2Bot/CNC-Serial
</code>

3. Change main.c to use specific settings for your machine.  Geany can be used to open the project file.

4. Compile this project.

<code>make
</code>

# LEDs
Red LED: On = Stopped (transmission is stopped).  Blinking = Paused (transmission and reception is paused). Off = Normal operation.


# Troubleshooting
The pi configuration needs to be set so the serial port is enabled, but the console is not. Also, enable I2C:
![PiSettings](https://user-images.githubusercontent.com/130330728/232680315-a24bd43c-57b1-4055-8e3e-fecd00290204.jpg)

After changing the settings, reboot for them to take effect.

If you are still having trouble, try the following:
<code>
sudo systemctl stop getty.target
sudo systemctl disable hciuart
</code>

Note: Try looking at the CNCSerial.log file to see if it explains any cause for the failure.  

More information will be added soon.
