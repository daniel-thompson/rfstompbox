This is the README file for EasyLogger.

EasyLogger is primarily an example application. It demonstrates how two more
I/O pins can be acquired on the ATTiny45 by omitting the external crystal.
Instead of a crystal, the internal RC oscillator is used and AVR-USB is
configured to run on 16.5 MHz +/- 1% clock rate with internal synchronization
to the host.

The example application is a data logger. It reads an ADC conversion value
every second and sends it to the host. In order to save the operating system
specific host software, it presents itself as a USB keyboard to the host and
automatically enters the values it measures. To get a series of measurement
values, just open a text editor or even a spreadsheet like Excel and press the
start/stop button on the logger. EasyLogger will start to type values directly
into your host application. The LED indicates that EasyLogger is active while
it takes samples and sends keystrokes to your computer.


FILES IN THE DISTRIBUTION
=========================
Readme.txt ........ The file you are currently reading.
firmware .......... Source code of the controller firmware.
firmware/usbdrv ... USB driver -- See Readme.txt in that directory for info
License.txt ....... Public license (GPL2) for all contents of this project.
Changelog.txt ..... Logfile documenting changes in firm- and hardware.


BUILDING AND INSTALLING
=======================
This project can be built on Unix (Linux, FreeBSD or Mac OS X) or Windows.

Building on Windows:
You need WinAVR to compile the firmware. A package can be downloaded from:

    WinAVR: http://winavr.sourceforge.net/

To build the firmware with WinAVR, change into the "firmware" directory,
check whether you need to edit the "Makefile" (e.g. change the ISP upload
tool) and type "make" to compile the source code. Before you upload the code
to the device with "make flash", you should set the fuses with "make fuse".

Building on Unix (Linux, FreeBSD and Mac):
You need the GNU toolchain and avr-libc to compile the firmware. See

    http://www.nongnu.org/avr-libc/user-manual/install_tools.html

for instructions on how to install avr-gcc and avr-libc.

To build the firmware, change to the "firmware" directory, edit "Makefile"
to use the programmer of your choice and type "make" to compile the source
code. Before you upload the code to the device with "make flash", you
should set the fuses with "make fuse".



ABOUT THE LICENSE
=================
It is our intention to make our USB driver and this demo application
available to everyone. Moreover, we want to make a broad range of USB
projects and ideas for USB devices available to the general public. We
therefore want that all projects built with our USB driver are published
under an Open Source license. Our license for the USB driver and demo code is
the GNU General Public License Version 2 (GPL2). See the file "License.txt"
for details.

If you don't want to publish your source code under the GPL2, you can simply
pay money for AVR-USB. As an additional benefit you get USB PIDs for free,
licensed exclusively to you. See the file "CommercialLicense.txt" for details.


MORE INFORMATION
================
For more information about Objective Development's firmware-only USB driver
for Atmel's AVR microcontrollers please visit the URL

    http://www.obdev.at/products/avrusb/

A technical documentation of the driver's interface can be found in the
file "firmware/usbdrv/usbdrv.h".


--
(c) 2007 by OBJECTIVE DEVELOPMENT Software GmbH.
http://www.obdev.at/
