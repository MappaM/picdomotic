PIC Domotic station
==========

PIC program for a domotic station controlling temperature sensors, relay and connected to a PC using UART.

It uses a PIC18F23K20 but should work on most of them. The PC sends simple commandes through /dev/ttyUSB0 like "A 32" to enable the device 32, or "E 32" to disable it. "T" to get temperature, ... See the source code in main.c.

This is MPLAB X IDE project. The only usefull file is in fact main.c, the rest is auto-generated, but I include them if someone would want to directly program the binaries, for example.

Technology used
===========
If you're looking for sample code, you may find here how to :
- Use the UART to communicate with a PC (UART is connected to a "uart usb" module very cheaped that you can find on ebay for example) .
- Use the watchdog timer to ensure a device reset if something goes wrong
- Use Brown-out to jump to do an interrupt and do "something" (blink in my case) if the voltage goes below some threshold
- Use the ADC to read the temperature given by two LM35
- Read/write to eeprom
- Control a 5V relay board with a 3.3V relay board using NPN Transistors (when I'll add the schematic though) 

The python script used to manage the PIC controller is available at https://github.com/MappaM/monitoring/blob/master/scripts/relay.py and a small HTTP server allowing you to remotely control it is available at https://github.com/MappaM/monitoring/blob/master/scripts/server.py
