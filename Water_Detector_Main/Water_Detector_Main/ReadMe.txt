/*

To Do:
Take out Tweet counter if no problems after a few months
Consider replacing Twitter with Tropo.com SMS service
Put Remote sensors in a class or namespace
Figure out how to set time when you can't connect to NTP server
Update V3 on PCB so ISCP pins can come out of bottom of Leonardo
If remote sensors come online after being offline, send a tweet that it's online. With it send hours it was down and battery volts

====================
Water Leak Detector.  Detects water leaks using sponge and op-amp circuit. 
The resistance across a dry sponge is essentially infinity, but when wet it varies from 30k - 150k.  For op-amp to trigger, wet 
résistance needs to be less then 200k.
Arduino needs impedance of inputs to be 10k or less, so an op-amp is used to send a signal to the arduino when a sponge 
is wet.  A high input means it's dry, low is wet.  

For sponges that are too far to be hard wired to the op-amps, pansStamps RF devices are used.  
A panStamp should be able to run for a couple months on one CR123 battery. 

There are two panStamps transmitting wet/dry status.  They also transmit their ID, battery voltage and temperature.
They sleep for 8 seconds, wake up for 1/2 second to transmit, then repeat.

The receiving panStamp is communicating with the Leonardo via I2C.  The Leonardo is the master and will make
a request for the data.  The panStamp will then transmit this as one long array of bytes. Sketch is using DSS's 
I2C.h library for the I2C master, not Wire.h library.

The sketch gets time from an NPT server.  Water detect notifications and low battery are sent out via Twitter.

Sketch is low on RAM

Hardware:
* Leak detector PCB 
* Leonardo
* panStamp
* Ethernet shield R3
* 3 op-amps: MCP6004 http://search.digikey.com/us/en/products/MCP6004-I%2FP/MCP6004-I%2FP-ND/523060
* 12 470k Ohm 1/4 watt resistors for Op-amp circuit
* 2 4.7k ohm resistors for I2C pull-up
* Screw Terminal for Screw shield to add two more ground terminals
* Sponges for water detector
* 1 set arduino headers
* 3 8-position screw terminals, 0.1" pitch
* 1 2-position screw terminal for power
* Red LED
* Yellow LED
* Green LED
* 2 headers for panstamp
* 2 DIN Rail clips
* Mounting hardware
* Enclosure OKW



See this Arduino Thread forum on a discussion on reading inputs and using an Op Amp 
  http://arduino.cc/forum/index.php/topic,90671.msg682130.html#msg682130


I2C wiring
panStamp: A4 & A5
Leonardo D2 (SDA), D3 (SCL)
4.7k pul-lup resistors to 3v


I2C Data packet 
== Master Bath ==
byte 0: panStamp Rx ID
byte 1: panStamp Tx ID
byte 2: Wet/Dry Status. Wet = true, Dry = false
byte 3-4: ADC Temperature from TMP36
Byte 5-6: ADC value for battery voltage

== Guest Bath ==
byte 7: panStamp Rx ID
byte 8: panStamp Tx ID
byte 9: Wet/Dry Status
byte 10-11: ADC Temperature from TMP36
Byte 12-13: ADC value for battery voltage


*/
