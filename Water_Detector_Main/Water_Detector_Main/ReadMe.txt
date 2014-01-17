To Do:
Take out Tweet counter if no problems after a few months
Consider replacing Twitter with Tropo.com SMS service
Put Remote sensors in a class or namespace
Figure out how to set time when you can't connect to NTP server
Update V3 on PCB so ISCP pins can come out of bottom of Leonardo
If remote sensors come online after being offline, send a tweet that it's online. With it send hours it was down and battery volts

====================

For sponges that are too far to be hard wired to the op-amps, pansStamps RF devices are used.  

The sketch gets time from an NPT server.  Water detect notifications and low battery are sent out via Twitter.

See this Arduino Thread forum on a discussion on reading inputs and using an Op Amp 
  http://arduino.cc/forum/index.php/topic,90671.msg682130.html#msg682130


I2C wiring
panStamp: A4 & A5
Leonardo D2 (SDA), D3 (SCL)
4.7k pullup resistors to 3v


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


