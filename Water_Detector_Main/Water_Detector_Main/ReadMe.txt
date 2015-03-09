The sketch gets time from an NPT server.  Water detect notifications and low battery are sent out via Twitter.

See this Arduino Thread forum on a discussion on reading inputs and using an Op Amp 
  http://arduino.cc/forum/index.php/topic,90671.msg682130.html#msg682130


I2C Data packet structure
byte 0: panStamp Tx ID
byte 1: Tx online: 0 = offline, 1 = online
byte 2: Wet/Dry Status: 0 = Dry, 1 = Wet
byte 3-4: temperature F
Byte 5-6: battery voltage (millivolts)
Byte 7-8: Spare
Byte 9: checksum


Put sensors in a class.  One class for sensors, then a wired subclass and wireless subclass
Create an array of class objects

Properties:
 For both wired and wireless
   Is wet input - live wet/dry status
   Is wet status - used to trigger dry to wet transition
   Is wet output - wet dry status after some delay
   Name - character array of sensor location, this text can be used for messages on OLED and twitter
   Dry out delay - timer used to let sponge dry out.  Elimantes cycling when sponge is drying out
   Wet delay - delay used to make sure sponge is really wet.  Elimantes cycling when sponge first gets wet
   
 For wireless only:
   Tx Address
   Tx is online
   Temperature
   Battery volts
   Flag - low volts message
   Flag - low temp message
   Flag - sensor offline message


Methods/Actions
  For both wired and wireless
    Send message when water is detected
    Send message when sponge dries up

  For wireless only
    Send message if Tx is offline
    Send messaeg if Tx goes back online
    Send message if battery is low
    Send message if temp is low
    Send message for weekly heartbeat
