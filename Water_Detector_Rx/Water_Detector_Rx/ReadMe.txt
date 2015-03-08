Water_Detector_Rx

The receiving panStamp is communicating with the Leonardo via I2C.  The Leonardo is the master and will make a request for the data.  The panStamp will then transmit this as one long array of bytes. Sketch is using DSS's I2C.h library for the I2C master, not Wire.h library.

I2C Data packet structure
byte 0: panStamp Tx ID
byte 1: Tx online: 0 = offline, 1 = online
byte 2: Wet/Dry Status: 0 = Dry, 1 = Wet
byte 3-4: temperature F
Byte 5-6: battery voltage (millivolts)
Byte 7-8: Spare
Byte 9: checksum
