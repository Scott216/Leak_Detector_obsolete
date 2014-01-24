Water_Detector_Rx

The receiving panStamp is communicating with the Leonardo via I2C.  The Leonardo is the master and will make a request for the data.  The panStamp will then transmit this as one long array of bytes. Sketch is using DSS's I2C.h library for the I2C master, not Wire.h library.

