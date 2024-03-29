#ifndef Water_Detector_Main_RELEASE
#define Water_Detector_Main_RELEASE 100

#include "Arduino.h"

/*
 Need to put typdef struct definitions in a .h file.  It's some kind of work-around
 Source: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1264977701/8#8
 http://www.arduino.cc/playground/Code/Struct#FAQ
 
 */

#ifndef Types_h
#define Types_h

typedef struct 
{
     bool online;              // True if wireless sensor is online
     byte ID;                  // wireless sensor Transmitter ID
     bool IsWet;               // Wet = true, Dry = false
     int  temp;                // temperature
     bool lowTempMsgFlag;      // one shot trigger for low temp tweet
     int  volts;               // millivolts
     bool lowVoltMsgFlag;      // one shot trigger for low volts tweet
     bool flag_SentMsg_TxIsOffline;       // One shot flag if transmitter goes offline
 uint32_t txOfflineTimestamp;  // Timestamp that sensor went offline
} RemoteSensorData_t;

#endif // Types_h


#endif

