/*
 panStamp Rx sketch for water detectors
 
 v2.00 07/04/14 - added checksum to panStamp and I2C data
 
 To Do:
 How to handle if one panStamp Tx is working and one is not. lastRxSuccess will not timeout and non-working
 panStamp data will stay in array


 Remote panStamp/Sponge sensors send data to this reciever.
 This panStamp communicates with main Arduino via I2C. The panStamp is the I2C slave, main Arduino is Master


 panStamp data received:
 byte 0:   panStamp Rx ID
 byte 1:   panStamp Tx ID
 byte 2:   Wet/Dry Status. Wet = true, Dry = false
 byte 3-4: Temperature from one wire
 Byte 5-6: Battery voltage (mV)
 Byte 7-8: Spare
 Byte 9:   Checksum
 
 I2C data (Same config as panStamp)
 bytes 0-9: Master Bath
 bytes 10-19: Guest Bath

 */

#include "Arduino.h"
#include "EEPROM.h"   // http://www.arduino.cc/en/Reference/EEPROM
#include "cc1101.h"   // http://code.google.com/p/panstamp/source/browse/trunk/arduino/libraries/panstamp/cc1101.h
#include "panstamp.h" // Xcode needs this to compile, arduino IDE does not
#include <Wire.h>     // http://arduino.cc/it/Reference/Wire

// Two LEDs are used to show panStamp and I2C communication
#define LED_COMM     4  // Flashes when there is I2C or Panstamp communication


// The networkAdress of panStamp sender and receiver must be the same
byte panStampNetworkAdress =    46;   // Network address for all Water Detector panStamps (can't be a const varialbe)
byte receiverAddress =          99;   // Device address of this panStamp (can't be a const variable)
const byte SLAVE_ADDR =         21;   // I2C Slave address of this device
const byte DATA_LENGTH =        10;  // Packets sent by each panstamp
const byte ADDR_MASTER_BATH =    1;  // panStamp device address for 2nd floor master bath
const byte ADDR_GUEST_BATH =     2;  // panStamp device address for 2nd floor guest bath
const byte NUM_TX =              2;  // number of panStamp transmitters on this network
const byte PANSTAMP_OFFLINE =  255;  // Send this to I2C master in the panStamp Tx address to indicate panStamp is offline
const uint32_t PANSTAMP_TIMEOUT = 120000; // 2 minute timeout for panStamps

byte I2C_Packet[NUM_TX *  DATA_LENGTH];   // Array to hold data sent over I2C to main Arduino
uint32_t lastRxSuccess[3];        // miliseconds since last successful receipt of panStamp data.  Arrary starrts at 1


CCPACKET packet;  // panStamp data http://code.google.com/p/panstamp/source/browse/trunk/arduino/libraries/panstamp/ccpacket.h



// The connection to the hardware chip CC1101 the RF Chip
CC1101 cc1101;  //http://code.google.com/p/panstamp/wiki/CC1101class

// a flag that a wireless packet has been received
boolean packetAvailable = false;

// Function prototype
void printpanStampConfig();
void printSensorValues();
void wireRequestEvent();


//============================================================================
// Blink LED cnt times
//============================================================================
void blinker(int LED, int cnt)
{
  for(int j=1; j<=cnt; j++)
  {
    digitalWrite(LED, HIGH);
    delay(50);
    digitalWrite(LED, LOW);
    delay(50);
  }
}  // blinker()

//================================================================================================================================================================================
// Handle interrupt from CC1101 (INT 0)
//================================================================================================================================================================================
void cc1101signalsInterrupt(void)
{
  // set the flag that a package is available
  packetAvailable = true;
} // cc1101signalsInterrupt()

//============================================================================
//============================================================================
void setup()
{
  Serial.begin(9600);
  Serial.println(F("Begin panStamp Rx setup, v2.00"));

  pinMode(LED_COMM, OUTPUT);  // Yellow LED
  digitalWrite(LED_COMM, HIGH);
  delay(500);
  digitalWrite(LED_COMM, LOW);

  Wire.begin(SLAVE_ADDR);    // Initiate the Wire library and join the I2C bus

  Wire.onRequest(wireRequestEvent); // Register a function to be called when a master requests data from this slave device.
  Serial.println(F("Wire library initialized"));

  cc1101.init(); // initialize the RF Chip in panStamp

  cc1101.setSyncWord(&panStampNetworkAdress, false);  // Set network address (pointer), false parameter tells function not to save to EEPROM

  // this receiverAddress needs to match the receiverAddress in the Tx panStamp
  cc1101.setDevAddress(receiverAddress, false);  // false parameter tells function not to save to EEPROM
  cc1101.enableAddressCheck(); // you can skip this line, because the default is to have the address check enabled

  // Set this panStamp to be a receiver
  cc1101.setRxState();

  // Enable wireless reception interrupt
  attachInterrupt(0, cc1101signalsInterrupt, FALLING);

  printpanStampConfig();

  Serial.println(F("panStamp Rx setup, complete"));
}  // setup()

//============================================================================
// Get data from panStamps and put into I2C packet.
// When Master requests data from this slave, the wireRequestEvent() will
// execute and send the data to the other Arduiino
//============================================================================
void loop()
{

  // Get data from remote panStamps on 2nd floor
  if(packetAvailable)
  {
    // clear the flag
    packetAvailable = false;

    // Disable wireless reception interrupt so this code finishes executing without inturruption
    detachInterrupt(0);

    if(cc1101.receiveData(&packet) > 0)
    {
      if (packet.crc_ok && packet.length > 1)
      {
        // Copy data from panStamp packet to I2C packet array
        // Put data from both transmiiters in one I2C packet. Master bath first, followed by guest bath.
        byte checksum = 0;
        bool checksumPassed;
        switch (packet.data[1])
        {
          case ADDR_MASTER_BATH:
            Serial.println(F("\nGot packet from Master Bath"));
            // Veify checksum
            for (int k = 0; k < DATA_LENGTH - 1; k++)
            { checksum += packet.data[k]; }
            checksumPassed = ( checksum == packet.data[DATA_LENGTH - 1] );
            for (int j = 0; j < DATA_LENGTH; j++)
            { I2C_Packet[j] = packet.data[j]; }
            
            if ( checksumPassed )
            { lastRxSuccess[1] = millis(); } // Time of successful receipt of panStamp data from Master Bath
            else
            { I2C_Packet[1] = PANSTAMP_OFFLINE; } // checksum failed, set Tx Address to indicate panStamp is offline
            
            blinker(LED_COMM, 1);  // blink LED once to indicate receipt of panStamp packet from Master Bath
            break;

          case ADDR_GUEST_BATH:
            Serial.println(F("\nGot packet from Guest Bath"));
            // Veify checksum
            for (int k = 0; k < DATA_LENGTH - 1; k++)
            { checksum += packet.data[k]; }
            checksumPassed = ( checksum == packet.data[DATA_LENGTH - 1] );

            for (int j = 0; j < DATA_LENGTH; j++)
            { I2C_Packet[j + DATA_LENGTH] = packet.data[j]; }

            if ( checksumPassed )
            { lastRxSuccess[2] = millis(); } // Time of successful receipt of panStamp data from Guest Bath
            else
            { I2C_Packet[1 + DATA_LENGTH] = PANSTAMP_OFFLINE; } // checksum failed, set Tx Address to indicate panStamp is offline
            
            blinker(LED_COMM, 2);  // blink LED twice to indicate receipt of panStamp packet from Guest Bath
            break;

          default:
            Serial.print(F("\nGot packet from unknown panStamp Tx ID = "));
            Serial.println(packet.data[1]);
            I2C_Packet[1] = PANSTAMP_OFFLINE;
            I2C_Packet[1 + DATA_LENGTH] = PANSTAMP_OFFLINE;
            break;

        }  // switch

        printSensorValues();  // Print data received

      } // packet is okay
    }  // got packet

    // Enable wireless reception interrupt
    attachInterrupt(0, cc1101signalsInterrupt, FALLING);
  }


} // loop


//========================================================================================================================================
// function that executes whenever data is requested by master
// this function is registered as an event in setup()
// It sends a 14 byte packet that has data from both panStamps
// If we haven't received data from panStamps in the last 30 minutes, then it won't send any data
//========================================================================================================================================
void wireRequestEvent()
{

  // If we haven't received any data from the panStamp in a while then
  // set the panStamp Tx byte to indicate it's offline by setting it to 255
  if ((long)( millis() - lastRxSuccess[1]) > PANSTAMP_TIMEOUT )    // Master Bath
  { I2C_Packet[1] = PANSTAMP_OFFLINE; }

  if ((long)( millis() - lastRxSuccess[2]) > PANSTAMP_TIMEOUT )   // Guest Bath
  { I2C_Packet[DATA_LENGTH + 1] = PANSTAMP_OFFLINE; }

  // Send byte array from panStamp. Main Arduino will decode bytes
  Wire.write(I2C_Packet, NUM_TX * DATA_LENGTH);

} // wireRequestEvent()



//=============================================================================================================================================
// Print panStamp config info: Frequence, Channel, Network address, Rx address
//=============================================================================================================================================
void printpanStampConfig()
{

  // Print device setup info
  Serial.print(F("Radio Frequency = "));
  if(cc1101.carrierFreq == CFREQ_868)
  {Serial.println(F("868 Mhz"));}
  else
  {Serial.println(F("915 Mhz"));}
  Serial.print(F("Channel = "));
  Serial.println(cc1101.channel);
  Serial.print(F("Network address = "));
  Serial.println(cc1101.syncWord[0]);
  Serial.print(F("Device address =  "));
  Serial.println(cc1101.devAddress);

}  // printpanStampConfig()

//=============================================================================================================================================
//  Convert data from panStamp Tx and print - for debugging
//=============================================================================================================================================
void printSensorValues()
{

  Serial.print("Link Quality: ");
  Serial.print(packet.lqi);
  Serial.print("   RSSI: ");
  Serial.println(packet.rssi);
  for (int p = 0; p < packet.length; p++)
  {
    Serial.print("data[");
    Serial.print(p);
    Serial.print("] = ");
    Serial.println(packet.data[p]);
  }

  int Vcc = 0;
  Vcc  = packet.data[5] << 8;
  Vcc |= packet.data[6];

  Serial.print("millivolts = ");
  Serial.println(Vcc);

  int tempADC = 0;
  tempADC  = packet.data[3] << 8;
  tempADC |= packet.data[4];
  Serial.print("Temp = ");
  Serial.println(tempADC);

} // printSensorValues()


