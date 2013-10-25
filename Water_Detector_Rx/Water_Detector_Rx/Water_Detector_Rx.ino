/*

 To Do:
 How to handle if one panStamp Tx is working and one is not. lastRxSuccess will not timeout and non-working
 panStamp data will stay in array

 panStamp Rx sketch for water detectors

 Remote panStamp/Sponge sensors send data to this reciever.
 This panStamp communicates with main Arduino via I2C. The panStamp is the slave

 Simple panStamp test - doesn't use SWAP protocal
 Source: http://www.panstamp.org/forum/showthread.php?tid=22


 PanStamp data received - this is put into I2C packet
 == panStamp Master Bath ==
 byte 0: panStamp Rx ID
 byte 1: panStamp Tx ID
 byte 2: Wet/Dry Status. Wet = true, Dry = false
 byte 3-4: Temperature from one wire
 Byte 5-6: ADC value for battery voltage
 == panStamp Guest Bath ==
 byte 7: panStamp Rx ID
 byte 8: panStamp Tx ID
 byte 9: Wet/Dry Status
 byte 10-11: ADC Temperature from TMP36
 Byte 12-13: ADC value for battery voltage


 panStamp Pinout

 ANT
 GND         GND
 D8          GND
 D9          D7
 A0          D6
 A1          D5
 A2          D4  (Amber LED)
 GND         D3
 A3          D1
 A4 SCA      D0
 A5 SCL      GND
 A6          VCC
 A7          RST


 */
#include "Arduino.h"
#include "EEPROM.h"   // http://www.arduino.cc/en/Reference/EEPROM
#include "cc1101.h"   // http://code.google.com/p/panstamp/source/browse/trunk/arduino/libraries/panstamp/cc1101.h
#include "panstamp.h" // Xcode needs this to compile, arduino IDE does not
#include <Wire.h>     // http://arduino.cc/it/Reference/Wire

// Two LEDs are used to show panStamp and I2C communication
#define LED_COMM     4  // Flashes when there is I2C or Panstamp communication


// The networkAdress of panStamp sender and receiver must be the same
byte panStampNetworkAdress =   46;  // Network address for all Water Detector panStamps
byte receiverAddress =         99;  // Device address of this panStamp
const byte addrSlaveI2C =      21;  // I2C Slave address of this device
const byte packetsPerPanStamp = 7;  // Packets sent by each panstamp
const byte addrMasterBath =     1;  // panStamp device address for 2nd floor master bath
const byte addrGuestBath =      2;  // panStamp device address for 2nd floor guest bath
const byte numTransmitters =    2;  // number of panStamp transmitters on this network
const byte panStampOffline =  255;  // Send this to I2C master in the panStamp Rx address to indicate panStamp is offline
const uint32_t panStampTimeout = 600000; // 10 minute timeout for panStamps.  If no connections in 10 mintues, don't send data to Master

byte I2C_Packet[numTransmitters *  packetsPerPanStamp];   // Array to hold data sent over I2C to main Arduino
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
/*
 888      888 d8b          888
 888      888 Y8P          888
 888      888              888
 88888b.  888 888 88888b.  888  888  .d88b.  888d888
 888 "88b 888 888 888 "88b 888 .88P d8P  Y8b 888P"
 888  888 888 888 888  888 888888K  88888888 888
 888 d88P 888 888 888  888 888 "88b Y8b.     888
 88888P"  888 888 888  888 888  888  "Y8888  888
 */
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
/*
 d888    d888   .d8888b.   d888           d8b                            888          8888888          888                                              888
 d8888   d8888  d88P  Y88b d8888           Y8P                            888            888            888                                              888
 888     888  888    888   888                                          888            888            888                                              888
 .d8888b .d8888b   888     888  888    888   888  .d8888b  888  .d88b.  88888b.   8888b.  888 .d8888b    888   88888b.  888888 .d88b.  888d888 888d888 888  888 88888b.  888888
 d88P"   d88P"      888     888  888    888   888  88K      888 d88P"88b 888 "88b     "88b 888 88K        888   888 "88b 888   d8P  Y8b 888P"   888P"   888  888 888 "88b 888
 888     888        888     888  888    888   888  "Y8888b. 888 888  888 888  888 .d888888 888 "Y8888b.   888   888  888 888   88888888 888     888     888  888 888  888 888
 Y88b.   Y88b.      888     888  Y88b  d88P   888       X88 888 Y88b 888 888  888 888  888 888      X88   888   888  888 Y88b. Y8b.     888     888     Y88b 888 888 d88P Y88b.
 "Y8888P "Y8888P 8888888 8888888 "Y8888P"  8888888 88888P' 888  "Y88888 888  888 "Y888888 888  88888P' 8888888 888  888  "Y888 "Y8888  888     888      "Y88888 88888P"   "Y888
 888                                                                                         888
 Y8b d88P                                                                                         888
 "Y88P"                                                                                          888
 */
// Handle interrupt from CC1101 (INT 0)
//================================================================================================================================================================================
void cc1101signalsInterrupt(void)
{
  // set the flag that a package is available
  packetAvailable = true;
} // cc1101signalsInterrupt()

//============================================================================
/*                888
 888
 888
 .d8888b   .d88b.  888888 888  888 88888b.
 88K      d8P  Y8b 888    888  888 888 "88b
 "Y8888b. 88888888 888    888  888 888  888
 X88 Y8b.     Y88b.  Y88b 888 888 d88P
 88888P'  "Y8888   "Y888  "Y88888 88888P"
 888
 888
 888

 http://patorjk.com/software/taag/#p=display&f=Colossal&t=setup  */
//============================================================================
void setup()
{
  Serial.begin(9600);
  Serial.println(F("Begin panStamp Rx setup()"));

  pinMode(LED_COMM, OUTPUT);  // Yellow LED
  digitalWrite(LED_COMM, HIGH);
  delay(500);
  digitalWrite(LED_COMM, LOW);

  Wire.begin(addrSlaveI2C);    // Initiate the Wire library and join the I2C bus

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

  Serial.println(F("panStamp Rx setup() complete"));
}  // setup()

//============================================================================
/*
 888
 888
 888
 888  .d88b.   .d88b.  88888b.
 888 d88""88b d88""88b 888 "88b
 888 888  888 888  888 888  888
 888 Y88..88P Y88..88P 888 d88P
 888  "Y88P"   "Y88P"  88888P"
 888
 888
 888
 */
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
        // Put Master bath data in 1st 7 bytes of I2C packet and guest bath in latter 7 bytes
        switch (packet.data[1])
        {
          case addrMasterBath:
            Serial.println(F("\nGot packet from Master Bath"));
            for (int j = 0; j < packetsPerPanStamp; j++)
            {
              I2C_Packet[j] = packet.data[j];
              lastRxSuccess[1] = millis(); // Track time of successful receipt of panStamp data from Master Bath
            }
            blinker(LED_COMM, 1);  // blink LED once to indicate receipt of panStamp packet from Master Bath
            break;

          case addrGuestBath:
            Serial.println(F("\nGot packet from Guest Bath"));
            for (int j = 0; j < packetsPerPanStamp; j++)
            {
              I2C_Packet[j + packetsPerPanStamp] = packet.data[j];
              lastRxSuccess[2] = millis(); // Track time of successful receipt of panStamp data from Master Bath
            }
            blinker(LED_COMM, 2);  // blink LED twice to indicate receipt of panStamp packet from Guest Bath
            break;

          default:
            Serial.print(F("\nGot packet from unknown panStamp Tx ID = "));
            Serial.println(packet.data[1]);

        }  // switch

        printSensorValues();  // Print data received

      } // packet is okay
    }  // got packet

    // Enable wireless reception interrupt
    attachInterrupt(0, cc1101signalsInterrupt, FALLING);
  }


} // loop


//========================================================================================================================================
/*
 d8b                 8888888b.                                              888    8888888888                           888
 Y8P                 888   Y88b                                             888    888                                  888
 888    888                                             888    888                                  888
 888  888  888 888 888d888 .d88b.  888   d88P .d88b.   .d88888 888  888  .d88b.  .d8888b  888888 8888888   888  888  .d88b.  88888b.  888888
 888  888  888 888 888P"  d8P  Y8b 8888888P" d8P  Y8b d88" 888 888  888 d8P  Y8b 88K      888    888       888  888 d8P  Y8b 888 "88b 888
 888  888  888 888 888    88888888 888 T88b  88888888 888  888 888  888 88888888 "Y8888b. 888    888       Y88  88P 88888888 888  888 888
 Y88b 888 d88P 888 888    Y8b.     888  T88b Y8b.     Y88b 888 Y88b 888 Y8b.          X88 Y88b.  888        Y8bd8P  Y8b.     888  888 Y88b.
 "Y8888888P"  888 888     "Y8888  888   T88b "Y8888   "Y88888  "Y88888  "Y8888   88888P'  "Y888 8888888888  Y88P    "Y8888  888  888  "Y888
 888
 888
 888
 */
// function that executes whenever data is requested by master
// this function is registered as an event in setup()
// It sends a 14 byte packet that has data from both panStamps
// If we haven't received data from panStamps in the last 30 minutes, then it won't send any data
//========================================================================================================================================
void wireRequestEvent()
{

  // If sketch hasn't received any data from the panStamp in a while then
  // set the panStamp Tx byte to indicate it's offline by setting it to 255
  if ((long)( millis() - lastRxSuccess[1]) > panStampTimeout )    // Master Bath
  {
    I2C_Packet[1] = panStampOffline;
  }

  if ((long)( millis() - lastRxSuccess[2]) > panStampTimeout )   // Guest Bath
  {
    I2C_Packet[packetsPerPanStamp + 1] = panStampOffline;
  }

  // Send byte array from panStamp. Main Arduino will decode bytes
  Wire.write(I2C_Packet, numTransmitters * packetsPerPanStamp);

} // wireRequestEvent()



//=============================================================================================================================================
/*
 d8b          888                                .d8888b.  888                                     .d8888b.                     .d888 d8b
 Y8P          888                               d88P  Y88b 888                                    d88P  Y88b                   d88P"  Y8P
 888                               Y88b.      888                                    888    888                   888
 88888b.  888d888 888 88888b.  888888 88888b.   8888b.  88888b.   "Y888b.   888888  8888b.  88888b.d88b.  88888b.  888         .d88b.  88888b.  888888 888  .d88b.
 888 "88b 888P"   888 888 "88b 888    888 "88b     "88b 888 "88b     "Y88b. 888        "88b 888 "888 "88b 888 "88b 888        d88""88b 888 "88b 888    888 d88P"88b
 888  888 888     888 888  888 888    888  888 .d888888 888  888       "888 888    .d888888 888  888  888 888  888 888    888 888  888 888  888 888    888 888  888
 888 d88P 888     888 888  888 Y88b.  888 d88P 888  888 888  888 Y88b  d88P Y88b.  888  888 888  888  888 888 d88P Y88b  d88P Y88..88P 888  888 888    888 Y88b 888
 88888P"  888     888 888  888  "Y888 88888P"  "Y888888 888  888  "Y8888P"   "Y888 "Y888888 888  888  888 88888P"   "Y8888P"   "Y88P"  888  888 888    888  "Y88888
 888                                  888                                                                 888                                                   888
 888                                  888                                                                 888                                              Y8b d88P
 888                                  888                                                                 888                                               "Y88P"
 */
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
/*


 d8b          888    .d8888b.                                              888     888         888
 Y8P          888   d88P  Y88b                                             888     888         888
 888   Y88b.                                                  888     888         888
 88888b.  888d888 888 88888b.  888888 "Y888b.    .d88b.  88888b.  .d8888b   .d88b.  888d888 Y88b   d88P 8888b.  888 888  888  .d88b.  .d8888b
 888 "88b 888P"   888 888 "88b 888       "Y88b. d8P  Y8b 888 "88b 88K      d88""88b 888P"    Y88b d88P     "88b 888 888  888 d8P  Y8b 88K
 888  888 888     888 888  888 888         "888 88888888 888  888 "Y8888b. 888  888 888       Y88o88P  .d888888 888 888  888 88888888 "Y8888b.
 888 d88P 888     888 888  888 Y88b. Y88b  d88P Y8b.     888  888      X88 Y88..88P 888        Y888P   888  888 888 Y88b 888 Y8b.          X88
 88888P"  888     888 888  888  "Y888 "Y8888P"   "Y8888  888  888  88888P'  "Y88P"  888         Y8P    "Y888888 888  "Y88888  "Y8888   88888P'
 888
 888
 888
 */
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

