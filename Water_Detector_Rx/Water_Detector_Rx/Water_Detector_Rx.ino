/*
 This receiver listens for data coming from the wireless leak detector transmitters.  This sketch sends wireless
 data to the main Arduino via I2c.
 
 IDE 1.6.x
 Board: panStamp AVR

 
Link to panStamp libraries on GitHub: http://git.io/pkhC
Link to panStamp Wiki, API section: http://github.com/panStamp/panstamp/wiki/panStamp-API


Forum question about new api
http://www.panstamp.org/forum/showthread.php?tid=4044


 
To Do:
Use array for struct WirelessWaterDetector

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

 
Change Log
07/04/14  v2.00 - added checksum to panStamp and I2C data
02/24/15  v2.01 - add first floor bathroom, use stuct to hold data
03/07/15  v2.02 - Upgraded to latest panStamp API 
*/

#define VERSION "2.02"

#include <Arduino.h> 
#include <HardwareSerial.h>  // Required by IDE 1.6.x
#include <EEPROM.h>          // http://www.arduino.cc/en/Reference/EEPROM
#include <Wire.h>            // http://arduino.cc/it/Reference/Wire

// This gets rid of compiler warning: Only initialized variables can be placed into program memory area
#undef PROGMEM
#define PROGMEM __attribute__(( section(".progmem.data") ))

// Two LEDs are used to show panStamp and I2C communication
#define LED_RX_DATA     4  // Flashes when there is I2C or Panstamp communication


// I2C Config
const byte I2C_ADDR_SLAVE =                21;  // I2C Slave address of this device
const byte I2C_DATA_LENGTH =               10;  // Bytes in I2C packet
const byte CHECKSUM_I2C = I2C_DATA_LENGTH - 1;  // checksum is the last byte in array
byte g_I2C_Packet[I2C_DATA_LENGTH];             // Array to hold data sent over I2C to main Arduino

// panStamp config
const     byte g_pS_Channel =                 0;  // panStamp RF channel
          byte g_pS_Address_Network[] = {46, 0};  // panStamp network address, aka SyncWord. All Tx and Receiver must be the same address
const     byte g_pS_Address_Rx =             99;  // receiver address of this panStamp
          byte g_lastTxToSendData =           0;  // Address of the most recent wireless transmitter that sent data
const     byte DATA_LENGTH_PS =              10;  // Bytes in panStamp packet
const     byte CHECKSUM_PS = DATA_LENGTH_PS - 1;  // Checksum is last byte in packet array 
const uint32_t PANSTAMP_TIMEOUT =        120000;  // 2 minute timeout for panStamps

     
const byte ADDR_BATH_MASTER =  1;  // panStamp device address for 2nd floor master bath
const byte ADDR_BATH_GUEST =   2;  // panStamp device address for 2nd floor guest bath
const byte ADDR_BATH_FIRST =   3;  // panStamp device address for 1st floor bathroom


// Create sturcture to hold info on remote wireless water detectors
struct WirelessWaterDetector
{
  bool     isOnline;         // true if transmitter is sendign data
  uint8_t  addressTx;        // Transmitter address
  bool     isWet;            // is sponge wet
  int16_t  tempF;            // temperature 
  uint16_t millivolts;       // battery volts in millivolts
  uint32_t lastRxTimestamp;  // timestamp of last received packet
  byte     rssi;             // signal strength
  byte     lqi;              // link quality index
};

struct WirelessWaterDetector g_bathFirstFloor;
struct WirelessWaterDetector g_bathMaster;
struct WirelessWaterDetector g_bathGuest;
struct WirelessWaterDetector g_unknownSensor;

volatile boolean g_pSpacketAvailable = false;  // Flag that a wireless packet has been received
            byte g_TxAddress_Requested =   0;  // Address of remote transmitter the I2C master is requesting


// Function prototypes
void printpanStampConfig();
void wireRequestEvent();
void radioSignalInterrupt();
void printSensorValues(WirelessWaterDetector& psPkt);


//============================================================================
//============================================================================
void setup()
{
  Serial.begin(9600);
  Serial.print(F("\n\rBegin panStamp Rx setup\n\rVersion "));
  Serial.println(VERSION);
  
  pinMode(LED_RX_DATA, OUTPUT);  // Yellow LED on panel to indicate Rx data received
  digitalWrite(LED_RX_DATA, HIGH);
  delay(500);
  digitalWrite(LED_RX_DATA, LOW);

  Wire.begin(I2C_ADDR_SLAVE);              // Initiate the Wire library and join the I2C bus
  Wire.onRequest(wireRequestEvent);        // Register a function to be called when I2C master requests data from this slave device.
  Wire.onReceive(wireGetAddressEvent);     // This function is called when I2C master tells slave which data it wants.  Master sends the address of the wireless transmitter it wants
  Serial.println(F("I2C initialized\n\r"));

  // Setup the panStamp radio
  panstamp.radio.setChannel(g_pS_Channel);             // panStamp Channel
  panstamp.radio.setSyncWord(g_pS_Address_Network);    // Set network address. Has to be the same for all panStamps on this network
  panstamp.radio.setDevAddress(g_pS_Address_Rx);       // Address of this panStamp
  panstamp.setPacketRxCallback(radioSignalInterrupt);  // Declare radio callback function which runs every time panStamp RF data comes in
  printpanStampConfig();
  Serial.println(F("panStamp Rx setup complete\n\r"));

}  // end setup()


//============================================================================
// Get data from panStamps and put into I2C packet.
// When Master requests data from this slave, the wireRequestEvent() will
// execute and send the data to the other Arduiino
//============================================================================
void loop()
{
  // Get data from remote wirelesss water detectors
  if(g_pSpacketAvailable)
  {
    g_pSpacketAvailable = false;  // reset flag
    blinker(LED_RX_DATA, 1);  // blink LED once to indicate receipt of wireless data
    switch (g_lastTxToSendData)
    {
      case ADDR_BATH_MASTER:
        printSensorValues(g_bathMaster);  // Print packet data
        break;
      case ADDR_BATH_GUEST:
        printSensorValues(g_bathGuest);  // Print packet data
        break;
      case ADDR_BATH_FIRST:
        printSensorValues(g_bathFirstFloor);  // Print packet data
        break;
      default:
        Serial.print(F("packet from unknown Tx addr = "));
        Serial.println(g_lastTxToSendData);
        printSensorValues(g_unknownSensor);  // Print packet data
        break;
    }
 
  }  // packet available

} // end loop()


//============================================================================
// Function runs when panStamp data comes in
//============================================================================
void radioSignalInterrupt(CCPACKET *psPacket)
{
  if (psPacket->length == DATA_LENGTH_PS)
  {
    struct WirelessWaterDetector *ptr_WirelessSensor;  // switch-case below will choose which instance of the structer the data is loaded into
    
    // Validate data with checksum.  
    byte checksum = 0;
    for (int cs = 0; cs < CHECKSUM_PS; cs++)
    { checksum += psPacket->data[cs]; }
    bool checksumPassed = ( checksum == psPacket->data[CHECKSUM_PS] );
   
    // If data is okay, then put it into the structure
    if ( checksumPassed )
    {
      g_pSpacketAvailable = true;  // set the flag indicating new packet came in
      
      g_lastTxToSendData = psPacket->data[1];
      switch (g_lastTxToSendData)
      {
        case ADDR_BATH_MASTER:
          ptr_WirelessSensor = &g_bathMaster;
          break;
        case ADDR_BATH_GUEST:
          ptr_WirelessSensor = &g_bathGuest;
          break;
        case ADDR_BATH_FIRST:
          ptr_WirelessSensor = &g_bathFirstFloor;
          break;
        default:
          ptr_WirelessSensor = &g_unknownSensor;
          break;
      }  // end switch

      // use pointer to put data into the structure
      ptr_WirelessSensor->addressTx       = g_lastTxToSendData;
      ptr_WirelessSensor->isWet           = psPacket->data[2];
      ptr_WirelessSensor->tempF           = psPacket->data[3] << 8;
      ptr_WirelessSensor->tempF          |= psPacket->data[4];
      ptr_WirelessSensor->millivolts      = psPacket->data[5] << 8;
      ptr_WirelessSensor->millivolts     |= psPacket->data[6];
      ptr_WirelessSensor->lastRxTimestamp = millis();
      ptr_WirelessSensor->rssi            = psPacket->rssi; // signal strength
      ptr_WirelessSensor->lqi             = psPacket->lqi;  // link quality index
    } // end checksum passed
  } // end length ok
  

} // end radioSignalInterrupt()


//========================================================================================================================================
// function executes when master sends over the address of the
// wireless water sensor that it wants
//========================================================================================================================================
void wireGetAddressEvent(int numBytes)
{
  g_TxAddress_Requested = Wire.read();
}

//========================================================================================================================================
// function that executes whenever data is requested by master
// this function is registered as an event in setup()
// It sends a 14 byte packet that has data from both panStamps
// If we haven't received data from panStamps in the last 30 minutes, then it won't send any data
//========================================================================================================================================
void wireRequestEvent()
{

  struct WirelessWaterDetector *ptr_Wireless_Sensor;

  switch (g_TxAddress_Requested)
  {
    case ADDR_BATH_MASTER:
      ptr_Wireless_Sensor = &g_bathMaster;
      break;
      
    case ADDR_BATH_GUEST:
      ptr_Wireless_Sensor = &g_bathGuest;
      break;

    case ADDR_BATH_FIRST:
      ptr_Wireless_Sensor = &g_bathFirstFloor;
      break;
      
    default:
      ptr_Wireless_Sensor = &g_unknownSensor;
      break;
  }

  if ((long)( millis() - ptr_Wireless_Sensor->lastRxTimestamp) > PANSTAMP_TIMEOUT )
  { ptr_Wireless_Sensor->isOnline = false; }
  else
  { ptr_Wireless_Sensor->isOnline = true; }

  g_I2C_Packet[0] = ptr_Wireless_Sensor->addressTx;
  g_I2C_Packet[1] = ptr_Wireless_Sensor->isOnline;
  g_I2C_Packet[2] = ptr_Wireless_Sensor->isWet;
  g_I2C_Packet[3] = ptr_Wireless_Sensor->tempF >> 8 & 0xff;       // low byte for temp
  g_I2C_Packet[4] = ptr_Wireless_Sensor->tempF & 0xff;            // high byte for temp
  g_I2C_Packet[5] = ptr_Wireless_Sensor->millivolts >> 8 & 0xff;  // low byte for mV
  g_I2C_Packet[6] = ptr_Wireless_Sensor->millivolts & 0xff;       // high byte for mV
  g_I2C_Packet[7] = 0; // spare
  g_I2C_Packet[8] = 0; // spare
  
  // Calculate checksum
  g_I2C_Packet[CHECKSUM_I2C] = 0;
  for (int cs = 0; cs < CHECKSUM_I2C; cs++)
  { g_I2C_Packet[CHECKSUM_I2C] += g_I2C_Packet[cs]; }
  
  
  // Send data to I2C master
  Wire.write(g_I2C_Packet, I2C_DATA_LENGTH);

} // end wireRequestEvent()


//============================================================================
// Blink LED count times
//============================================================================
void blinker(byte led_pin, int count)
{
  for(int j=1; j<=count; j++)
  {
    digitalWrite(led_pin, HIGH);
    delay(50);
    digitalWrite(led_pin, LOW);
    delay(50);
  }
}  // end blinker()


//=============================================================================================================================================
// Print panStamp config info: Frequence, Channel, Network address, Rx address
//=============================================================================================================================================
void printpanStampConfig()
{

  // Print device setup info
  Serial.print(F("Radio Frequency = "));
  if(panstamp.radio.carrierFreq == CFREQ_868)
  {Serial.println(F("868 Mhz"));}
  else
  {Serial.println(F("915 Mhz"));}
  Serial.print(F("Channel = "));
  Serial.println(panstamp.radio.channel);
  Serial.print(F("Network address = "));
  Serial.println(panstamp.radio.syncWord[0]);
  Serial.print(F("Device address =  "));
  Serial.println(panstamp.radio.devAddress);

}  // end printpanStampConfig()



//=============================================================================================================================================
//  Convert data from panStamp Tx and print - for debugging
//=============================================================================================================================================
void printSensorValues(WirelessWaterDetector& psPkt)
{

  Serial.print("\n\rAddr = ");
  Serial.print(psPkt.addressTx);
  
  Serial.print("\ttemp = ");
  Serial.print(psPkt.tempF);
  
  Serial.print("\tVcc = ");
  Serial.print(psPkt.millivolts);
  
  Serial.print("\tIs Wet = ");
  Serial.print(psPkt.isWet);

  Serial.print("\tRSSI = ");
  Serial.print(psPkt.rssi);

  Serial.print("\tLQI = ");
  Serial.print(psPkt.lqi);


} // end printSensorValues()


