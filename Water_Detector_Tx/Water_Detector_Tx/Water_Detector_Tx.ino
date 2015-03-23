/*
Wireless Water Detector Trasnmitter
Compile on IDE 1.6.x w/panStamp patch
Board: panStamp AVR

Link to panStamp libraries on GitHub: http://git.io/pkhC
Link to panStamp Wiki, API section: http://github.com/panStamp/panstamp/wiki/panStamp-API

 
To Do:
Add transmitter name to data that's sent to panstamp - maybe 
Start Tx addresses at zero, this way in the Rx sketch, you can use an array and make the array element the Tx address. You want array to start at zero

 
Inputs:
 Wet/Dry status from sponge
 Temperature from OneWire DS18B20
 Status button - when pressed RGB LED will display status
 Address selector rotary dip switch
 
Outputs:
 3 PWM for RGB LED

Data to Transmit (10 bytes):
byte 0: Rx ID - ID of panStamp we are sending data too
byte 1: Tx ID - ID of this panStamp
byte 2: Wet/Dry Status from sponge/Op-amp. Wet = true, Dry = false
byte 3-4: Air temperature
Byte 5-6: battery voltage in milliamps. Battery voltage is measured with code, not an input pin
Byte 7-8: Spare
Byte 9: Checksum
 
panStamp will sleep for 8 second, sleepWd(), wake up and transmit data, about 0.4 seconds, then repeat.
Average current draw is 1mA (about 20mA when awake and 0.009 mA when asleep)
CR123 batteries will last several months


=== I/O ===
D3 - Status LED, Blu, PWM
D4 - Water detector - sponge with op-amp
D5 - Status LED, Grn, PWM
D6 - Status LED, Red, PWM
D8 - 1Wire temperature sensor input
D9 - Status button input
A0-3 - BCD switch for address selector

 
Change Log:
07/04/14 v2.00 - Made ledColor an enum, added spare data field and checksum
02/26/15 v2.01 - Moved voltage calibration from main sketch to this one. Changed some variable names, formatting. Support for 3rd wireless transmitter
                 Started adding support for BCD address switch.  Added code to save voltage calibration to EEPROM
03/08/15 v2.02 - Upgraded to work with latest panStamp API library. Lots of formatting and comment changes.
                 Removed readVcc() function and replaced with panstamp.getVcc().  Removed unused files: Water_detector_Tx_Library.h/cpp
                 Changed addresses of panStamps transmitters to start at 0 - bacause address ID is used in as array element number
03/09/15 v2.03 - Trying to get sketch to compile in Xcode
03/22/15 v2.04 - added print statements to voltage calibration section
*/

#define VERSION  "2.04"

const byte g_Tx_Address         = 2;  // Master Bath = 0, Guest = 1, First Floor  = 2.  SRG - delete after BCD switch is hooked up

const bool SAVE_VOLTAGE_CALIB = false; // set to true to save voltage calibration offset (g_volt_calibration) to EEPROM.  Just do this once right after calibration offset is calculated with a voltmeter
     int16_t g_volt_calibration = 25;  // Master = -71, Guest = -40, First Floor = 0


#include <Arduino.h>
#include <HardwareSerial.h>
#include <EEPROM.h>              // panStamp address is saved to EEPROM http://www.arduino.cc/en/Reference/EEPROM
#include "OneWire.h"             // http://www.pjrc.com/teensy/td_libs_OneWire.html
#include "DallasTemperature.h"   // http://milesburton.com/Main_Page?title=Dallas_Temperature_Control_Library


#define WET LOW                  // When digital input from sponge/Op-Amp is LOW, the sponge is wet
#define TEMPERATURE_PRECISION 9  // sets precision of DS18B20 1-wire temperature sensor

const byte PIN_LED_RED =          6;  // RGB LED Red, PWM pin
const byte PIN_LED_GRN =          5;  // RGB LED Green, PWM pin
const byte PIN_LED_BLU =          3;  // RGB LED Blue, PWM pin
const byte PIN_STATUS_BUTTON =    9;  // Status pushbutton input pin
const byte PIN_SPONGE  =          4;  // Input pin from op-amp connected to the sponge
const byte PIN_TEMPERATURE =      8;  // Data pin for 1-Wire temperature sensor
const byte PIN_ADDR_BCD1    =    A0;  // Rotary BCD switch pin 1
const byte PIN_ADDR_BCD2    =    A0;  // Rotary BCD switch pin 2
const byte PIN_ADDR_BCD4    =    A0;  // Rotary BCD switch pin 4
const byte PIN_ADDR_BCD8    =    A0;  // Rotary BCD switch pin 8


const byte DATA_LENGTH_PS =                10;  // number of bytes in wireless packet going to receiver
const byte CHECKSUM_BYTE = DATA_LENGTH_PS - 1;  // packet checksum (last byte in packet)
const byte EEPROM_ADDR_CALIB =            100;  // Starting EEPROM address used to store for voltage calibration offset


// Color settings for LED
// Green = volts ok, yellow = volts getting low, red = low volts, blue = water detected
enum ledColor_t { GREEN, YELLOW, RED, BLUE, WHITE };


// Setup a 1-Wire instances to communicate 1-Wire DS18B20 temperature sensor
OneWire oneWire(PIN_TEMPERATURE);
DallasTemperature tempSensor(&oneWire);


// panStamp config
const byte g_RF_Channel =               0;  // panStamp channel
      byte g_psNetworkAddress[] = {46, 0};  // panStamp network address, aka SyncWord.  Tx and Rx must have same network address
const byte g_psReceiverAddress =       99;  // panStamp receiver address
const uint16_t LOW_VOLT_LIMIT =      2500;  // Minimum millivolts that panStamp can operate.  If volts is below this, don't send data

// Function Prototypes
bool isWet();
byte getDeviceAddress();          // returns panStamp address of this transmitter by reading BCD rotary switch
void blinkLED(ledColor_t color); 



//============================================================================
//============================================================================
void setup()
{
  
  Serial.begin(9600);
  Serial.print("Wireless Water Detector Transmitter\n\rVersion: ");
  Serial.println(VERSION);
  
  delay(1000);

  // Setup digital I/O pins
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GRN, OUTPUT);
  pinMode(PIN_LED_BLU, OUTPUT);
  pinMode(PIN_ADDR_BCD1, INPUT_PULLUP);  // Rotary BCD switch to select panStamp address
  pinMode(PIN_ADDR_BCD2, INPUT_PULLUP);
  pinMode(PIN_ADDR_BCD4, INPUT_PULLUP);
  pinMode(PIN_ADDR_BCD8, INPUT_PULLUP);
  
  pinMode(PIN_TEMPERATURE, INPUT);
  pinMode(PIN_STATUS_BUTTON,  INPUT_PULLUP);

  Serial.print("Tx Address: ");
  Serial.println(getDeviceAddress());
  
  // initialize panStamp radio
  panstamp.radio.setChannel(g_RF_Channel);
  panstamp.radio.setSyncWord(g_psNetworkAddress);    // Set network address
  panstamp.radio.setDevAddress(getDeviceAddress());  // Set Tx address for this panStamp
  
  panstamp.radio.setTxPowerAmp(PA_LongDistance);     // Turns on high power mode. PA_LowPower is the default 

  // Start up the library for temp sensor
  tempSensor.begin();

  byte volt_calib_lsb, volt_calib_msb;
  const byte CALIB_DATA_SAVED = 111;  // calibration status, 111 = calibration data has been saved, anything else it hasn't
  if( SAVE_VOLTAGE_CALIB )
  {
    // Save voltage (mV) calibration offset to EEPROM
    volt_calib_msb = g_volt_calibration >> 8 & 0xff;
    volt_calib_lsb = g_volt_calibration & 0xff;
    EEPROM.write(EEPROM_ADDR_CALIB,   CALIB_DATA_SAVED);  // So sketch can detect if calib data has been saved to EEPROM
    EEPROM.write(EEPROM_ADDR_CALIB + 1, volt_calib_lsb);
    EEPROM.write(EEPROM_ADDR_CALIB + 2, volt_calib_msb);
    Serial.print("Saved new voltage calibration offset: ");
    Serial.print(g_volt_calibration);
    Serial.println(" mV");
    blinkLED(WHITE); // Blink status light
    blinkLED(WHITE); 
  }
  else 
  {
    // read voltage (mV) calibration from EEPROM
    if ( EEPROM.read(EEPROM_ADDR_CALIB) == CALIB_DATA_SAVED )  // check to see if calibration has been saved to EEPROM
    { 
      // Calibration data has previously been saved to EEPROM, so go ahead and read it
      volt_calib_lsb = EEPROM.read(EEPROM_ADDR_CALIB + 1);
      volt_calib_msb = EEPROM.read(EEPROM_ADDR_CALIB + 2);
      g_volt_calibration = volt_calib_msb << 8;
      g_volt_calibration |= volt_calib_lsb;
      Serial.print("Voltage calibration is: ");
      Serial.print(g_volt_calibration);
      Serial.println(" mV");
      blinkLED(GREEN); // Blink status light green 
      blinkLED(GREEN);
    }
    else // there is no calibration data in EEPROM
    { 
      g_volt_calibration = 0; 
      Serial.println("No voltage calibration set");
      blinkLED(RED); // Blink status red 
      blinkLED(RED);
    }  
  }
  
} // end setup()


//============================================================================
// Main loop.  Read temp sensor, wet/dry status, battery voltage
// Sleeps 8 seconds, then wakes up to send data, then goes to sleep again
//============================================================================
void loop()
{
  
  static int16_t airTemp;
  panstamp.sleepSec(8);  // Sleep for 8 seconds. millis() doesn't increment while sleeping See: http://github.com/panStamp/panstamp/wiki/panStamp-API#sleepsec
  delay(100);  // time to wake up

  uint16_t battery = panstamp.getVcc(); // Read battery voltage
  
  tempSensor.requestTemperatures();         // Send the command to get temperatures
  airTemp = tempSensor.getTempFByIndex(0);  // Read temp sensor
  if ( airTemp > 100 || airTemp < 0)
  { airTemp = -1; }  // got invalid temp from 1wire

  // Send data only if voltage is greater than LOW_VOLT_LIMIT
  // If voltage is lower then this, panStamp doesn't work properly and send out bad data
  if ( battery >= LOW_VOLT_LIMIT )
  {
    CCPACKET psPacket;
    psPacket.length = DATA_LENGTH_PS;        // # bytes that make up packet to transmit
    psPacket.data[0] = g_psReceiverAddress;  // Address of panStamp Rx data is sent too. THIS IS REQUIRED BY THE CC1101 LIBRARY
    psPacket.data[1] = getDeviceAddress();   // Address of this panStamp Tx
    psPacket.data[2] = isWet();              // Read wet-dry status
    psPacket.data[3] = airTemp >> 8 & 0xff;  // High byte - shift bits 8 places, 0xff masks off the upper 8 bits
    psPacket.data[4] = airTemp      & 0xff;  // Low byte, just mask off the upper 8 bits
    psPacket.data[5] = battery >> 8 & 0xff;  // High byte - shift bits 8 places, 0xff masks off the upper 8 bits
    psPacket.data[6] = battery      & 0xff;  // Low byte, just mask off the upper 8 bits
    psPacket.data[7] = 0x00;                 // Spare
    psPacket.data[8] = 0x00;                 // Spare
    // Calculate checksum
    psPacket.data[CHECKSUM_BYTE] = 0; // clear checksum
    for( byte cs = 0; cs < CHECKSUM_BYTE; cs++)
    { psPacket.data[CHECKSUM_BYTE] += psPacket.data[cs]; }
    
    bool sentStatus = panstamp.radio.sendData(psPacket);  // send the data
  }
    
  // When status button is pressed (LOW) turn on status LED
  // User needs to hold down the button up to 8 seconds because panStamp is probably asleep
  if(digitalRead(PIN_STATUS_BUTTON) == LOW)   
  {
    // LED color reflects voltage level
    if ( battery < 2700 )
    { blinkLED(RED); }
    else if ( battery < 2900 )
    { blinkLED(YELLOW); }
    else
    { blinkLED(GREEN); }
    delay(500);
    
    // Make LED blue if sponge is wet
    if(isWet() == true)
    { 
      blinkLED(BLUE);
      delay(500);
    }
  }
  
}  // end loop()


//=======================================================================
// Read the wet/dry status of sponge input
// Return true if sponge is wet
//=======================================================================
bool isWet()
{
  // Loop until two consecutive reading are the same
  bool firstreading;
  bool secondreading;
  do 
  {
    firstreading = digitalRead(PIN_SPONGE);
    delay(10);
    secondreading = digitalRead(PIN_SPONGE);
  } while (firstreading != secondreading);
  
  if (firstreading == WET)
  { return true; } 
  else
  { return false;}
  
} // end isWet()


//=======================================================================
// Read BCD switch to get address for this transmitter
//=======================================================================
byte getDeviceAddress()
{

  byte txAddress = 0;
  
  // Read BCD switch and set address
  if (digitalRead(PIN_ADDR_BCD1))
  { txAddress |= 0x01; }
  if (digitalRead(PIN_ADDR_BCD2))
  { txAddress |= 0x02; }
  if (digitalRead(PIN_ADDR_BCD4))
  { txAddress |= 0x04; }
  if (digitalRead(PIN_ADDR_BCD8))
  { txAddress |= 0x08; }
//srg  return txAddress;
  
  
// srg, once BCD is connected, delete the code below
  return g_Tx_Address;
  
}  // end getDeviceAddress()


//============================================================================
// Blink LED for 1/2 second.  color is passed to this function
//============================================================================
void blinkLED(ledColor_t color)
{
  byte redColor;
  byte grnColor;
  byte bluColor;
  
  // Set LED color
  switch (color)
  {
    case RED:
      redColor = 255;
      grnColor = 0;
      bluColor = 0;
      break;
    case GREEN:
      redColor = 0;
      grnColor = 255;
      bluColor = 0;
      break;
    case YELLOW:
      redColor = 255;
      grnColor = 255;
      bluColor = 0;
      break;
    case BLUE:
      redColor = 0;
      grnColor = 0;
      bluColor = 255;
      break;
    default:  // White
      redColor = 255;
      grnColor = 255;
      bluColor = 255;
      break;
  } 

  analogWrite(PIN_LED_RED, redColor);
  analogWrite(PIN_LED_GRN, grnColor);
  analogWrite(PIN_LED_BLU, bluColor);
  
  delay(500);
  
  // Turn LED off
  analogWrite(PIN_LED_RED, 0);
  analogWrite(PIN_LED_GRN, 0);
  analogWrite(PIN_LED_BLU, 0);
  delay(100);

} // end blinkLED()




