/*
panStamp Tx for Water detector
  
Simple panStamp test - doesn't use SWAP protocal
Source: http://www.panstamp.org/forum/showthread.php?tid=22


To do
 How to wake panStamp with input button

Changes for v2 PCB
 Use square RGB Led, sparkfun COM-11679
 Connect DS18B20 power to an output pin and use to turn it on/off - if V1 draws too much power
 Move batteries .08" up
 put Led to the right of status pushbutton
 move on/off switch up and put text below switch
 move temp sensor to LEDs old position
 use 1/4 watt resistors, not 1/8
 Use bigger wirepads for probes that go into the sponge
 
Inputs:
Wet/Dry status from sponge
Temperature from OneWire DS18B20
Status button - when pressed LED will display status

Outputs:
3 PWM for RGB LED


Data to Transmit (7 bytes):
byte 0: Rx ID - ID of panStamp we are sending data too
byte 1: Tx ID - ID of this panStamp
byte 2: Wet/Dry Status from sponge/Op-amp. Wet = true, Dry = false
byte 3-4: Air temperature
Byte 5-6: battery voltage in milliamps. Battery voltage is measured with code, not an input pin

panStamp will sleep for 8 second, sleepWd(), wake up and transmit data, about 0.4 seconds, then repeat.
Average current draw is 1mA (about 20mA when awake and 0.009 mA when asleep)
One CR123 battery should last 2 months

For temperature sensor, use a TMP36.  It only draws 0.05mA.
Pinout: http://learn.adafruit.com/system/assets/assets/000/000/471/small360/tmp36pinout.gif?1340740328


=== I/O ===
D3 - Status LED, Blu, PWM
D4 - Water detector - sponge an op-amp
D5 - Status LED, Grn, PWM
D6 - Status LED, Red, PWM
D8 - 1Wire temperature sensor input
D9 - Status button input

panStamp Pinout
     
      ANT
GND         GND
D8          GND
D9          D7
A0          D6
A1          D5
A2          D4
GND         D3
A3          D1
A4 SCA      D0
A5 SCL      GND
A6          VCC
A7          RST

*/

// Comment out all but one DEVICE_xxx.  This sets the devide ID for the panStamp
#define DEVICE_MASTER_BATH   // gray antenna
// #define DEVICE_GUEST_BATH  // orange anteanna

#include <Arduino.h>
#include "EEPROM.h"              // panStamp address is saved to EEPROM http://www.arduino.cc/en/Reference/EEPROM
#include "cc1101.h"              // http://code.google.com/p/panstamp/source/browse/trunk/arduino/libraries/panstamp/cc1101.h
#include "panstamp.h"            // http://code.google.com/p/panstamp/source/browse/trunk/arduino/libraries/panstamp/panstamp.h
                                 // http://code.google.com/p/panstamp/wiki/PANSTAMPclass
#include <OneWire.h>             // http://www.pjrc.com/teensy/td_libs_OneWire.html
#include <DallasTemperature.h>   // http://milesburton.com/Main_Page?title=Dallas_Temperature_Control_Library


#define WET LOW    // When digital input from sponge/Op-Amp is LOW, the sponge is wet
#define TEMPERATURE_PRECISION 9

byte const LED_Red =            6;  
byte const LED_Grn =            5;  
byte const LED_Blu =            3;  
byte const statusBtn =          9;  // Status pushbutton
byte const spongeInD =          4;  // Digital Input D3
byte const bytesToSesnd =       7;  // bytes to send to receiving panStamp
byte const tempSensor =         8;  // One Wire temp sensor

// color setting for LED
byte const YELLOW = 1;  // Volts starting to get low
byte const GREEN =  2;  // volts okay
byte const RED =    3;  // volts low
byte const BLUE =   4;  // Sponge is wet

CC1101 cc1101;   // http://code.google.com/p/panstamp/wiki/CC1101class

// Setup a oneWire instances to communicate OneWire temp sensor
OneWire oneWire(tempSensor); // strand is under kitchen and dining room, IDs 0-16
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);


// The networkAdress of sender and receiver must be the same
// in the cc1101 documentation this byte is called syncword
// in the SWAP world of the panStamp it is called networkAddress
byte networkAddress =    46;
byte receiverAddress =   99;
#ifdef DEVICE_MASTER_BATH
  byte senderAddress =    1;  // For master bath, orange antenna
#endif
#ifdef DEVICE_GUEST_BATH
  byte senderAddress =    2;  // For Guest bath, gray antenna
#endif

// Function Prototypes
void statusLight(byte color);
bool isWet();
unsigned int readVcc();

//============================================================================
void statusLight(byte color)
{
  byte redColor;
  byte grnColor;
  byte bluColor;
  
  // Set color
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
    default:  // brown
      redColor = 165;
      grnColor = 42;
      bluColor = 42;
      break;
  } 

  analogWrite(LED_Red, redColor);
  analogWrite(LED_Grn, grnColor);
  analogWrite(LED_Blu, bluColor);
  
  delay(500);
  
  // Turn LED off
  analogWrite(LED_Red, 0);
  analogWrite(LED_Grn, 0);
  analogWrite(LED_Blu, 0);

} // statusLight()


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
  delay(1000);
  
  // Setup digital I/O pins
  pinMode(LED_Red, OUTPUT);
  pinMode(LED_Grn, OUTPUT);
  pinMode(LED_Blu, OUTPUT);
    
  pinMode(tempSensor, INPUT);
  pinMode(statusBtn,  INPUT_PULLUP);
  
  
  // Initialize the CC1101 RF Chip
  cc1101.init();
  cc1101.setSyncWord(&networkAddress, false);   // true saves address to EEPROM
  cc1101.setDevAddress(senderAddress, false);   // true saves address to EEPROM

  // Start up the library for temp sensors
  sensors.begin();

  // Blink status light green
  statusLight(GREEN);

} // setup()

//============================================================================
// continuously reads packets, looking for RX16 only
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
// Sleeps 8 seconds, then wakes up to send data.  That's is
//============================================================================
void loop()
{
  
  static uint16_t airTemp; 
  static int debugcnt;
  panstamp.sleepWd(WDTO_8S);  // Sleep for 8 seconds. millis() doesn't increment while sleeping

    sensors.requestTemperatures(); // Send the command to get temperatures
    delay(100);
    airTemp = sensors.getTempFByIndex(0);  // Read temp sensor
 

  CCPACKET data;
  data.length = bytesToSesnd;                     // # bytes that make up packet to transmit
  data.data[0] = receiverAddress;                 // Address of panStamp Receiver we are sending too. THIS IS REQUIRED BY THE CC1101 LIBRARY
  data.data[1] = senderAddress;                   // Address of this panStamp Tx
  data.data[2] = isWet();                         // Read wet-dry status
  data.data[3] = airTemp >> 8 & 0xff;             // High byte - shift bits 8 places, 0xff masks off the upper 8 bits
  data.data[4] = airTemp & 0xff;                  // Low byte, just mask off the upper 8 bits
  uint16_t battery = readVcc();                   // Read battery voltage
  data.data[5] = battery >> 8 & 0xff;             // High byte - shift bits 8 places, 0xff masks off the upper 8 bits
  data.data[6] = battery & 0xff;                  // Low byte, just mask off the upper 8 bits

  
  cc1101.sendData(data); // Send out data
    
  // When status button is pressed (LOW) turn on status LED
  // Need to hold down up to 8 seconds because panStamp is asleep most of the time
  if(digitalRead(statusBtn)  == LOW)   
  {
    // LED should reflect voltage level
    if(battery < 280)
    { statusLight(RED); }
    
    if(battery < 300)
    { statusLight(YELLOW); }
    else
    { statusLight(GREEN); }
    
    // Make LED blue if sponge is wet
    if(isWet() == true)
    { statusLight(BLUE); }
  }
  
   
}  // loop()


//=======================================================================
/*
d8b          888       888          888    
Y8P          888   o   888          888    
             888  d8b  888          888    
888 .d8888b  888 d888b 888  .d88b.  888888 
888 88K      888d88888b888 d8P  Y8b 888    
888 "Y8888b. 88888P Y88888 88888888 888    
888      X88 8888P   Y8888 Y8b.     Y88b.  
888  88888P' 888P     Y888  "Y8888   "Y888 
*/
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
    firstreading = digitalRead(spongeInD);
    delay(10);
    secondreading = digitalRead(spongeInD);
  } while (firstreading != secondreading);
  
  if (firstreading == WET)
  { return true; } 
  else
  { return false;}
  
} // isWet()


//=======================================================================
/*
                              888 888     888                  
                              888 888     888                  
                              888 888     888                  
888d888 .d88b.   8888b.   .d88888 Y88b   d88P  .d8888b .d8888b 
888P"  d8P  Y8b     "88b d88" 888  Y88b d88P  d88P"   d88P"    
888    88888888 .d888888 888  888   Y88o88P   888     888      
888    Y8b.     888  888 Y88b 888    Y888P    Y88b.   Y88b.    
888     "Y8888  "Y888888  "Y88888     Y8P      "Y8888P "Y8888P 
*/
// Source: http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
// Returns battery vols in mA
//=======================================================================
unsigned int readVcc() 
{
  
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return (unsigned int) result; // Vcc in millivolts
  
} // readVcc()

