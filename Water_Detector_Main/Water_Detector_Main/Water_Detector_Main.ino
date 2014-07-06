// See Readme.txt file
// directory cd Dropbox/Arduino/Water_Detector/ 
// Uses Leonardo

/*  
To do:


Rev history
v2.00 05/28/14 - Changed low temp alarm logic so it only sends low temp tweet if voltage > 2500 mA.  Get false alarms when 
                 voltage is below this
v2.01 06/27/14 - Updated tx offline message to display hrs or sec depending on how long it was offline.  Fixed 
                 updateOledDisplay so Tx offline messages would display
v2.02 07/04/14 - Added checksum to data packet, renamed some constants
 
*/


#include "Arduino.h"
#include <SPI.h>             // Allows you to communicate with SPI devices. See: http://arduino.cc/en/Reference/SPI
#include <Ethernet.h>        // http://arduino.cc/en/Reference/Ethernet
#include <Twitter.h>         // http://arduino.cc/playground/Code/TwitterLibrary
#include <I2C.h>             // http://github.com/rambo/I2C
#include <Tokens.h>          // Contains Twitter token
#include <Adafruit_GFX.h>    // http://github.com/adafruit/Adafruit-GFX-Library
#include <SSD1306_I2C_DSS.h> // For OLED  https://github.com/Scott216/SSD1306_I2C_DSS
#include "Water_Detector_Main_Library.h"    // Include application, user and local libraries
#include <avr/pgmspace.h>    // Store data in flash.  http://arduino.cc/en/Reference/PROGMEM

// #define PRINT_DEBUG      // Comment this out to turn off verbose printing

byte mac[] = { 0x90, 0xA2, 0xDA, 0xEF, 0x46, 0x81 };
byte ip[] =  { 192, 168, 46, 81 };

// Analog inputs 0 & 1 are configured as Digitol I/O.
//      Name              I/O    Description
#define ISWETOUTPUT         0   // Turns on Red LED and relay
#define OLED_RESET          1
// Reserved                 2      I2C
// Reserved                 3      I2C
// Reserved                 4      SD Card
//                          5      Unused
#define HOTTUBFILTER        6   // Sponge in crawlspace by hot tub filter and pump
#define HOTTUBBACK          7   // Sponge in crawlspace behind hot tub
#define WATERTANK           8   // Sponge in crawlspace corner by water tank
#define KITCHENSINK         9   // Sponge under kitchen sink
// reserved                10      Slave select
#define TBD2               11   // Future use
#define TBD1               12   // Future Use
#define WATERHEATER        A0   // Sponge under hot water heater
#define BOILER             A1   // Sponge next to boiler
#define FRIG               A2   // Sponge behind refrigerator
#define DISHWASHER         A3   // Sponge behind dishwasher
#define FIRSTFLOORBRSINK   A4   // Sponge in first floor bathroom sink
#define WASHINGMACH        A5   // Sponge next to washing machine

#define WET HIGH       // When a sponge is wet, the digital input is HIGH
#define DRY LOW        // When a sponge is dry, the digital input is LOW

const uint32_t MINUTE  =                   60000;  // milliseconds in a minute
const uint32_t DOUBLE_CHECK_DELAY =   2 * MINUTE;  // Delay after sensor first detects water to check again after a few minutes
const uint32_t DRYING_DELAY =       180 * MINUTE;  // 3 hour delay when sponges are drying out before twitter it is updated. Prevents extra alerts from going out

#define ADDR_I2C_OLED           0x3C   // OLED Display I2C address
#define ADDR_I2C_PANSTAMP         21   // I2C Slave address of panStamp Rx
const byte ADDR_MASTER_BATH =      1;  // panStamp device address for 2nd floor master bath
const byte ADDR_GUEST_BATH =       2;  // panStamp device address for 2nd floor guest bath
const byte NUM_WIRELESS_SENSORS =  2;  // number of panStamp transmitters on this network
const byte NUM_WIRED_SENSORS =    12;  // Number of wired water detector inputs
const byte DATA_LENGTH =          10;  // Packets sent by each panStamp
bool isAnythingWet =           false;  // Flags if anything is wet


// Use arrays to hold input status.  Total number of inputs are wired plus wireless sensors
bool  InputState[NUM_WIRED_SENSORS + NUM_WIRELESS_SENSORS];     // Input reading, HIGH when water is present LOW when it's not
bool WaterDetect[NUM_WIRED_SENSORS + NUM_WIRELESS_SENSORS];     // Sensor state after delay to make sure it's really wet or dry
uint32_t weeklyHeartbeatTimer = 0;                               // mS until Sunday at noon, at which time a tweet will go out to verify sketch is still running  static uint8_t TweetCounter;  // prevent tweets from getting to high

// Define typedef structurs for panStamp data from remote sensors.  Typedef definition is in LocalLibrary.h
RemoteSensorData_t masterBath;
RemoteSensorData_t guestBath;

// Token for Twitter Account
Twitter twitter(TWITTER_TOKEN);

// Put input pin numbers in array so you can use loops to read wired inputs
int InputPinNum[] = { FIRSTFLOORBRSINK, WASHINGMACH, TBD1, TBD2, WATERHEATER, BOILER, FRIG, DISHWASHER, KITCHENSINK, HOTTUBFILTER, HOTTUBBACK, WATERTANK };


// Function Prototypes
bool ReadRFSensors(RemoteSensorData_t* rfsensor, byte panStampID);
void SendAlert(byte SensorArrayPosition, bool IsWet);
void ProcessSensors(void);
int  SendTweet(char msgTweet[]);
void PrintStates();
uint32_t getMsUntilSundayNoon(uint8_t *ntpTime);  // mS until Sunday noon - for weeklyheartbeat timer
void software_Reset();
bool setupNTPTime();
bool getTime(uint8_t *ntpTime);
void parseTimePacket(uint8_t *ntpTime);
unsigned long sendNTPpacket(IPAddress& address);
void updateOledDisplay();


// NPT servers work as of 4/17/13
// List of servers: http://tf.nist.gov/tf-cgi/servers.cgi
// don't query more then 4 seconds
IPAddress timeServer1( 216, 171, 112,  36); 
IPAddress timeServer2( 206, 246, 122, 250);
IPAddress timeServer3( 64,   90, 182,  55);  

const int NTP_PACKET_SIZE= 48;           // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE];      //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

// Instantiate OLED dispplay object
Adafruit_SSD1306 display(OLED_RESET);


//=========================================================================================================================
//=========================================================================================================================
void setup ()
{  

  // Start I2C communication
  I2c.begin();
  I2c.timeOut(30000);  // set I2C timeout to 30 seconds
  I2c.pullup(0);       // disable internal pullup resistors on I2C pins, don't want to pull-up to 5 volts because of panStamp

  display.begin(SSD1306_SWITCHCAPVCC, ADDR_I2C_OLED);  
  display.clearDisplay();  
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,10);
  display.println("Leak Detect v2.02");
  display.display();

  Serial.begin(9600);
  #ifdef PRINT_DEBUG
    while (!Serial && millis() < 6000) {}  // for Leonardo wait here for a few seconds for serial monitor connection
    Serial.println(F("Leak Detect v2.02"));
  #endif
  
  // Initialize Ethernet connection and UDP
  uint8_t ntpTime[6];
  bool EthernetOK = setupNTPTime();
  delay(2000);
  if(EthernetOK)
  {
    // Get the NTP Time
    if(getTime(ntpTime))
    {
      // Set weekly countdown timer - Sunday noon
      weeklyHeartbeatTimer = millis() + getMsUntilSundayNoon(ntpTime);
      #ifdef PRINT_DEBUG
        Serial.print(F("hours until Sunday noon = "));
        Serial.println(weeklyHeartbeatTimer / (MINUTE * 60UL));
      #endif
    }
    else
    { 
      #ifdef PRINT_DEBUG
        Serial.println(F("Failed to get NTP time in setup()")); 
      #endif
    }
  }
  
  
  for(int i = 0; i < NUM_WIRED_SENSORS; i++)
  {  pinMode(InputPinNum[i], INPUT); }
  pinMode(ISWETOUTPUT, OUTPUT);  // Red LED and reed relay
  digitalWrite(ISWETOUTPUT, LOW);
  
  // Initialiaze one shot triggers for messages
  masterBath.lowVoltMsgFlag = false;
  masterBath.lowTempMsgFlag = false;
  guestBath.lowVoltMsgFlag =  false;
  guestBath.lowTempMsgFlag =  false;

  SendTweet("Leak Detector Restarted v2.02");  // Send startup tweet
  
  ProcessSensors();  // Check sensors to see if anything is wet

  display.clearDisplay();
  display.display();

} // end setup()


//=========================================================================================================================
//=========================================================================================================================
void loop ()
{
  static uint32_t checkNtpTimer = millis() + (MINUTE * 60UL * 48UL);  // Check NTP time in 48 hours
  static uint32_t CheckSensorsTimer =    0;                           // Timer to check the sensors every second

  if ( (long)(millis() - CheckSensorsTimer) >= 0 )
  {
    display.clearDisplay();    
    display.setCursor(0,10);
    display.println("CHECKING SENSORS...");
    display.display();
    
uint32_t debugTimer = millis();  // srg debug
    ProcessSensors();  // Check sensors to see if anything is wet
Serial.print("Time to process sensors: ");
Serial.println(millis() - debugTimer);
    
    updateOledDisplay();
    CheckSensorsTimer = millis() + 1000UL; // add 1 second to timer
  }
  
  // Every Sunday at noon send a Tweet to indicate sketch is still running (heartbeat). Send battery voltages from wireless sensors
  if( (long)(millis() - weeklyHeartbeatTimer) >= 0 )
  {
    char tweetMsg[34+1];  // longest text is 34 characters
    sprintf_P(tweetMsg, PSTR("Leak Detector: Master Bath = %d mV"), masterBath.volts);
    SendTweet(tweetMsg);
    sprintf_P(tweetMsg, PSTR("Leak Detector: Guest Bath = %d mV"), guestBath.volts);
    SendTweet(tweetMsg);
    
    weeklyHeartbeatTimer =  millis() + (MINUTE * 60UL * 24UL * 7UL); // add 1 week to heartbeat timer

    #ifdef PRINT_DEBUG
      PrintStates();
    #endif
  }  // end weekly heartbeat

  // Check time once every 2 days and adjust Sunday countdown
  if( (long)( millis() -  checkNtpTimer) > 0 )
  {
    uint8_t ntpRefresh[6];
    
    if( getTime(ntpRefresh) )
    {  
      checkNtpTimer = millis() + (MINUTE * 60UL * 48UL);  // add 48 hours to check NPT timer
      weeklyHeartbeatTimer = millis() + getMsUntilSundayNoon(ntpRefresh); 
    } // Update weekly countdown timer with mS until Sunday noon
    else
    {  
//      SendTweet("Failed to update NTP time");  // Failer to get time from server, send tweet about NTP failure
      checkNtpTimer = millis() + (5UL * MINUTE);  // try again in 5 minutes
    } 
  }
  
  // If anything is wet, turn on LED and relay (they are on the same output
  bool wetOutputState = LOW;
  for(int i = 0; i < NUM_WIRED_SENSORS + NUM_WIRELESS_SENSORS; i++)
  {
    if( WaterDetect[i] == WET )
    { wetOutputState = HIGH; }
  }    
  digitalWrite(ISWETOUTPUT, wetOutputState);  // turn on Red LED and Relay if anything is wet
  
}  // end loop()


//=========================================================================================================================
// Check inputs to see if any sponges are wet
//=========================================================================================================================
void ProcessSensors()
{
  char twitterMsg[60];  // max message actually used is 31 char
  static bool masterTxOfflineMsg = false;    // One shot flag when Master Bath transmitter goes offline
  static bool guestTxOfflineMsg =  false;    // One shot flag when Guest Bath transmitter goes offline
  static uint32_t masterBathOfflineTime = 0; // Time master bath sensor went offline
  static uint32_t guestBathOfflineTime  = 0;  // Time guest bath sensor went offline
  static uint32_t DoubleCheckTime[ NUM_WIRED_SENSORS + NUM_WIRELESS_SENSORS];     // Used to wait a few minutes after a sensor is triggered to check the sensor a 2nd time to see if there is still water. Like button debouncing
  static uint32_t WetToDryDelay[   NUM_WIRED_SENSORS + NUM_WIRELESS_SENSORS];     // Delay used to let sponge dry out before indicating it's dry
  static bool     isWet[           NUM_WIRED_SENSORS + NUM_WIRELESS_SENSORS];     // Used to trigger if an input goes from Dry to Wet.  Trigger will turn on WaterDetectOutput for 10 seconds.

  isAnythingWet = false;  // reset flag
  
  // read hard wired inputs
  for(int i = 0; i < NUM_WIRED_SENSORS; i++)
  {
    // Read hard wired sensors
    // Loop until two consecutive reading are the same
    bool firstreading = DRY;
    bool secondreading = DRY;
    do
    {
      firstreading = digitalRead(InputPinNum[i]);
      delay(10);
      secondreading = digitalRead(InputPinNum[i]);
    } while (firstreading != secondreading);
    InputState[i] = firstreading;
    
    if ( InputState[i] == WET )
    { isAnythingWet = true; }
  }
  
  // Read Remote Master Bath sensor
  if( ReadRFSensors( &masterBath, ADDR_MASTER_BATH ) )
  {
    // If master bath came back online, send a tweet
    if ( masterTxOfflineMsg == true )
    {
      if ( (millis() - masterBathOfflineTime) < 10800000UL ) // < 3 hours
      sprintf_P(twitterMsg, PSTR("Master Bath is back online: %dmV. Offline for %lu sec"), masterBath.volts, (millis() - masterBathOfflineTime) / 1000UL );
      else
      sprintf_P(twitterMsg, PSTR("Master Bath is back online: %dmV. Offline for %lu hrs"), masterBath.volts, (millis() - masterBathOfflineTime) / 3600000UL );
      SendTweet(twitterMsg);
    }

    masterTxOfflineMsg = false; // reset flag for sending Tweeet about wireless sensor being offline
        
    InputState[NUM_WIRED_SENSORS] = masterBath.IsWet;
    if ( masterBath.IsWet == WET )
    { isAnythingWet = true; }

    // Check for low volts
    if( masterBath.online == true && masterBath.volts < 2800 && masterBath.lowVoltMsgFlag == false)
    {
      sprintf_P(twitterMsg, PSTR("Low battery Master Bath: %dmV"), masterBath.volts );
      SendTweet(twitterMsg);
      masterBath.lowVoltMsgFlag = true;
    }
    // Reset low volts flag if volts is over 3000 mV
    if(masterBath.volts > 3000)
    {
      masterBath.lowVoltMsgFlag = false;
    }
    
    // Check for low temperature in Masster Bath
    if( masterBath.online == true && masterBath.temp <= 40 && masterBath.temp >= 20 && masterBath.lowTempMsgFlag == false && masterBath.volts > 2500 )
    {
      sprintf_P(twitterMsg, PSTR("Low temp Master Bath: %d F"), masterBath.temp);
      SendTweet(twitterMsg);
      masterBath.lowTempMsgFlag = true;
    }
    // Reset low temp flag if temp is 50 F
    if(masterBath.temp >= 50)
    {
      masterBath.lowTempMsgFlag = false;
    }
  }
  else
  {
    // If Master bath is offline, send tweet
    if( masterBath.online == false && masterTxOfflineMsg == false )
    {
      strcpy_P(twitterMsg, PSTR("Master Bath is offline"));
      if( SendTweet(twitterMsg) == 200 )
      { masterTxOfflineMsg = true; }
      masterBathOfflineTime = millis(); // Time master bath went offline
    }
  }
  
  // Read Remote Guest Bath sensor
  if( ReadRFSensors( &guestBath, ADDR_GUEST_BATH  ) )
  {
    // If guest bath came back online, send a tweet
    if ( guestTxOfflineMsg == true )
    {
      if ( (millis() - guestBathOfflineTime) < 10800000UL ) // < 3 hours
      { sprintf_P(twitterMsg, PSTR("Guest Bath is back online: %dmV. Offline for %lu sec"), guestBath.volts, (millis() - guestBathOfflineTime) / 1000UL );}
      else
      { sprintf_P(twitterMsg, PSTR("Guest Bath is back online: %dmV. Offline for %lu hrs"), guestBath.volts, (millis() - guestBathOfflineTime) / 3600000UL );}
      SendTweet(twitterMsg);
    }
     guestTxOfflineMsg = false; // reset flag for sending Tweeet about wireless sensor being offline
   
    InputState[NUM_WIRED_SENSORS + 1] = guestBath.IsWet;
    if ( guestBath.IsWet == WET )
    { isAnythingWet = true; }
    
    // Check for low volts
    if(guestBath.online == true &&  guestBath.volts < 2800 && guestBath.lowVoltMsgFlag == false)
    { 
      sprintf_P(twitterMsg, PSTR("Low battery Guest Bath: %dmV"), guestBath.volts );
      SendTweet(twitterMsg);
      guestBath.lowVoltMsgFlag = true;
    }
    // Reset low volts flag if volts is over 3000 mV
    if(guestBath.volts > 3000)
    { guestBath.lowVoltMsgFlag = false; }
    
    // Check for low temperature in Guest Bath
    if(guestBath.online == true &&  guestBath.temp <= 40 && guestBath.temp >= 20 && guestBath.lowTempMsgFlag == false && guestBath.volts > 2500)
    {
      sprintf_P(twitterMsg, PSTR("Low temp in Guest Bath: %d F"), guestBath.temp);
      SendTweet(twitterMsg);
      guestBath.lowTempMsgFlag = true;
    }
    // Reset low temp flag if temp is 50 F
    if(guestBath.temp >= 50)
    { guestBath.lowTempMsgFlag = false; }
  }
  else
  { 
    // If Guest bath is offline, send tweet
    if( guestBath.online == false && guestTxOfflineMsg == false )
    {
      strcpy_P(twitterMsg, PSTR("Guest Bath is offline"));
      if( SendTweet(twitterMsg) == 200 )
      { guestTxOfflineMsg = true; }
      guestBathOfflineTime = millis(); // Time guest bath went offline
    }
  }
  
  // See if any inputs have gone from Dry to Wet
  for(int i = 0; i < NUM_WIRED_SENSORS + NUM_WIRELESS_SENSORS; i++)
  {
    if((InputState[i] == WET) && (isWet[i] == DRY ))
    {
      #ifdef PRINT_DEBUG
        PrintStates();
      #endif
      
      isWet[i] = WET;
      
      #ifdef PRINT_DEBUG
        Serial.print(F("Water detected ID: "));
        Serial.println(i);
        PrintStates();
      #endif
      
      // Set time to check again to make sure it's really wet
      DoubleCheckTime[i] = millis() + DOUBLE_CHECK_DELAY;
    }
  } // End check for Dry to Wet
  
  
  // Double check input after a delay to see it it's still wet
  for(int i = 0; i < NUM_WIRED_SENSORS + NUM_WIRELESS_SENSORS; i++)
  {
    if( (isWet[i] == WET) && (WaterDetect[i] == DRY) && ((long)(millis() - DoubleCheckTime[i]) >= 0) )
    {
      WaterDetect[i] = WET;  // Sponge is still wet after delay
      SendAlert(i , WET);    // One of the inputs is still wet after DOUBLE_CHECK_DELAY.  Send Tweet out
      #ifdef PRINT_DEBUG
        PrintStates();
      #endif
     }
  } // End double check for wet sponge
  
  
  // If input is dry, reset WetFlag[]
  // A sponge isn't determined to be dry unless the input has been dry for DRYING_DELAY (3 hours)
  for(int i = 0; i < NUM_WIRED_SENSORS + NUM_WIRELESS_SENSORS; i++)
  {
    if(InputState[i] == DRY)
    { isWet[i] = DRY; }
    else
    { WetToDryDelay[i] = millis() + DRYING_DELAY; } // If input is WET, reset drying delay
  }
  
  // Check for dried out sponge, after long delay
  for(int i = 0; i < NUM_WIRED_SENSORS + NUM_WIRELESS_SENSORS; i++)
  {
    if( (isWet[i] == DRY) && (WaterDetect[i] == WET) && ((long)(millis() - WetToDryDelay[i]) >= 0))
    {
      // Sensor went from Wet To Dry
      #ifdef PRINT_DEBUG
        PrintStates();
      #endif
      
      WaterDetect[i] = DRY;
      
      SendAlert(i, DRY);
    }
  } // End Wet to Dry
  
} // end ProcessSensors()


//=========================================================================================================================
// Get the data from wireless sensors 
// Data from both sensors is saved in panStampData[], but this function only
// returns data for one sensor at a time depending on the panStampID passed to the function
//=========================================================================================================================
bool ReadRFSensors(RemoteSensorData_t* rfsensor, byte panStampID)
{
  static bool gotI2CPacket = false;  // Flag to indicate I2C packet came in

  int voltCalibration[3]; // voltage calibration, millivolt adjustment.  Uses 3 array elements because zero is not used; panStamp IDs are 1 and 2, so it uses those
  voltCalibration[ADDR_MASTER_BATH] = -71;
  voltCalibration[ADDR_GUEST_BATH ] = -40;
  
  const byte PANSTAMP_OFFLINE =  255;  // if value in panStamp Tx ID byte is 255, it means the wireless sensor is offline or checksum failed
  byte panStampData[DATA_LENGTH * NUM_WIRELESS_SENSORS];   // Array to hold wireless sensor data 
  
  int readstatus = I2c.read(ADDR_I2C_PANSTAMP, NUM_WIRELESS_SENSORS * DATA_LENGTH , panStampData); // read data from panStamp I2C slave
  if(readstatus == 0)
  {  gotI2CPacket = true; }
  
  // If got I2C packet, then  process it
  // First set of bytes of data are Master bath, followed by Guest bath
  if( gotI2CPacket )
  {
    gotI2CPacket = false;  // reset
    byte byteOffset = 0;
    switch (panStampID)
    {
      case ADDR_MASTER_BATH:
        byteOffset = 0;
        break;
      case ADDR_GUEST_BATH :
        byteOffset = DATA_LENGTH;
        break;
    }
    
    // verify checksum
    byte checksum = 0;
    for (int k = 0; k < DATA_LENGTH - 1; k++)
    { checksum += panStampData[k + byteOffset]; }
    if (  panStampData[DATA_LENGTH - 1  + byteOffset] !=  checksum )  // if checksum is on the left, line is always true
    {
      // Checksum failed
      #ifdef PRINT_DEBUG
        Serial.println(F("Checksum failed")); 
      #endif
      rfsensor->ID = panStampID;
      rfsensor->online = false;
      return false;
    }
    
    // Calculate sensor values and return in rfsensor
    // panStamp Rx sketch will put 255 in panStamp Tx ID byte if it hasn't received data from Tx in 30 minutes or if checksum failed in panStamp Rx
    if( panStampData[1 + byteOffset] != PANSTAMP_OFFLINE )
    { // panStamp Tx is ok
      
      // Get millivolts
      int millivolts;
      millivolts  = panStampData[5 + byteOffset] << 8;
      millivolts |= panStampData[6 + byteOffset];
      millivolts = millivolts + voltCalibration[panStampID];
      if(millivolts < 0)
      { millivolts = 0;}
      
      // Get temperature 
      int localTemp;
      localTemp  = panStampData[3 + byteOffset] << 8;
      localTemp |= panStampData[4 + byteOffset];
      if(localTemp > 100 || localTemp < 0 )
      { localTemp = 0;} // check for bad data
      
      // Put panStamp data into rfsensor structure
      rfsensor->online = true;
      rfsensor->ID =     panStampData[1 + byteOffset];
      rfsensor->IsWet =  panStampData[2 + byteOffset];
      rfsensor->volts =  millivolts;
      rfsensor->temp =   localTemp;
      return true;
    }
    else
    { // panStamp Tx is offline
      rfsensor->ID = panStampID;
      rfsensor->online = false;
      return false;
    }
  }
  else // didn't get an I2C packet
  {
    rfsensor->ID = panStampID;
    rfsensor->online = false;
    return false;
  }
  
} // ReadRFSensors()


//=========================================================================================================================
// Based on which sensor went off, and it it changed from dry to wet, or wet to dry, create message for Twitter
//=========================================================================================================================
void SendAlert(byte SensorArrayPosition, bool IsWet)
{
  char alertMsg[17+26+1]; // longest text is 26 characters, add space for time stamp
  if(IsWet == WET)
  {strcpy_P(alertMsg, PSTR("Water Detected - ")); }  // 17 characters
  else
  {strcpy_P(alertMsg, PSTR("Water Dried Up - ")); }
  
  switch(SensorArrayPosition)
  {
    case 0:
      strcat_P(alertMsg, PSTR("First Fl bathroom sink"));
      break;
    case 1:
      strcat_P(alertMsg, PSTR("Washing machine"));
      break;
    case 2:
      strcat_P(alertMsg, PSTR("TBD1"));
      break;
    case 3:
      strcat_P(alertMsg, PSTR("TBD2"));
      break;
    case 4:
      strcat_P(alertMsg, PSTR("Water Heater"));
      break;
    case 5:
      strcat_P(alertMsg, PSTR("Boiler"));
      break;
    case 6:
      strcat_P(alertMsg, PSTR("Frig"));
      break;
    case 7:
      strcat_P(alertMsg, PSTR("Dishwasher"));
      break;
    case 8:
      strcat_P(alertMsg, PSTR("Kitchen Sink"));
      break;
    case 9:
      strcat_P(alertMsg, PSTR("Hot Tub Pump/Filter"));
      break;
    case 10:
      strcat_P(alertMsg, PSTR("Behind Hot tub"));
      break;
    case 11:
      strcat_P(alertMsg, PSTR("Water Tank"));
      break;
    case NUM_WIRED_SENSORS:  // Master Bath
      strcat_P(alertMsg, PSTR("Master Bath Sink"));
      break;
    case NUM_WIRED_SENSORS + 1:  // Guest Bath
      strcat_P(alertMsg, PSTR("2nd Fl Guest Bathroom Sink"));  // longest text - 26 char
      break;
  }  // end switch
  
  SendTweet(alertMsg);  // Send message to Twitter
  
} // SendAlert ()


//=========================================================================================================================
// Send twitter text, appends the time to the message to avoid twitter blocking duplicate messages
// Example usage:
//    strcpy(msgTweet, "Water leak by washing machine");
//    SendTweet(msgTweet);
//=========================================================================================================================
int SendTweet(char msgTweet[])
{
  char tweetAndTime[strlen(msgTweet) + 12];  // set char array so it can hold message and timestamp
  strcpy(tweetAndTime, msgTweet);            // copy twitter message into bigger character array
  
  // Get time from NTP Time server and append to twitter message.  This avoids duplicate tweets which may not get sent
  uint8_t getServerTime[6];
  char timebuf[11];  // char array to hold formatted time
  if(getTime(getServerTime))
  {
    if(getServerTime[4] == 1)
    { sprintf(timebuf, "  %d:%02d AM", getServerTime[1], getServerTime[2]); }
    else
    { sprintf(timebuf, "  %d:%02d PM", getServerTime[1], getServerTime[2]); }
    
    strcat(tweetAndTime, timebuf);  // add timestamp to twiiter message
  }
  
  delay(500);  // Thought it would be good to have a short delay after getting the time and before sending a Tweet
  
#ifdef PRINT_DEBUG
  Serial.print(F("Tweet: "));
  Serial.println(tweetAndTime);  // Print Tweet to serial monitor
  Serial.println();
#endif
  
  
  // Send message and timestamp to twitter
  if(twitter.post(tweetAndTime))
  {
    // Specify &Serial to output received response to Serial.
    // If no output is required, you can just omit the argument, e.g.  int status = twitter.wait();
    int status = twitter.wait(&Serial);
#ifdef PRINT_DEBUG
    if(status == 200)
    {
      Serial.println(F("\nTwitter OK."));
    }
    else  // status != 200
    {
      Serial.print(F("Twitter failed. Err code: "));
      Serial.println(status);
    }
#endif
    return status;
  }
  else
  {
#ifdef PRINT_DEBUG
    Serial.println(F("Twitter connection failed.\n"));
#endif
    return 0;
  }
 
} // SendTweet()


//=========================================================================================================================
// Print states of all the sensors - remove when debugging isn't needed
//=========================================================================================================================
void PrintStates()
{
  int totalInputs = NUM_WIRED_SENSORS + NUM_WIRELESS_SENSORS;
  
  Serial.print(F("\nInputState\t"));
  for(int i = 0; i < totalInputs; i++)
  {
    Serial.print(InputState[i]);
    Serial.print("\t");
  }
  Serial.println();
//  Serial.print(F("WetFlag    \t"));
//  for(int i = 0; i < totalInputs; i++)
//  {
//    Serial.print(isWet[i]);
//    Serial.print("\t");
//  }
//  Serial.println();

  Serial.print(F("WaterDetect\t"));
  for(int i = 0; i < totalInputs; i++)
  {
    Serial.print(WaterDetect[i]);
    Serial.print("\t");
  }
  Serial.println();
  
  if (masterBath.online)
  {
    Serial.println(F("\t\tIsWet\tTemp\tVolts"));
    Serial.print(F("Master Bath"));
    Serial.print("\t");
    Serial.print(masterBath.IsWet);
    Serial.print("\t");
    Serial.print(masterBath.temp);
    Serial.print("\t");
    Serial.println(masterBath.volts);
  }
  else
  {
    Serial.println(F("Master Bath is offline"));
  }
  
  if(guestBath.online)
  {
    Serial.print("Guest Bath");
    Serial.print("\t");
    Serial.print(guestBath.IsWet);
    Serial.print("\t");
    Serial.print(guestBath.temp);
    Serial.print("\t");
    Serial.println(guestBath.volts);
  }
  else
  {
    Serial.println(F("Guest Bath is offline"));
  }

} // End PrintStates()


//=========================================================================================================================
// Update the OLED display showing any wet inputs
// There is no "sponge debounce" like with the twitter alers
// this shows the real-time high-low status of the inputs
//=========================================================================================================================
void updateOledDisplay()
{ 
  const int dispDelay = 1500;
  uint8_t ntpTime[6];
  getTime(ntpTime);

  if( masterBath.online == false)
  {
    display.clearDisplay();    
    display.setCursor(0,10);
    display.println("MASTER BATH OFFLINE");
    display.display();
    delay(dispDelay);
  }
  if( guestBath.online == false)
  {
    display.clearDisplay();    
    display.setCursor(0,10);
    display.println("GUEST BATH OFFLINE");
    display.display();
    delay(dispDelay);
  }
  
  display.clearDisplay();   
  
  if (isAnythingWet == false )
  {
    char timebuf[12];
    sprintf(timebuf, "%d:%02d ", ntpTime[0], ntpTime[2]);
    switch ( ntpTime[5] )
    {
      case 0: strcat(timebuf, "Sun"); break; 
      case 1: strcat(timebuf, "Mon"); break; 
      case 2: strcat(timebuf, "Tue"); break; 
      case 3: strcat(timebuf, "Wed"); break; 
      case 4: strcat(timebuf, "Thu"); break; 
      case 5: strcat(timebuf, "Fri"); break; 
      case 6: strcat(timebuf, "Sat"); break; 
    }

    display.setCursor(0,10);
    display.println("EVERYTHING IS DRY");
    display.setCursor(0,20);
    display.println(timebuf);
    display.display();
  }
  else
  {
    // At least one sponge is Wet, cycle through them all and display the wet ones
    for(int i = 0; i < NUM_WIRED_SENSORS + NUM_WIRELESS_SENSORS; i++)
    {
      if ( InputState[i] == WET )         
      {
        display.clearDisplay();   
        display.setCursor(0,10);
        switch(i)
        {
          case 0: 
            display.println("1ST FL BATH SINK");
            break;
          case 1: 
            display.println("WASHING MACHINE");
            break;
          case 2: 
            display.println("TBD1");
            break;
          case 3: 
            display.println("TBD2");
            break;
          case 4: 
            display.println("WATER HEATER");
            break;
          case 5: 
            display.println("BOILER");
            break;
          case 6: 
            display.println("FRIG");
            break;
          case 7: 
            display.println("DISH WASHER");
            break;
          case 8: 
            display.println("KITCHEN SINK");
            break;
          case 9: 
            display.println("HOT TUB FILTER");
            break;
          case 10: 
            display.println("HOT TUB REAR");
            break;
          case 11: 
            display.println("WATER TANK");
            break;
          case NUM_WIRED_SENSORS:
            display.println("MASTER BATH");
            break;
          case NUM_WIRED_SENSORS + 1:
            display.println("GUEST BATH");
            break;   
        }  // switch()
        display.setCursor(0,20);
        display.println("IS WET");
        display.display();
        delay(dispDelay);
      } // something is wet
    } // end for loop
  } // end isAnythingWet
  
}  // updateOledDisplay()


//=========================================================================================================================
// Returns mS until noon on next Sunday
// Used for weekly heartbeat
// ntpTime Array:
// 0 - hour (12 hr format)
// 1 - hour (24 hr format)
// 2 - minute
// 3 - second
// 4 - 1 for AM, 2 for PM
// 5 - day of week.  0=Sunday
//=========================================================================================================================
uint32_t getMsUntilSundayNoon(uint8_t *ntpTime)
{
  uint32_t mStoMidnight; // mS from now to midnight
  mStoMidnight =  (24 - (ntpTime[0] + 1)) * 3600000UL;  // convert hours left in the day to mS
  mStoMidnight += (60 - (ntpTime[2] + 1)) * 60000UL;    // add minutes left in current hour to mS
  mStoMidnight += (60 - ntpTime[3]) * 1000UL;           // add seconds left in current minute left to mS
  
  // Starting from midnight today, calculate how many days until Sunday starts
  int daysToSunday = 7 - ntpTime[5]  - 1;
  
  // 43200000 = mS in 12 hours - this moves time from midnight to noon on Sunday
  return ((daysToSunday * 86400000UL) + mStoMidnight + 43200000UL);
} // getMsUntilSundayNoon()


//=========================================================================================================================
// Restarts program from beginning but does not reset the peripherals and registers
//=========================================================================================================================
void software_Reset()
{
  asm volatile ("  jmp 0");
} // End software_Reset()



//========================================================================================================================
// start Ethernet and UDP
//========================================================================================================================
bool setupNTPTime()
{
  unsigned int localPort = 8888;           // local port to listen for UDP packets
  
  Ethernet.begin(mac, ip);
  Udp.begin(localPort);
  
}  // setupNTPTime()


//========================================================================================================================
// Get the time from NPT server
// try up to 3 NPT servers
//
// ntpTime Array
// 0 - hour (12 hr format)
// 1 - hour (24 hr format)
// 2 - minute
// 3 - second
// 4 - 1 for AM, 2 for PM
// 5 - day of week.  0=Sunday
//========================================================================================================================
bool getTime(uint8_t *ntpTime)
{
  static uint32_t lastNtpTimeCheck =  0;  // Last time NPT was checked, don't do within 4 seconds

  // Don't check NPT time again too quickly
  if ((long) (millis() - lastNtpTimeCheck ) < 5000)
  { delay(6000); }
  
  lastNtpTimeCheck = millis();
  
  sendNTPpacket(timeServer1); // send an NTP packet to a time server
  delay(500);  // wait for response, if you remove this, you get the same time every time.
  if ( Udp.parsePacket() )
  {
    parseTimePacket(ntpTime);
    return true;  // Got UDP packet from first server
  }

  // First NTP server didn't work, try the second one
  sendNTPpacket(timeServer2); // send an NTP packet to a time server
  delay(500);  // wait for response, if you remove this, you get the same time every time.
  if ( Udp.parsePacket() )
  {
    parseTimePacket(ntpTime);
    return true;  // Got UDP packet from first server
  }

  // Second NTP server didn't work, try the third one
  sendNTPpacket(timeServer3); // send an NTP packet to a time server
  delay(500);  // wait for response, if you remove this, you get the same time every time.
  if ( Udp.parsePacket() )
  {
    parseTimePacket(ntpTime);
    return true;  // Got UDP packet from first server
  }

  // No time servers worked
  return false;
  
} // getTime()


//==========================================================================================================================
// Got time from NTP server
// This function puts in ntpTime[] array so it can be used
//==========================================================================================================================
void parseTimePacket(uint8_t *ntpTime)
{
  // positions in ntpTime array
  const byte h24 = 0;
  const byte h12 = 1;
  const byte m = 2;
  const byte s = 3;
  const byte AMPM = 4;
  const byte dow = 5; // Day of week, 0 = Sunday
  
  
  // We've received a packet, read the data from it
  Udp.read(packetBuffer,NTP_PACKET_SIZE);  // read the packet into the buffer
  
  // the timestamp starts at byte 40 of the received packet and is four bytes,
  // or two words, long. First, esxtract the two words:
  
  unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
  unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
  
  // combine the four bytes (two words) into a long integer
  // this is NTP time (seconds since Jan 1 1900):
  unsigned long secsSince1900 = highWord << 16 | lowWord;
  
  // now convert NTP time into everyday time:
  const unsigned long seventyYears = 2208988800UL;
  // subtract seventy years:
  unsigned long epoch = secsSince1900 - seventyYears;
  
  // Adjust for time zone
  const int estOffset = 5;
  epoch -= (estOffset * 3600);
  
  // get the hour, minute and second:
  ntpTime[h24] = (epoch  % 86400L) / 3600;  // hour (86400 equals secs per day)
  ntpTime[m] =   (epoch  %   3600) / 60;    // minute (3600 equals secs per minute)
  ntpTime[s] =   (epoch  %     60);         // second
  
  // Convert hr from 24 hr format to 12 hour
  // and set AM/PM
  if (ntpTime[h24] > 12)
  {
    ntpTime[h12] = ntpTime[h24] - 12;
    ntpTime[AMPM] = 2;   // Time is PM
  }
  else if (ntpTime[h24] == 12)
  { // Time is PM but don't subtract 12
    ntpTime[h12] = 12;
    ntpTime[AMPM] = 2;   // Time is PM
  }
  else if (ntpTime[h24] == 0)
  { // Set time for 12 midnight
    ntpTime[h12] = 12;
    ntpTime[AMPM] = 1;   // Time is AM
  }
  else
  { // It's morning
    ntpTime[h12] = ntpTime[h24];
    ntpTime[AMPM] = 1;   // Time is AM
  }
  
  // Calculate day of week
  // 0 = Sunday
  long days = epoch / 86400L;
  ntpTime[dow] = (days+4) % 7;
}  // parseTimePacket


//==========================================================================================================================
// send an NTP request to the time server at the given address
//==========================================================================================================================
unsigned long sendNTPpacket(IPAddress& address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
                           // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
} // sendNTPpacket()


//=========================================================================================================================
// Print amount of free RAM
//=========================================================================================================================
int freeRam(bool PrintRam)
{
  int freeSRAM;
  extern int __heap_start, *__brkval;
  int v;
 
  freeSRAM =  (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
  if(PrintRam)
  {
  Serial.print(F("RAM "));
  Serial.println(freeSRAM);
  }
  return freeSRAM;
} // freeRam()
