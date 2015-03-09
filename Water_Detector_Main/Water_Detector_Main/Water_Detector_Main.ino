/*  
See Readme.txt file
directory /Dropbox/Arduino/Water_Detector/ 
Board: Leonardo
Not much memory left 

To do:
Make a class library and have each instance trigger actions instead of looping through arrays - maybe
setupNTPTime() always returns true, see if you can return actual state of ethernet connection
See if you can save memory by using http://github.com/nomadnt/uHTTP
Consider replacing Twitter with Tropo.com SMS service
 
 
Change Log
05/28/14 v2.00 - Changed low temp alarm logic so it only sends low temp tweet if voltage > 2500 mA.  Get false alarms when
                 voltage is below this
06/27/14 v2.01 - Updated tx offline message to display hrs or sec depending on how long it was offline.  Fixed
                 updateOledDisplay so Tx offline messages would display
07/04/14 v2.02 - Added checksum to data packet, renamed some constants
07/05/14 v2.03 - Changed NTP update. It was updating every time you checked sensors and every 48 hours.  Changed to check once a minutes
07/30/14 v2.04 - Added ability to print version if a "v" is typed into serial monitor
12/18/14 v2.05 - Added return to setupNTPTime().  Made sendNTPpacket() void instead of unsigned long
01/09/15 v2.06 - Renamed constant for RED led output to PIN_ALARM
01/12/15 v2.07 - Fixed Sunday heartbeat alert, I hope.  Compiled size (v1.0.5) 27,372/28,672 bytes
02/10/15 v2.08 - Added dehumidifier.  Changed TBD2 to DEHUMIDIFIER.  Size (IDE 1.0.5) 27,390 / 28,672 bytes
02/14/15 v2.09 - Removed printStates().  Size (IDE 1.0.5) 27,276 / 28,672 bytes
02/27/15 v3.00 - Changed I2C so master requests a specific wireless sensor.  Added 3rd wireless sensor.  Added temperature to Sunday heartbeat alert.
                 Used F() with display.print() for OLED to free up some RAM.  Made an array of struct and got wireless data by looping through array.
*/


#define VERSION "v3.00"
// #define PRINT_DEBUG      // Comment this out to turn off verbose printing

#include <HardwareSerial.h>
#include <SPI.h>             // Allows you to communicate with SPI devices. See: http://arduino.cc/en/Reference/SPI
#include <Ethernet.h>        // http://arduino.cc/en/Reference/Ethernet
#include <Twitter.h>         // http://arduino.cc/playground/Code/TwitterLibrary
#include <I2C.h>             // http://dsscircuits.com/index.php/articles/66-arduino-i2c-master-library  https://github.com/DSSCircuits/I2C-Master-Library
#include <Tokens.h>          // Contains Twitter token
#include <Adafruit_GFX.h>    // For OLED display http://github.com/adafruit/Adafruit-GFX-Library
#include <SSD1306_I2C_DSS.h> // For OLED display http://github.com/Scott216/SSD1306_I2C_DSS
#include "Water_Detector_Main_Library.h"  // Include application, user and local libraries


// This gets rid of compiler warning: Only initialized variables can be placed into program memory area
#undef PROGMEM
#define PROGMEM __attribute__(( section(".progmem.data") ))

byte mac[] = { 0x90, 0xA2, 0xDA, 0xEF, 0x46, 0x81 };
byte ip[] =  { 192, 168, 46, 81 };

// Analog inputs 0 & 1 are configured as Digitol I/O.
//      Name              I/O    Description
#define PIN_ALARM           0   // Turns on red LED and relay
#define OLED_RESET          1
// Reserved                 2      I2C
// Reserved                 3      I2C
// Reserved                 4      SD Card
//                          5      Unused
#define HOTTUBFILTER        6   // Sponge in crawlspace by hot tub filter and pump
#define HOTTUBBACK          7   // Sponge in crawlspace behind hot tub
#define WATERTANK           8   // Sponge in crawlspace corner by water tank
#define KITCHENSINK         9   // Sponge under kitchen sink
// reserved                10   // Slave select
#define DEHUMIDIFIER       11   // Dehumidifier
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

const uint8_t ADDR_I2C_OLED  =     0x3C;  // OLED Display I2C address
const uint8_t ADDR_I2C_SLAVE =       21;  // I2C Slave address of panStamp Rx
const uint8_t ADDR_BATH_MASTER =      0;  // panStamp device address for 2nd floor master bathroom  // SRG: Change addresses to 0,1,2
const uint8_t ADDR_BATH_GUEST =       1;  // panStamp device address for 2nd floor guest bathroom
const uint8_t ADDR_BATH_FIRST =       2;  // panStamp device address for 1st floor bathroom
const uint8_t NUM_SENSORS_WIRELESS =  3;  // number of wireless water sensors
const uint8_t NUM_SENSORS_WIRED =    12;  // Number of wired water detector inputs
const uint8_t DATA_LENGTH_I2C =      10;  // Packets sent by each panStamp
bool       g_isAnythingWet =      false;  // Flags if anything is wet


// Use arrays to hold input status.  Total number of inputs are wired plus wireless sensors
bool  InputState[NUM_SENSORS_WIRED + NUM_SENSORS_WIRELESS];     // Input reading, HIGH when water is present LOW when it's not
bool WaterDetect[NUM_SENSORS_WIRED + NUM_SENSORS_WIRELESS];     // Sensor state after delay to make sure it's really wet or dry
uint32_t weeklyHeartbeatTimer = 0;                              // mS until Sunday at noon, at which time a tweet will go out to verify sketch is still running  static uint8_t TweetCounter;  // prevent tweets from getting to high

// Define typedef structurs for wireless sensors.  typedef struct definition is in LocalLibrary.h
RemoteSensorData_t g_wirelessSensor[NUM_SENSORS_WIRELESS];  // array for wireless sensors

// Token for Twitter Account
Twitter twitter(TWITTER_TOKEN);

// Put input pin numbers in array so you can use loops to read wired inputs
uint8_t InputPinNum[] = { FIRSTFLOORBRSINK, WASHINGMACH, TBD1, DEHUMIDIFIER, WATERHEATER, BOILER, FRIG, DISHWASHER, KITCHENSINK, HOTTUBFILTER, HOTTUBBACK, WATERTANK };


// Function Prototypes
bool     ReadRFSensors(RemoteSensorData_t* rfsensor);
void     SendAlert(byte SensorArrayPosition, bool IsWet);
void     ProcessSensors(void);
int16_t SendTweet(char msgTweet[]);
uint32_t getMsUntilSundayNoon(uint8_t *ntpTime);  // mS until Sunday noon - for weeklyheartbeat timer
void     software_Reset();
bool     setupNTPTime();
bool     getTime(uint8_t *ntpTime);
void     parseTimePacket(uint8_t *ntpTime);
void     sendNTPpacket(IPAddress& address);
void     updateOledDisplay();
bool     sundayHeartbeatMsg(); // sends twitter message Sunday at noon with info from wireless sensors

// NPT servers work as of 4/17/13
// List of servers: http://tf.nist.gov/tf-cgi/servers.cgi
// don't query more then 4 seconds
IPAddress timeServer1( 216, 171, 112,  36); 
IPAddress timeServer2( 206, 246, 122, 250);
IPAddress timeServer3( 64,   90, 182,  55);  

const uint8_t NTP_PACKET_SIZE= 48;       // NTP time stamp is in the first 48 bytes of the message
      uint8_t packetBuffer[NTP_PACKET_SIZE];      //buffer to hold incoming and outgoing packets
      uint8_t ntpTime[6];

// A UDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

// Instantiate OLED dispplay object
Adafruit_SSD1306 display(OLED_RESET);


//=========================================================================================================================
//=========================================================================================================================
void setup ()
{  
  Serial.begin(9600);
  #ifdef PRINT_DEBUG
    while (!Serial && millis() < 6000) {}  // for Leonardo wait here for a few seconds for serial monitor connection
    Serial.print(F("Leak Detecter "));
    Serial.println(VERSION);
  #endif

  // Start I2C communication
  I2c.begin();
  I2c.timeOut(30000);  // set I2C timeout to 30 seconds
  I2c.pullup(0);       // disable internal pullup resistors on I2C pins, don't want to pull-up to 5 volts because of panStamp

  // Initialize OLED displey
  display.begin(SSD1306_SWITCHCAPVCC, ADDR_I2C_OLED);  
  display.clearDisplay();  
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,10);
  display.print(F("Leak Detect "));
  display.println(VERSION);
  display.display();
  
  // Initialize Ethernet connection and UDP
  bool EthernetOK = setupNTPTime();
  delay(2000);
  if(EthernetOK)
  {
    // Get the NTP Time
    if( getTime(ntpTime) )
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
  
  
  for(int i = 0; i < NUM_SENSORS_WIRED; i++)
  { pinMode(InputPinNum[i], INPUT); }
  pinMode(PIN_ALARM, OUTPUT);  // Alarm output: red LED and reed relay
  digitalWrite(PIN_ALARM, LOW);
  
  // Initialiaze one shot triggers for messages and IDs. 
  for ( byte i=0; i < NUM_SENSORS_WIRELESS; i++)
  {
    g_wirelessSensor[i].lowVoltMsgFlag =           false;
    g_wirelessSensor[i].lowTempMsgFlag =           false;
    g_wirelessSensor[i].txOfflineTimestamp =           0;
    g_wirelessSensor[i].flag_SentMsg_TxIsOffline = false;
    g_wirelessSensor[i].ID = i;  // Sensor ID number is also used as array element number.
  }
  
  char tweetMsg[] = "Leak Detector Restarted";
  SendTweet(tweetMsg);  // Send startup tweet
  
  ProcessSensors();  // Check sensors to see if anything is wet

  display.clearDisplay();
  display.display();

} // end setup()


//=========================================================================================================================
//=========================================================================================================================
void loop ()
{
  static uint32_t CheckSensorsTimer = 0;                // Timer to check the sensors
  static uint32_t checkNtpTimer = millis() +  MINUTE;
  
  if ( (long)(millis() - CheckSensorsTimer) >= 0 )
  {
    display.clearDisplay();    
    display.setCursor(0,10);
    display.println(F("CHECKING SENSORS..."));
    display.display();
    
    ProcessSensors();  // Check sensors to see if anything is wet
    
    updateOledDisplay();
    CheckSensorsTimer = millis() + 1000UL; // add 1 second to timer
  }
  
  sundayHeartbeatMsg();  // sends tweet for each wireless sensor every Sunday at noon 

  // If anything is wet, turn on LED and relay (they are on the same output)
  bool wetOutputState = LOW;
  for(int i = 0; i < NUM_SENSORS_WIRED + NUM_SENSORS_WIRELESS; i++)
  {
    if( WaterDetect[i] == WET )
    { wetOutputState = HIGH; }
  }    
  digitalWrite(PIN_ALARM, wetOutputState);  // turn on red LED and relay if anything is wet

  // Check NTP timer
  if( (long)( millis() -  checkNtpTimer) > 0 )
  {
    if( getTime(ntpTime) )
    { checkNtpTimer = millis() + MINUTE; }
  }
  
  // To let user see what version is running, code below looks for a v typed into the
  // serial monitor.  If it sees V, it will display the version in the serial monitor
  if ( Serial.available() )
  {
    byte serialData = Serial.read();
    if( serialData == 86 || serialData == 118 )  // Check for upper and lower case V
    { Serial.print(VERSION); }
  }
  
}  // end loop()


//=========================================================================================================================
// Check inputs to see if any sponges are wet
//=========================================================================================================================
void ProcessSensors()
{
  char twitterMsgNoName[50];  // Twitter message without the sensor name
  char sensorName[16];        // Sensor name to be used in twitter message
  char twitterMsg[70];        // Twitter message with the sensor name
 
  
  static bool isWet[NUM_SENSORS_WIRED + NUM_SENSORS_WIRELESS];  // Used to trigger if an input goes from Dry to Wet.  Trigger will turn on WaterDetectOutput for 10 seconds.

  g_isAnythingWet = false;  // reset flag
  
  // read hard wired inputs
  for(int i = 0; i < NUM_SENSORS_WIRED; i++)
  {
    // Read hard wired sensors
    // Loop until two consecutive reading are the same.  This helps elimanite bad readings
    bool  firstreading = DRY;
    bool secondreading = DRY;
    do
    {
      firstreading = digitalRead(InputPinNum[i]);
      delay(10);
      secondreading = digitalRead(InputPinNum[i]);
    } while (firstreading != secondreading);
    InputState[i] = firstreading;
    
    if ( InputState[i] == WET )
    { g_isAnythingWet = true; }
  }
  
  // Loop through wireless sensors and get data
  // Send Tweet if anything is wrong
  for (byte ws = 0; ws < NUM_SENSORS_WIRELESS; ws++)
  {
    
    // Sensor name for start of Twitter message
    switch( ws)
    {
      case ADDR_BATH_MASTER:
        strcpy(sensorName, "Master Bath ");
        break;
      case ADDR_BATH_GUEST:
        strcpy(sensorName, "Guest Bath ");
        break;
      case ADDR_BATH_FIRST:
        strcpy(sensorName, "First Fl Bath ");
        break;
      default:
        strcpy(sensorName, "Unknown  ");
        break;
    }
    
    // Get wiresless sensor data
    bool wirelessSensorIsOkay = ReadRFSensors( &g_wirelessSensor[ws]);
    if( wirelessSensorIsOkay )
    {
      
      // If wireless sensor came back online, send a Tweet
      if ( g_wirelessSensor[ws].flag_SentMsg_TxIsOffline == true )  // if this is true, it means sensor is now online, but offline message has been sent.  So now send a back online message
      {
        if ( (millis() - g_wirelessSensor[ws].txOfflineTimestamp) < 10800000UL ) // < 3 hours
        sprintf_P(twitterMsgNoName, PSTR("is back online: %dmV. Offline for %lu sec"), g_wirelessSensor[ws].volts, (millis() - g_wirelessSensor[ws].txOfflineTimestamp) / 1000UL );
        else
        sprintf_P(twitterMsgNoName, PSTR("is back online: %dmV. Offline for %lu hrs"), g_wirelessSensor[ws].volts, (millis() - g_wirelessSensor[ws].txOfflineTimestamp) / 3600000UL );
        strcpy(twitterMsg, sensorName);
        strcat(twitterMsg, twitterMsgNoName);
        SendTweet(twitterMsg);
      }

      g_wirelessSensor[ws].flag_SentMsg_TxIsOffline = false; // reset flag for sending Tweeet about wireless sensor being offline
          
      InputState[NUM_SENSORS_WIRED + ws] = g_wirelessSensor[ws].IsWet;
      if ( g_wirelessSensor[ws].IsWet == WET )
      { g_isAnythingWet = true; }

      // Check for low volts
      if( g_wirelessSensor[ws].online == true && g_wirelessSensor[ws].volts < 2800 && g_wirelessSensor[ws].lowVoltMsgFlag == false)
      {
        sprintf_P(twitterMsgNoName, PSTR("Low Battery: %dmV"), g_wirelessSensor[ws].volts );
        strcpy(twitterMsg, sensorName);
        strcat(twitterMsg, twitterMsgNoName);
        SendTweet(twitterMsg);
        g_wirelessSensor[ws].lowVoltMsgFlag = true;
      }
      // Reset low volts flag if volts is over 3000 mV
      if(g_wirelessSensor[ws].volts > 3000)
      {
        g_wirelessSensor[ws].lowVoltMsgFlag = false;
      }
      
      // Check for low temperature
      if( g_wirelessSensor[ws].online == true && g_wirelessSensor[ws].temp <= 40 && g_wirelessSensor[ws].temp >= 20 && g_wirelessSensor[ws].lowTempMsgFlag == false && g_wirelessSensor[ws].volts > 2500 )
      {
        sprintf_P(twitterMsgNoName, PSTR("Low Temp: %d°F"), g_wirelessSensor[ws].temp);
        strcpy(twitterMsg, sensorName);
        strcat(twitterMsg, twitterMsgNoName);
        SendTweet(twitterMsg);
        g_wirelessSensor[ws].lowTempMsgFlag = true;
      }
      // Reset low temp flag if temp is 50 F
      if(g_wirelessSensor[ws].temp >= 50)
      {
        g_wirelessSensor[ws].lowTempMsgFlag = false;
      }
    }  // end if( wirelessSensorIsOkay )
    else  // didn't get wireless data
    {
      // If wireless sensor is offline, send tweet
      if( g_wirelessSensor[ws].online == false && g_wirelessSensor[ws].flag_SentMsg_TxIsOffline == false )
      {
        strcpy_P(twitterMsgNoName, PSTR("is offline"));
        strcpy(twitterMsg, sensorName);
        strcat(twitterMsg, twitterMsgNoName);
        if( SendTweet(twitterMsg) == 200 )
        { g_wirelessSensor[ws].flag_SentMsg_TxIsOffline = true; }  // set flag so only one Tweet is sent
        g_wirelessSensor[ws].txOfflineTimestamp = millis(); // Time wireless sensor went offline
      }
    }  // finished processing 1 wireless sensor
  
  } // finished processing all wireless sensors
  
  
  // See if any inputs have gone from Dry to Wet
  static uint32_t DoubleCheckTime[NUM_SENSORS_WIRED + NUM_SENSORS_WIRELESS];  // Used to wait a few minutes after a sensor is triggered to check the sensor a 2nd time to see if there is still water. Like button debouncing
  for(int i = 0; i < NUM_SENSORS_WIRED + NUM_SENSORS_WIRELESS; i++)
  {
    if((InputState[i] == WET) && (isWet[i] == DRY ))
    {
      isWet[i] = WET;
      DoubleCheckTime[i] = millis() + DOUBLE_CHECK_DELAY;  // Set time to check again to make sure it's really wet
    }
  } // End check for Dry to Wet
  
  
  // Double check input after a delay to see it it's still wet
  for(int i = 0; i < NUM_SENSORS_WIRED + NUM_SENSORS_WIRELESS; i++)
  {
    if( (isWet[i] == WET) && (WaterDetect[i] == DRY) && ((long)(millis() - DoubleCheckTime[i]) >= 0) )
    {
      WaterDetect[i] = WET;  // Sponge is still wet after delay
      SendAlert(i , WET);    // One of the inputs is still wet after DOUBLE_CHECK_DELAY.  Send Tweet out
     }
  } // End double check for wet sponge
  
  
  // If input is dry, reset WetFlag[]
  // A sponge isn't determined to be dry unless the input has been dry for DRYING_DELAY (3 hours)
  static uint32_t   WetToDryDelay[NUM_SENSORS_WIRED + NUM_SENSORS_WIRELESS];  // Delay used to let sponge dry out before indicating it's dry
  for(int i = 0; i < NUM_SENSORS_WIRED + NUM_SENSORS_WIRELESS; i++)
  {
    if(InputState[i] == DRY)
    { isWet[i] = DRY; }
    else
    { WetToDryDelay[i] = millis() + DRYING_DELAY; } // If input is WET, reset drying delay
  }
  
  // Check for dried out sponge, after long delay
  for(int i = 0; i < NUM_SENSORS_WIRED + NUM_SENSORS_WIRELESS; i++)
  {
    if( (isWet[i] == DRY) && (WaterDetect[i] == WET) && ((long)(millis() - WetToDryDelay[i]) >= 0))
    {
      // Sensor went from Wet To Dry
      WaterDetect[i] = DRY;
      SendAlert(i, DRY);
    }
  } // End Wet to Dry
  
} // end ProcessSensors()


//=========================================================================================================================
// Get the data from wireless sensors
// Returns true if sensor is online and master recieved data, otherwise false
//=========================================================================================================================
bool ReadRFSensors(RemoteSensorData_t* rfsensor)
{
  bool gotI2cPacket = false;  // Flag to indicate I2C packet came in
  const uint8_t CHECKSUMBYTE = DATA_LENGTH_I2C - 1;
  const uint8_t I2C_OKAY = 0;
  
  uint8_t i2cPacketData[DATA_LENGTH_I2C];   // Array to hold wireless sensor data
  
  uint8_t i2cReturnStatus = I2c.write(ADDR_I2C_SLAVE, rfsensor->ID);  // Tell slave which remote sensor data to send back
  delay(10); // Slave needs a little time before next command
  if(i2cReturnStatus == I2C_OKAY)
  {
    i2cReturnStatus = I2c.read(ADDR_I2C_SLAVE, DATA_LENGTH_I2C , i2cPacketData); // gets data from slave for selected remote sensor
    if(i2cReturnStatus == I2C_OKAY)
    {  gotI2cPacket = true; }
  }
  
  // If got I2C packet, then  process it
  if( gotI2cPacket )
  {
    // verify checksum
    byte checksum = 0;
    for (int cs = 0; cs < CHECKSUMBYTE; cs++)
    { checksum += i2cPacketData[cs]; }
    
    if ( i2cPacketData[CHECKSUMBYTE] !=  checksum )  // SRG if checksum is on the left, condition is always true.  Can't reproduce issue in simplified sketch
    {
      // Checksum failed
      #ifdef PRINT_DEBUG
        Serial.println(F("Checksum failed"));
      #endif
      rfsensor->online = false;
      return false;  // exit function
    }
    
    // See if slave returned data from the correct remote and if the remote is online
    bool addressMatches = ( i2cPacketData[0] == rfsensor->ID );
    bool remoteIsOnline = ( i2cPacketData[1] == true );
    if ( addressMatches && remoteIsOnline )
    {
      // Move sensor data from I2C packet to local structure
      rfsensor->ID =     i2cPacketData[0];
      rfsensor->online = i2cPacketData[1];
      rfsensor->IsWet =  i2cPacketData[2];
      rfsensor->temp  =  i2cPacketData[3] << 8;
      rfsensor->temp |=  i2cPacketData[4];
      rfsensor->volts  = i2cPacketData[5] << 8;
      rfsensor->volts |= i2cPacketData[6];
      return true;
    }
    else  // Wrong address ID or remote is offline
    {
      rfsensor->online = false;
      return false;
    }
  }
  else // didn't get I2C packet
  {
    rfsensor->online = false;
    return false;
  }
  
} // end ReadRFSensors()


//=========================================================================================================================
// Pick message to send to Twitter
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
      strcat_P(alertMsg, PSTR("Dehumidifier"));
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
    case NUM_SENSORS_WIRED:  // Master Bath
      strcat_P(alertMsg, PSTR("Master Bath Sink"));
      break;
    case NUM_SENSORS_WIRED + 1:  // Guest Bath
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
int16_t SendTweet(char msgTweet[])
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
// Update the OLED display showing any wet inputs
// There is no "sponge debounce" like with the twitter alers
// this shows the real-time high-low status of the inputs
//=========================================================================================================================
void updateOledDisplay()
{ 
  const int dispDelay = 1500;

  if( g_wirelessSensor[ADDR_BATH_MASTER].online == false)
  {
    display.clearDisplay();    
    display.setCursor(0,10);
    display.println(F("MASTER BATH OFFLINE"));
    display.display();
    delay(dispDelay);
  }

  if( g_wirelessSensor[ADDR_BATH_GUEST].online == false)
  {
    display.clearDisplay();    
    display.setCursor(0,10);
    display.println(F("GUEST BATH OFFLINE"));
    display.display();
    delay(dispDelay);
  }

  if( g_wirelessSensor[ADDR_BATH_FIRST].online == false)
  {
    display.clearDisplay();    
    display.setCursor(0,10);
    display.println(F("FIRST FL BATH OFFLINE"));
    display.display();
    delay(dispDelay);
  }
  
  display.clearDisplay();   
  
  if (g_isAnythingWet == false )
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
    display.println(F("EVERYTHING IS DRY"));
    display.setCursor(0,20);
    display.println(timebuf);
    display.display();
  }
  else
  {
    // At least one sponge is Wet, cycle through them all and display the wet ones
    for(int i = 0; i < NUM_SENSORS_WIRED + NUM_SENSORS_WIRELESS; i++)
    {
      if ( InputState[i] == WET )         
      {
        display.clearDisplay();   
        display.setCursor(0,10);
        switch(i)
        {
          case 0: 
            display.println(F("1ST FL BATH SINK"));
            break;
          case 1: 
            display.println(F("WASHING MACHINE"));
            break;
          case 2: 
            display.println(F("TBD1"));
            break;
          case 3: 
            display.println(F("DEHUMIDIFIER"));
            break;
          case 4: 
            display.println(F("WATER HEATER"));
            break;
          case 5: 
            display.println(F("BOILER"));
            break;
          case 6: 
            display.println(F("FRIG"));
            break;
          case 7: 
            display.println(F("DISH WASHER"));
            break;
          case 8: 
            display.println(F("KITCHEN SINK"));
            break;
          case 9: 
            display.println(F("HOT TUB FILTER"));
            break;
          case 10: 
            display.println(F("HOT TUB REAR"));
            break;
          case 11: 
            display.println(F("WATER TANK"));
            break;
          case NUM_SENSORS_WIRED:
            display.println(F("MASTER BATH"));
            break;
          case NUM_SENSORS_WIRED + 1:
            display.println(F("GUEST BATH"));
            break;   
        }  // switch()
        display.setCursor(0,20);
        display.println(F("IS WET"));
        display.display();
        delay(dispDelay);
      } // something is wet
    } // end for loop
  } // end g_isAnythingWet
  
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
  mStoMidnight += (60 -  ntpTime[3]) * 1000UL;          // add seconds left in current minute left to mS
  
  // Starting from midnight today, calculate how many days until Sunday starts
  int daysToSunday = 7 - ntpTime[5]  - 1;
  
  // 43200000 = mS in 12 hours - this moves time from midnight to noon on Sunday
  return ((daysToSunday * 86400000UL) + mStoMidnight + 43200000UL);
  
} // getMsUntilSundayNoon()


//=========================================================================================================================
// If Sunday noon, send tweet for each wireless sensor with battery voltage and temperature
// Returns true if function sends the heartbeat message, otherwise falce
//=========================================================================================================================
bool sundayHeartbeatMsg()
{
  // If Sunday at noon, send the message
  if( (long)(millis() - weeklyHeartbeatTimer) >= 0 )
  {
    char tweetMsg[45];
    sprintf_P(tweetMsg, PSTR("Leak Detector: Master Bath = %dmV, %d°F"), g_wirelessSensor[ADDR_BATH_MASTER].volts, g_wirelessSensor[ADDR_BATH_MASTER].temp);
    SendTweet(tweetMsg);
    sprintf_P(tweetMsg, PSTR("Leak Detector: Guest Bath = %dmV, %d°F"),  g_wirelessSensor[ADDR_BATH_GUEST].volts,  g_wirelessSensor[ADDR_BATH_GUEST].temp);
    SendTweet(tweetMsg);
    sprintf_P(tweetMsg, PSTR("Leak Detector: 1st Fl Bath = %dmV, %d°F"), g_wirelessSensor[ADDR_BATH_FIRST].volts,  g_wirelessSensor[ADDR_BATH_FIRST].temp);
    SendTweet(tweetMsg);
    
    // Add 1 week to heartbeat timer
    if( getTime(ntpTime) )
    { weeklyHeartbeatTimer = millis() + getMsUntilSundayNoon(ntpTime); }
    else
    { weeklyHeartbeatTimer =  millis() + (MINUTE * 60UL * 24UL * 7UL); }  // couldn't refresh NTP time, just add 1 week of mS
    return true;
  }
  else
  { return false; }
  
}  // end sundayHeartbeatMsg()



//=========================================================================================================================
// Restarts program from beginning but does not reset the peripherals and registers
//=========================================================================================================================
void software_Reset()
{
  asm volatile ("  jmp 0");
} // end software_Reset()



//========================================================================================================================
// Start Ethernet and UDP
//========================================================================================================================
bool setupNTPTime()
{
  unsigned int localPort = 8888;           // local port to listen for UDP packets
  
  Ethernet.begin(mac, ip);
  Udp.begin(localPort);
  
  return true;
  
}  // end setupNTPTime()


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
  static uint32_t lastNtpTimeCheck = 0;  // Last time NPT was checked

  // Don't check NPT time again too quickly
  if ( (long)(millis() - lastNtpTimeCheck ) < 5000L )
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
  
} // end getTime()


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
  unsigned long lowWord =  word(packetBuffer[42], packetBuffer[43]);
  
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
  
}  // end parseTimePacket()


//==========================================================================================================================
// send an NTP request to the time server at the given address
//==========================================================================================================================
void sendNTPpacket(IPAddress& address)
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

} // end sendNTPpacket()


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
  
} // end freeRam()
