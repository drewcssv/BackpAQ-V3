//// ----------------------------------- Version 3 for Blynk IOT -------------------
//// BackpAQ    V0.99G-IOT (for AdaFruit Feather Huzzah ESP32 and Sensirion SCD4X, Plantower PMAs003i , MEMS I2S Sound Sensor + onboard GPS +SDcard + standalone + conversions + Blynk 1.2                            /////
//// (c) 2021, 2022, 2023 BackpAQ Labs LLC and Sustainable Silicon Valley
//// BackpAQ Personal Air Quality Monitor
//// Written by A. L. Clark, BackpAQ Labs
/*
  MIT License

  Copyright (c) 2021, 2022, 2023 BackpAQ Labs LLC and Sustainable Silicon Valley

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE
*/

#define USE_SPIFFS  // Using SPIFFS

#include <FS.h>
#ifdef USE_LittleFS
//#define SPIFFS LITTLEFS
#include <LITTLEFS.h>
#else
#include <SPIFFS.h> // This version used LITTLEFS
#endif

// realtime clock 
#include "RTClib.h" 
RTC_PCF8523 rtc;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// include these libs for noise sampling via I2S
//#include <driver/i2s.h> // needs to be here before config!
//#include "arduinoFFT.h" // note special version!

#include "config_90.h" // config file  make config changes here

//-------------------------------------------------------------------------------------------------------------
// NOTE: This version is for BLYNK IOT with WiFi provisioning, OTA
//-------------------------------------------------------------------------------------------------------------

#define BLYNK_TEMPLATE_ID "TMPLFxu7Ig6o"
#define BLYNK_DEVICE_NAME "BackpAQV3"

#define BLYNK_FIRMWARE_VERSION        "0.5.6" // starting new versioning with V3; new PM averaging; adjust time intervals; cleanup; Blynk 1.1.0; bitmap; sync; conversions; calibration; temp offset

#define BLYNK_PRINT Serial
#define BLYNK_DEBUG

#define APP_DEBUG

// Board is Adafruit Feather Huzzah32  ESP32

#include "BlynkEdgent.h" // this is BLYNK IOT V2, not legacy!!

#ifdef ESP32
#include <esp_wifi.h>

#define ESP_getChipId()   ((uint32_t)ESP.getEfuseMac())

#define LED_ON      HIGH
#define LED_OFF     LOW
#else
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>

#define ESP_getChipId()   (ESP.getChipId())

#define LED_ON      LOW
#define LED_OFF     HIGH
#endif

#define GPS_BackpAQ 1           // Use GPS in BackpAQ device

#ifdef ESP32
//#include <BlynkSimpleEsp32.h>
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#else
#include <BlynkSimpleEsp8266.h>
#endif

// includes for Arduino and Adafruit libraries
#include <EEPROM.h>
#include <Adafruit_GFX.h>        // https://github.com/adafruit/Adafruit-GFX-Library
//#include <Adafruit_SSD1306.h>  //https://github.com/adafruit/Adafruit_SSD1306
#include <Adafruit_SH110X.h>     // https://github.com/adafruit/Adafruit_SH110x
#include <SensirionI2CScd4x.h>   // https://github.com/Sensirion/arduino-i2c-scd4x
#include <Arduino.h>
#include "ArduinoJson.h"          // Needs V5 (in script folder)
#include <widgetRTC.h>
#include <TimeLib.h>              //https://github.com/PaulStoffregen/Time
#include "ThingSpeak.h"           // Thingspeak Arduino library
#include "Adafruit_PM25AQI.h"     // https://github.com/adafruit/Adafruit_PM25AQI
#include "LinearRegression.h"     // Linear Regression Library
#include "SparkFun_Qwiic_OpenLog_Arduino_Library.h" // OpenLog Lib
// includes and variables for Sound
#include "soundsensor.h"
#include "measurement.h"
#include <Wire.h> //Needed for I2C 

#include "bitmap.h" // BackpAQ logo in array form

// libraries for GPS
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;
#include <MicroNMEA.h>            //http://librarymanager/All#MicroNMEA
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

// Open Log
OpenLog myLog; //Create instance

/* OLED Variables */
#include "oled.h"
static Oled oled;

int wifiFlag = 0;
#define wifiLed   2   //D2

/* Sound Variables */
static char deveui[32];
static int cycleTime = CYCLETIME;
static bool ttnOk = false;
String currLoudnessDBA; // holds currrent dB(A) average noise value
String currLoudnessDBC; // holds currrent dB(C) average noise value
String currLoudnessDBZ; // holds currrent dB(Z) average noise value
String currSpectrumA;
String currSpectrumC;
String currSpectrumZ;

/* task semaphores */
bool audioRequest = false;
bool audioReady = false;

/* payloadbuffer */
unsigned char payload[80];
int payloadLength = 0;

/* Global variables */
//const int PINPMSGO = D0; // PMS "sleep" pin to save power (on WeMos)
#define LENG 31  //0x42 + 31 bytes equal to 32 bytes  
#define LENG 31   //0x42 + 31 bytes equal to 32 bytes
#define MSG_LENGTH 31   //0x42 + 31 bytes equal 
#define DEBUG_PRINTLN(x)  Serial.println(x)
#define DEBUG_PRINT(x)  Serial.print(x)

/* SSID and PW for Router */
String Router_SSID;
String Router_Pass;

/* Buffers */
unsigned char buf[LENG];
char floatString[15]; // for temp/humidity string conversion
char buffer[20];
char pmLabel[20];
String pmLabel1;
bool thingspeakWebhook = true;
bool backpaq_mobile = false; // default mode is station
char default_GPS[20];
char default_gps_position[25];
char resultstr[50]; // for PM Sensor
int status;

/* Temp, humidity variables */
float p; // pressure (not used here)
float senTemp; // temperature F
float senHumid; // humidity
char tempF[20];
char humid[20];
//float temperature;
// float humidity;

/* CO2, TVOC variables */
uint16_t co2;
uint16_t tempCO2;
int SCD4Xcount;
float eCO2;
float TVOC;
uint16_t co22; // co2 is a global var
float temperature;
float humidity;
float tempTemp, tempHumid;
uint16_t TVOC_base;
uint16_t eCO2_base;
bool noSGP = false;
float tempOffset;

/* Linear Regression variables */
int tv_counter = 0;
float values[2];        // define variables
int counter = 0;        // counting values for regression
int forecast = 0;       // in time[min] will be reach the next threshold
double correlation;

/* GPS variables */
float latI = 0;
float lonI = 0;
double latID = 0;
double lonID = 0;
float gpsSpeed = 0;
long speed = 0;
long heading = 0;

float lastKnownLat = 0;
float lastKnownLon = 0;
float default_lat = 0;
float default_lon = 0;
int markerNum = 0; // index for GPS position, incremented for each location

/* Track variables */
String sensorName = "";
String trackName = "no name";
String trackStartTime = "";
String trackEndTime = "";
bool amTracking = false;
String track_comment = "";
String last_track_comment = "";
String track_comment_array[500];
int track_id = 0;
float latISav [500]; // array to save tracks
float lonISav [500];
char buffr[16];
String userID = "xxxxxxxxxx";

/* session variables */
int session = 0;
String description = "";

/* AQI Variables */
int airQualityIndex = 0;
String newColor;
String iconURL = "";

/* Battery voltage, power variables */
#define batteryPin 35
//#define batteryPin A13
#define battery_K ((2) * (3300.0) / 4096.0)
String batteryColor;
String newLabel;
int gaugeValue;
float batteryV = 0.0;

/* Misc global variables */
float rssi = 0;
String currentTime;
String currentDate;
String timeHack;
//unsigned long previousMillis;

// modes
int privateMode = 0;
bool standalone = false; // testing 1 2 3
bool noBlynk;

// Logging
bool label_A = true;
bool label_B = true;
bool label_C = true;
bool needHeader = false;

#define STATUS_SD_INIT_GOOD 0
#define STATUS_FILE_OPEN 3
#define STATUS_IN_ROOT_DIRECTORY 4

String BackpAQ_Logfile_A = "";
String BackpAQ_Logfile_B = "";
String BackpAQ_Logfile_C = "";

int sensorID = 0;

/* display variables */
bool not_Connected = false;
bool display_PM25 = false;
bool display_PM10 = false;
bool display_AQI = false;
bool display_TD = false;
String conversion = "NONE";
int convFactor = 1;

/*flag for saving data */
bool shouldSaveConfig = false;

/* Onboard LED I/O pin on NodeMCU board */
const int PIN_LED = 2; // D4 on NodeMCU and WeMos. Controls the onboard LED.

/* Indicates whether ESP has WiFi credentials saved from previous session, or double reset detected*/
bool initialConfig = false;

#define ESP_DRD_USE_EEPROM      false
#define ESP_DRD_USE_SPIFFS      true    //false
#define DOUBLERESETDETECTOR_DEBUG       true  //false

// Number of seconds after reset during which a
// subseqent reset will be considered a double reset.
#define DRD_TIMEOUT 7

// RTC Memory Address for the DoubleResetDetector to use
#define DRD_ADDRESS 0

/* set up CO2 forecast "traffic light" parameters */
String trafficLight = "green";
int greenLevel = 0;     // threshold - enterng green level
int yellowLevel = 500;  // threshold - entering yellow level
int redLevel = 1000;    // threshold - entering red level

#define ADC_PIN 13
#define CONV_FACTOR 4.0

// --------------------------------------------------------------------------------------------------------

LinearRegression lr;    // Linear Regression: define lr object

/* create soundsensor */
static SoundSensor soundSensor;

// Start I2C  with both ESP3Kit ports (for Heltec only)
//TwoWire OLEDWire =  TwoWire(1);
//TwoWire SensorWire =  TwoWire(1);

// Declaration for the OLED display connected to I2C (SDA, SCL pins) and uses pin 16 on ESP32 for reset
#define OLED_RESET     16 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SH1107 OLED = Adafruit_SH1107(64, 128, &Wire); // OLED display is feather "wing"
//---------------------------------------------------------------------------------------------------------
/* Initialize sensors */
Adafruit_PM25AQI aqi = Adafruit_PM25AQI(); // PM sensor library

SensirionI2CScd4x scd4x;  // Sensirion SCD4X Sensor 

/* Blynk widgets */
//WidgetMap myMap(V7); // set up Map widget on Blynk
WidgetLED led2(V91); // indicate tracking started
WidgetTerminal myTerm(V70); //Terminal widget for entering comments on Map
BlynkTimer timer; // Blynk timer
WidgetRTC rtcB; // initialize realtime clock

WiFiClient  client1; // initialize WiFi client

////--------------------------------------------------------------------------------------------------------------------
// Blynk Mobile App Events and Handlers ----
// The following are functions that are invoked upon request from the BackpAQ device
////--------------------------------------------------------------------------------------------------------------------
// This function is called whenever the assigned Button changes state
BLYNK_WRITE(V29) {  // V29 is the Virtual Pin assigned to the Privacy Button Widget
  if (param.asInt()) {  // User desires private data mode, no update to Thingspeak
    Serial.print("Privacy Mode on, ");
    privateMode = true;
    noBlynk = true; // set if not using Blynk/smartphone app
  } else {
    Serial.print("Privacy Mode off, ");
    privateMode = false;
  }
}
// This function is called whenever the assigned Button changes state
BLYNK_WRITE(V22) {  // V22 is the Virtual Pin assigned to the Standalone Button Widget
  if (param.asInt()) {  // User desires standalone mode, no update to Thingspeak, log data to SDcard
    Serial.print("!!!!!!!!!!!!!!!!! Standalone mode on, ");
    standalone = true;
  } else {
    Serial.print("????????????????? Standalone mode off, ");
    standalone = false;
  }
}
// This function is called whenever the assigned Button changes state [[ not currently being used ]]
BLYNK_WRITE(V45) {  // V45 is the Virtual Pin assigned to the Thingspeak Send Mode Button Widget
  if (param.asInt()) {  // thingspeakWebhook is *true* for sending to Thingspeak via Smartphone, *false* for sending via this script
    Serial.println(F("ThingSpeak WebHook is on, "));
    thingspeakWebhook = true;
  } else {
    Serial.println(F("Thingspeak WebHook =  off, "));
    thingspeakWebhook = false;
  }
}
// This function is called whenever "erase markers" button on Map is pressed
BLYNK_WRITE(V40) {
  if (param.asInt()) {
    // myMap.clear(); // erase any marker(s)
    Blynk.virtualWrite(V5, "clr"); // V5 is th;
  }
}
// This function is called whenever Session (tracking) Start/End is started/stopped
BLYNK_WRITE(V90) {
  if (param.asInt()) {
   
    blinkLedWidget();  // blink LED
    session ++ ;
     Serial.print("Session "); 
     Serial.print (session);
     Serial.println(" started at " + timeHack);
   // trackName = trackName + ">"; // append start
   // userID = "START";
    if (amTracking) { // if already tracking...
      amTracking = false;
    }
  } else {
    Serial.print("Session "); 
    Serial.print( session) ;
    Serial.println(" stopped at " + timeHack);
    led2.off();  // set LED to OFF
   // trackName = "<" + trackName; // append stop
   // userID = "STOP";
    if (!amTracking) { // if not tracking...
      amTracking = true;
    }
  }
}
// This function is called whenever Track Name is entered
BLYNK_WRITE(V84) {
  trackName = param.asStr();
  Serial.println("Track name is " + trackName);
}

// This function is called whenever "PM Conversion" menu is accessed
BLYNK_WRITE(V60) {
  switch (param.asInt())
  {
    case 1: // Item 1
      Serial.println("None selected");
      convFactor = 1;
      conversion = "No Conv";
      break;
    case 2: // Item 2
      Serial.println("US EPA selected");
      convFactor = 2;
      conversion = "USEPA";
      break;
    case 3: // Item 3
      Serial.println("LRAPA selected");
      convFactor = 3;
      conversion = "LRAPA";
      break;
    case 4: // Item 4
      Serial.println("AQandU selected");
      convFactor = 4;
      conversion = "AQandU";
      break;
    default:
      Serial.println("Unknown item selected");
  }
}

// Fires when BackpAQ device is connected
BLYNK_CONNECTED() {
  rtcB.begin(); // sync time on Blynk connect
  setSyncInterval(1 * 60);
  //Blynk.syncAll(); // sync after re-start
  Blynk.syncVirtual(V12, V13, V21, V90, V91, V22); // selective sync: GPS position, privacy, tracking status after (re)start

  /* Retrieve sensor metadata (Note: metadata values are unique to each BackpAQ sensor) Note: metadata values entered in Blynk Console */
  /* Note these are mostly about provisioning the ThingSpeak database */
  Blynk.sendInternal("meta", "get", "Sensor Name");
  Blynk.sendInternal("meta", "get", "Sensor Type");
  Blynk.sendInternal("meta", "get", "Channel A Number");
  Blynk.sendInternal("meta", "get", "Channel A Key");
  Blynk.sendInternal("meta", "get", "Channel B Number");
  Blynk.sendInternal("meta", "get", "Channel B Key");
  Blynk.sendInternal("meta", "get", "Channel C Number");
  Blynk.sendInternal("meta", "get", "Channel C Key");
  Blynk.sendInternal("meta", "get", "Default GPS Position");
  Blynk.sendInternal("meta", "get", "Backpaq Mode");
  Blynk.sendInternal("meta", "get", "Calibration");
  Blynk.sendInternal("meta", "get", "Temp Offset");
}

// Process sensor metadata
// Fires when Blynk is initiated

BLYNK_WRITE(InternalPinMETA) {
  String field = param[0].asStr();

  if (field == "Sensor Name")
  {
    String value = param[1].asStr();
    Serial.print("Sensor Name = ");
    Serial.println(value);
    sensorName = value; 
  }
  if (field == "Sensor Type")
  {
    String value = param[1].asStr();
    Serial.print("Sensor Type = ");
    Serial.println(value);
  }
  if (field == "Channel A Number")
  {
    thingspeak_A_channel_i = param[1].asInt();
    Serial.print("Channel A Number = ");
    Serial.println(thingspeak_A_channel_i);
  }
  if (field == "Channel B Number")
  {
    thingspeak_B_channel_i = param[1].asInt();
    Serial.print("Channel B Number = ");
    Serial.println(thingspeak_B_channel_i);
  }
  if (field == "Channel C Number")
  {
    thingspeak_C_channel_i = param[1].asInt();
    Serial.print("Channel C Number = ");
    Serial.println(thingspeak_C_channel_i);
  }
  if (field == "Channel A Key")
  {
    thingspeak_A_key_s = param[1].asStr();
    Serial.print("Channel A Key = ");
    Serial.println(thingspeak_A_key_s);
  }
  if (field == "Channel B Key")
  {
    thingspeak_B_key_s = param[1].asStr();
    Serial.print("Channel B Key = ");
    Serial.println(thingspeak_B_key_s);
  }
  if (field == "Channel C Key")
  {
    thingspeak_C_key_s = param[1].asStr();
    Serial.print("Channel C Key = ");
    Serial.println(thingspeak_C_key_s);
  }
  if (field == "Default GPS Position")
  {
    String default_gps_pos = param[1].asStr();
    Serial.print("Default GPS Position = ");
    Serial.println(default_gps_pos);
  }
  if (field == "Backpaq Mode")
  {
    String value = param[1].asStr();
    Serial.print("BackpAQ Mode = ");
    Serial.println(value);
    backpaq_mobile = 1; // for now
  }
   if (field == "Calibration")
  {
    int value = param[1].asInt();
    Serial.print("Calibration = ");
    Serial.println(value);
    calibration = value; 
  }
   if (field == "Temp Offset")
  {
    String value = param[1].asStr();
    Serial.print("Temperature Offset = ");
    Serial.println(value);
    tempOffset = value.toFloat(); 
  }
}

#ifdef GPS_SmartPhone
//This function is called whenever GPS position on smartphone is updated
BLYNK_WRITE(V15) {
  GpsParam gps(param); // fetch GPS position from smartphone

  latI = gps.getLat();
  lonI = gps.getLon();
  gpsSpeed = gps.getSpeed();

  if (latI == 0) { // in case Blynk not active or GPS not in range...
    latI = lastKnownLat; // assign last known pos
    lonI = lastKnownLon;
  } else {
    lastKnownLat = latI; // store pos as last known pos
    lastKnownLon = lonI;
  }
}
#endif
// Via Blynk Terminal Widget, Capture and Add comments to track data file on Thingspeak
BLYNK_WRITE(V70) {
  track_comment = "";
  track_id++;
  myTerm.clear(); // clear the pipes
  track_comment = param.asStr(); // fetch new comments from MAP page via Terminal
  if (track_comment != "" ) {
    String comment_str = timeHack + "," + String(latI) + "," + String(lonI) + "," + track_comment; // assemble comment string into CSV
    myTerm.println(comment_str); // echo back to terminal
    if (track_comment == last_track_comment) {
      track_comment_array[track_id] = ""; // if not new just leave blank
    }
    else {
      track_comment_array[track_id] = track_comment; //  add new comment
    }   
    myTerm.flush();
    // myTerm.clear();
  }
  last_track_comment = track_comment; // remember last comment
}
// function to blink LED on smartphone
void blinkLedWidget()
{
  if (led2.getValue()) {
    led2.off();

  } else if (amTracking) {
    led2.on();

  } else {
    led2.off();
  }
}
void printUint16Hex(uint16_t value) {
  Serial.print(value < 4096 ? "0" : "");
  Serial.print(value < 256 ? "0" : "");
  Serial.print(value < 16 ? "0" : "");
  Serial.print(value, HEX);
}

void printSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2) {
  Serial.print("Serial: 0x");
  printUint16Hex(serial0);
  printUint16Hex(serial1);
  printUint16Hex(serial2);
  Serial.println();
}

////////////////////////////////////////////////////////////
//  Structure for raw sensor data from PMS-7003 / PMSA003i
////////////////////////////////// /////////////////////////
struct dustvalues {
  uint16_t PM01Val_cf1 = 0; // Byte 4&5 (CF1 Bytes 4-9)
  uint16_t PM2_5Val_cf1 = 0; // Byte 6&7
  uint16_t PM10Val_cf1 = 0; // Byte 8&9
  uint16_t PM01Val_atm = 0; // Byte 10&11  (ATM bytes 10-15)
  uint16_t PM2_5Val_atm = 0; // Byte 12&13
  uint16_t PM10Val_atm = 0; // Byte 14&15
  uint16_t Beyond03 = 0; // Byte 16&17  # Particles
  uint16_t Beyond05 = 0; // Byte 18&19
  uint16_t Beyond1 = 0; // Byte 20&21
  uint16_t Beyond2_5 = 0; // Byte 22&23
  uint16_t Beyond5 = 0; //Byte 24&25
  uint16_t Beyond10 = 0; //Byte 26&27
};
struct dustvalues dustvalues1;

// send commands to PMS - not currently used
unsigned char gosleep[] = { 0x42, 0x4d, 0xe4, 0x00, 0x00, 0x01, 0x73 };
unsigned char gowakeup[] = { 0x42, 0x4d, 0xe4, 0x00, 0x01, 0x01, 0x74 };

void heartBeatPrint(void)
{
  static int num = 1;

  if (WiFi.status() == WL_CONNECTED)
    Serial.print("H");        // H means connected to WiFi
  else
    Serial.print("F");        // F means not connected to WiFi

  if (num == 80)
  {
    Serial.println();
    num = 1;
  }
  else if (num++ % 10 == 0)
  {
    Serial.print(" ");
  }
}
// Heatbeat function
void check_status()
{
  static ulong checkstatus_timeout = 0;

#define HEARTBEAT_INTERVAL    10000L
  // Print hearbeat every HEARTBEAT_INTERVAL (10) seconds.
  if ((millis() > checkstatus_timeout) || (checkstatus_timeout == 0))
  {
    heartBeatPrint();
    checkstatus_timeout = millis() + HEARTBEAT_INTERVAL;
  }
}
//--------------------------------------------------------------------------------------
//Handle sound measurements. Called from Blynk Timer.
void getSoundData( ) {

  // oled.begin(deveui);

  // Weighting lists
  static float aweighting[] = A_WEIGHTING;
  static float cweighting[] = C_WEIGHTING;
  static float zweighting[] = Z_WEIGHTING;

  // measurement buffers
  static Measurement aMeasurement( aweighting);
  static Measurement cMeasurement( cweighting);
  static Measurement zMeasurement( zweighting);

  soundSensor.offset( MIC_OFFSET);   // set microphone dependent correction in dB
  soundSensor.begin();

  // main loop task 0
  /* while( true){ */
  // read chunk from MEMS mic and perform FFT, and sum energy in octave bins
  float* energy = soundSensor.readSamples();

  // update
  aMeasurement.update( energy);
  cMeasurement.update( energy);
  zMeasurement.update( energy);

  // calculate averages and compose ThingSpeak message
  if ( audioRequest) {
    audioRequest = false; // if first time else...

    aMeasurement.calculate();
    cMeasurement.calculate();
    zMeasurement.calculate();

    // debug info, normally commented out
    aMeasurement.print();
    cMeasurement.print();
    zMeasurement.print();

    // oled.showValues( aMeasurement, cMeasurement, zMeasurement, ttnOk);

    composeMessage( aMeasurement, cMeasurement, zMeasurement); // compose and send data to ThingSpeak

    //Serial.write(payload, payloadLength);

    // reset counters etc.
    aMeasurement.reset();
    cMeasurement.reset();
    zMeasurement.reset();

    //  printf("end compose message core=%d\n", xPortGetCoreID());
    audioReady = true;    // signal that audio report is ready
  }
  audioRequest = true; // ensure one pass per time segment
}
// compose ThingSpeak payload message
static bool composeMessage( Measurement& la, Measurement& lc, Measurement& lz) {
  /*
    // find max value to compress values [0 .. max] in an unsigned byte from [0 .. 255]
    float max = ( la.max > lc.max) ? la.max : lc.max;
    max = ( lz.max > max) ? lz.max : max;
    float c = 255.0 / max;
    int i = 0;
    payload[ i++] = round(max);   // save this constant as first byte in the message

    payload[ i++] = round(c * la.min);
    payload[ i++] = round(c * la.max);
    payload[ i++] = round(c * la.avg);

    payload[ i++] = round(c * lc.min);
    payload[ i++] = round(c * lc.max);
    payload[ i++] = round(c * lc.avg);

    payload[ i++] = round(c * lz.min);
    payload[ i++] = round(c * lz.max);
    payload[ i++] = round(c * lz.avg);

    for ( int j = 0; j < OCTAVES; j++) {
      payload[ i++] = round(c * lz.spectrum[j]);
    }

    payloadLength = i;
    printf( "messagelength=%d\n", payloadLength);

    if ( payloadLength > 51)   // max TTN message length
      printf( "message too big, length=%d\n", payloadLength);
  */
  // Build spectrum array

  String energyStringA = ""; String energyA = "";
  String energyStringC = ""; String energyC = "";
  String energyStringZ = ""; String energyZ = "";
  String energyStringStart = "["; String energyStringEnd = "]";

  // Loop through spectrum for each weighting
  for (int i = 0; i < OCTAVES; i++) {
    energyA += String(la.spectrum[i]) + ",";
    energyC += String(lc.spectrum[i]) + ",";
    energyZ += String(lz.spectrum[i]) + ",";
  }
  // package for AQView in JSON
  energyStringA = energyStringStart + energyA + energyStringEnd;
  energyStringC = energyStringStart + energyC + energyStringEnd;
  energyStringZ = energyStringStart + energyZ + energyStringEnd;

  // DEBUG ONLY
  Serial.println("SpectrumA: " + energyStringA);
  Serial.println("SpectrumC: " + energyStringC);
  Serial.println("SpectrumZ: " + energyStringZ);

  static char outstrA[10]; static char outstrC[10]; static char outstrZ[10];
  // strings
  dtostrf(la.avg, 6, 2, outstrA);
  dtostrf(lc.avg, 6, 2, outstrC);
  dtostrf(lz.avg, 6, 2, outstrZ);

  // package to send to ThingSpeak
  currLoudnessDBA = outstrA; // update OLED, Blynk value is avg dB(A)
  currLoudnessDBC = outstrC; // update OLED, Blynk value is avg dB(C)
  currLoudnessDBZ = outstrZ; // update OLED, Blynk value is avg dB(Z)
  currSpectrumA = energyStringA;
  currSpectrumC = energyStringC;
  currSpectrumZ = energyStringZ;

  // DEBUG ONLY
  Serial.print("dB(A) average: "); Serial.println(outstrA);
  Serial.print("dB(C) average: "); Serial.println(outstrC);
  Serial.print("dB(Z) average: "); Serial.println(outstrZ);

  // Finally, update Blynk with sound values

  Blynk.virtualWrite(V25, la.avg);  // Update Blynk with current average Noise dB(A)
  Blynk.virtualWrite(V26, lc.avg);  // Update Blynk with current average Noise dB(C)
  Blynk.virtualWrite(V27, lz.avg);  // Update Blynk with current average Noise dB(Z)

  Blynk.virtualWrite(V28, energyA); // write "A" spectrum to Blynk (array of sound levels)
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//******************* Following are routines to calculate AQI from PM2.5 concentrations *******************************************

// US AQI formula: https://en.wikipedia.org/wiki/Air_Quality_Index#United_States
int toAQI(int I_high, int I_low, int C_high, int C_low, int C) {
  return (I_high - I_low) * (C - C_low) / (C_high - C_low) + I_low;
}
//// Note: It's important to keep the following EPA AQI breakpoints up to date. See the following references
// Source: https://gist.github.com/nfjinjing/8d63012c18feea3ed04e
// Based on https://en.wikipedia.org/wiki/Air_Quality_Index#United_States
// Updated 12/19 per  https://aqs.epa.gov/aqsweb/documents/codetables/aqi_breakpoints.html

int calculate_US_AQI25(float density) {

  int d10 = (int)(density * 10); // to counter documented sensitivity issue
  if (d10 <= 0) {
    return 0;
  } else if (d10 <= 120) { // Good
    return toAQI(50, 0, 120, 0, d10);
  } else if (d10 <= 354) { // Moderate
    return toAQI(100, 51, 354, 121, d10);
  } else if (d10 <= 554) { // Unhealthy for Sensitive Groups
    return toAQI(150, 101, 554, 355, d10);
  } else if (d10 <= 1504) { // Unhealthy
    return toAQI(200, 151, 1504, 555, d10);
  } else if (d10 <= 2504) { // Very Unhealthy
    return toAQI(300, 201, 2504, 1505, d10);
  } else if (d10 <= 3504) { // Hazardous
    return toAQI(400, 301, 3504, 2505, d10);
  } else if (d10 <= 5004) { // Hazardous
    return toAQI(500, 401, 5004, 3505, d10);
  } else if (d10 <= 10000) { // Hazardous
    return toAQI(1000, 501, 10000, 5005, d10);
  } else { // Are you still alive ?
    return 1001;
  }
}
// Set value, color of AQI gauge on Blynk gauge ////////
void calcAQIValue() {
  gaugeValue = airQualityIndex;

  // assign color according to US AQI standard (modified per https://airnow.gov/index.cfm?action=aqibasics.aqi)

  if (gaugeValue > 300) {
    newColor = AQI_MAROON;
    newLabel = "AQI: HAZARDOUS";
    iconURL = PURPLEICONURL;
  } else if (gaugeValue > 200) {
    newColor = AQI_PURPLE;
    newLabel = "AQI: VERY UNHEALTHY";
    iconURL = PURPLEICONURL;
  } else if (gaugeValue > 150) {
    newColor = AQI_RED;
    newLabel = "AQI: UNHEALTHY";
    iconURL = REDICONURL;
  } else if (gaugeValue > 100) {
    newColor = AQI_ORANGE;
    newLabel = "AQI: UNHEALTHY FOR SOME";
    iconURL = ORANGEICONURL;
  } else if (gaugeValue > 50) {
    newColor = AQI_YELLOW;
    newLabel = "AQI: MODERATE";
    iconURL = YELLOWICONURL;
  } else {
    newColor = AQI_GREEN;  //"Safe"
    newLabel = "AQI: GOOD";
    iconURL = GREENICONURL;
  }
}
// Function to convert CF1 to ATM
// Note that the PMS data from the Sensor comes in two flavors: (Standard Particles or CF-1, bytes 4-9)
// and  (Atmospheric Environment, bytes 10-15)
int cf1_to_sat(int cf1)
{
  int sat = -1;

  if (cf1 < 30) sat = cf1;
  if (cf1 > 100) sat = cf1 * 2 / 3;
  if (cf1 >= 30 && cf1 <= 100) sat = 30 + cf1 * (cf1 - 30) / 70 * 2 / 3;
  return sat;
}
////////////////////////////////////  Not currently used  /////////////////////////////////////////////
// Function to calculate NowCast from PM hourly average. Needed as proper input to AQI.
// See https://airnow.zendesk.com/hc/en-us/articles/212303417-How-is-the-NowCast-algorithm-used-to-report-current-air-quality-
// See also: https://www3.epa.gov/airnow/aqicalctest/nowcast.htm
// Method: Enter up to twelve hours of PM2.5 concentrations in ug/m3; the NowCast is calculated below.
// Usage: c = NowPM25(hourly);

int NowPM25(int *hourly)
{
  int hour12Avg = 0;
  int hour4Avg = 0;
  int ratioRecentHour = 0;
  float adjustedhourly[12];
  int i;
  int items = 12;
  int items3 = 0;
  int missing4 = 0;
  int missing12 = 0;
  float adjusted4hour[4];
  float averagearray[12];
  int items4 = 0;
  float max = 0.0;
  float min = 99999.9;
  int range = 0;
  int rateofchange = 0;
  int weightfactor = 0;
  int sumofweightingfactor = 0;
  int sumofdatatimesweightingfactor = 0;
  int nowCast = -999;

  // Find the number of missing values
  //
  missing12 = 0;
  missing4 = 0;
  items3 = 3;
  for (i = 0; i < items3; i++)
  {
    if (hourly[i] = -1)
    {
      missing4++;
    }
  }
  if (missing4 < 2)
  {
    for (i = 0; i < items; i++)
    {
      if (hourly[i] != -1)
      {
        if (hourly[i] < min)
        {
          min = hourly[i];
        }
        if (hourly[i] > max)
        {
          max = hourly[i];
        }
      }
    }
    range = max - min;
    rateofchange = range / max;
    weightfactor = 1 - rateofchange;
    if (weightfactor < .5 )
    {
      weightfactor = .5;
    }
    /// step 4 here.
    for (i = 0; i < items; i++)
    {
      if (hourly[i] != -1)
      {
        sumofdatatimesweightingfactor += hourly[i] * (pow(weightfactor, i));
        sumofweightingfactor += pow(weightfactor, i);
      }
    }
    nowCast = sumofdatatimesweightingfactor / sumofweightingfactor;
    nowCast = floor(10 * nowCast) / 10;
  }
  return (nowCast);
}

// update local OLED display
void updateOLED() {

  OLED.setTextColor(WHITE, BLACK);
  OLED.clearDisplay(); //
  OLED.setTextSize(1); //
  OLED.setCursor(0, 0); // reset cursor to origin
  OLED.print("   SSV BackpAQ ");
  OLED.println(version); // version from config.h
  OLED.setTextSize(1); // smallest size
  // Particulates
  OLED.println("  ");
  OLED.print("   PM1.0 = ");
  OLED.print(dustvalues1.PM01Val_cf1); // write PM1.0 data
  OLED.println(" ug/m3");

  OLED.print("   PM2.5 = ");
  OLED.print(dustvalues1.PM2_5Val_cf1); // write PM2.5 data
  OLED.println(" ug/m3");

  OLED.print("   PM10  = ");
  OLED.print(dustvalues1.PM10Val_cf1); // write PM10 data
  OLED.println(" ug/m3");

  //OLED.print("   PM2.5 AQI  = ");
  //OLED.println(airQualityIndex); // write AQI
  // Gas Phase
  OLED.print("   CO2 = ");
  OLED.print(co2);
  OLED.println(" ppm");

  // Sound Levels
  OLED.print("   Noise = ");
  OLED.print(currLoudnessDBA.toInt());
  OLED.println(" dB(A)");

  OLED.display(); //  display data on OLED
}
/***------------------------------------------------------------------------- */
/**************************** Send To Thinkspeak and SDCard Log *****************************/
/***------------------------------------------------------------------------- */
void SendToThingspeakWF(void) // Send to Thingspeak using web services call
{

  Serial.println("Writing to Thingspeak and logging...");

  ThingSpeak.begin(client1);  // Initialize ThingSpeak API endpoint

  String def_latlon = default_gps_position; // get default GPS position from metadata

  // Parse Lat-lon string
  int comma = def_latlon.indexOf(",", 0);
  String default_lat_str = def_latlon.substring(0, comma - 1); // extract lat
  String default_lon_str = def_latlon.substring(comma + 1, 18); // extract lon

  // convert to float
  default_lat  = default_lat_str.toFloat();
  default_lon = default_lon_str.toFloat();

  if (latI == 0.0 && lastKnownLat > 0.0) { // if no sat lock from GPS
    latI = lastKnownLat; // use default lat, lon
    lonI = lastKnownLon;
    Serial.println("No GPS...using last known GPS position");
  }

  Serial.print("Latitude: ");  Serial.print(latI, 7);
  Serial.print(", Longitude: ");  Serial.println(lonI, 7);

  // get time from RTC and build timestamp

  DateTime now = rtc.now();
  String timestamp = "";
  timestamp += now.year();
  timestamp += '-';
  if(now.month() < 10)   { timestamp += '0';}
  timestamp += now.month();
  timestamp += '-';
  if(now.day() < 10)   { timestamp += '0';}
  timestamp += now.day();
  timestamp += "T";
  // Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  // Serial.print(") ");
  if(now.hour() < 10)   { timestamp += '0';}
  timestamp += now.hour();
  timestamp += ':';
  if(now.minute() < 10)   { timestamp += '0';}
  timestamp += now.minute();
  timestamp += ':';
  if(now.second() < 10)   { timestamp += '0';}
  timestamp += now.second();
  String timestamp1 = timestamp;
  timestamp += " ";
  timestamp += "-0800"; // add UTC adjustment for Pacific TZ
  Serial.print("Timestamp: ");
  Serial.println(timestamp);

 // Build log file names...because FAT we have only "8.3" to work with
 // Three files: The A "Data" file; the B "Conc(entrations) file; the C "Track(s)" file

 // prototype: "YAQA-5"
 int bpLen = sensorName.length();
 String bpNum = sensorName.substring(bpLen-1,bpLen); // grab just backpaq device # ("-1" - "-99")
 bpNum.replace("-", "0");
 Serial.print("Logging started for BackpAQ ");
 Serial.println(sensorName);
 
 // A
 BackpAQ_Logfile_A = bpNum; // start off with device ID number (ex: "05")
 //BackpAQ_Logfile_A += "A-";
 BackpAQ_Logfile_A += "-Data";
 /*
 if(now.month() < 10)   { BackpAQ_Logfile_A += '0';}
 BackpAQ_Logfile_A += now.month();
 if(now.day() < 10)     { BackpAQ_Logfile_A += '0';}
 BackpAQ_Logfile_A += now.day();
 */
 //BackpAQ_Logfile_A += "_";
 //BackpAQ_Logfile_A += timestamp1;
 BackpAQ_Logfile_A += ".csv";

 // B
 BackpAQ_Logfile_B = bpNum;
 //BackpAQ_Logfile_B += "B-";
 BackpAQ_Logfile_B += "-Conc";
 /*
 if(now.month() < 10)   { BackpAQ_Logfile_B += '0';}
 BackpAQ_Logfile_B += now.month();
 if(now.day() < 10)     { BackpAQ_Logfile_B += '0';}
 BackpAQ_Logfile_B += now.day();
 */
 //BackpAQ_Logfile_B += "_";
 //BackpAQ_Logfile_B += timestamp1;
 BackpAQ_Logfile_B += ".csv";

 // C
  BackpAQ_Logfile_C = bpNum;
 //BackpAQ_Logfile_C += "-C";
 //BackpAQ_Logfile_C += "C-";
 BackpAQ_Logfile_C += "-Track";
 /*
 if(now.month() < 10)   { BackpAQ_Logfile_C += '0';}
 BackpAQ_Logfile_C += now.month();
 if(now.day() < 10)     { BackpAQ_Logfile_C += '0';}
 BackpAQ_Logfile_C += now.day();
 */
// BackpAQ_Logfile_C += "_";
// BackpAQ_Logfile_C += timestamp1;
 BackpAQ_Logfile_C += ".csv";

 // Verify SDcard inserted, ready to Log

 byte status = myLog.getStatus();

  Serial.print("Status byte: 0x");
  if(status < 0x10) Serial.print("0");
  Serial.println(status, HEX);

  if(status & 1<<STATUS_SD_INIT_GOOD)
    Serial.println("SD card is good");
  else
    Serial.println("SD init failure. Is the SD card present? Formatted?");

  if(status & 1<<STATUS_IN_ROOT_DIRECTORY)
    Serial.println("Root directory open");
  else
    Serial.println("Root failed to open. Is SD card present? Formatted?");

  if(status & 1<<STATUS_FILE_OPEN)
    Serial.println("Log file open and ready for recording");
  else
    Serial.println("No log file open. Use append command to start a new log.");

 //Get size of file
  long sizeOfFile = myLog.size(BackpAQ_Logfile_A);

  if (sizeOfFile == -1 ) //  file and header does not exist
   {
    needHeader = true;
   }

  sensorID ++; // iterate Sensor ID #

  // -------------------------------------------------------
  // CHANNEL 1 dataframe: Update the first channel with PM1, PM2.5, PM10 + temp, humidity, lat, lon, CO2
  //--------------------------------------------------------

  ThingSpeak.setField(1, dustvalues1.PM01Val_atm);
  ThingSpeak.setField(2, dustvalues1.PM2_5Val_atm);
  ThingSpeak.setField(3, dustvalues1.PM10Val_atm);
  ThingSpeak.setField(4, senTemp);
  ThingSpeak.setField(5, senHumid);
  ThingSpeak.setField(6, latI); //
  ThingSpeak.setField(7, lonI); //
  //ThingSpeak.setField(8, airQualityIndex);
  ThingSpeak.setField(8, co2);
  ThingSpeak.setStatus(currSpectrumA); // sneaky!
  ThingSpeak.setElevation(currLoudnessDBA.toFloat()); // sneaky!
  ThingSpeak.setLatitude(currLoudnessDBC.toFloat()); //
  ThingSpeak.setLongitude(currLoudnessDBZ.toFloat()); //
  ThingSpeak.setCreatedAt(timestamp); //

  // write to the ThingSpeak channel A

  //  char thingspeak_A_key[20] = "xxxxxxxxxxxxxxxx";

  thingspeak_A_key_c = thingspeak_A_key_s.c_str();

  //thingspeak_A_key_c = thingspeak_A_key; // cast to const char *
  Serial.print("A Key is "); Serial.println(thingspeak_A_key_c);
  Serial.print("A Channel is "); Serial.println(thingspeak_A_channel_i);
  
  if (!standalone)
  {
    int x = ThingSpeak.writeFields(thingspeak_A_channel_i, thingspeak_A_key_c);
    if (x == 200) {
      Serial.println("Thingspeak channel 1 update successful.");
    }
    else {
      Serial.println("Problem updating channel. HTTP error code " + String(x));
    }

    delay(1000); // take a breath...
  }


 // write to A LOG
    myLog.append(BackpAQ_Logfile_A);  // set log file
    delay(500);
    
 if(needHeader)
 {
    //print out column headers
    while (label_A) { //runs once
      // write log headers
      
      myLog.println("ccreated_at, SensorID, PM1.0 (ug/m3), PM2.5 (ug/m3), PM10 (ug/m3), Temperature (deg F), Humidity (%), Latitude (Deg), Longitude (Deg), CO2 (ppm)");
      label_A = false;
    }
 }
   
  delay(200);
    
    // write to LOG file
    Serial.print("Writing to log file ");
    Serial.println(BackpAQ_Logfile_A);
    
    myLog.print("");
    myLog.print(timestamp);
    myLog.print(",");
    myLog.print(sensorID);
    myLog.print(",");
    myLog.print(PM1_0);
    myLog.print(",");
    myLog.print(PM2_5);
    myLog.print(",");
    myLog.print(PM10_0);
    myLog.print(",");
    myLog.print(senTemp);
    myLog.print(",");
    myLog.print(senHumid);
    myLog.print(",");
    myLog.print(latI,4); // to 4 places
    myLog.print(",");
    myLog.print(lonI,4); // to 4 places
    myLog.print(",");
    myLog.println(co2);
    myLog.syncFile(); // flush
 // }
  
  // ----------------------------------------------------------
  // CHANNEL 2 dataframe: Update the second channel
  // set the fields with the concentration values + noise in Db(A) and CO2
  // ----------------------------------------------------------
  ThingSpeak.setField(1, timeHack);
  ThingSpeak.setField(2, dustvalues1.Beyond05);
  ThingSpeak.setField(3, dustvalues1.Beyond1);
  ThingSpeak.setField(4, dustvalues1.Beyond2_5);
  ThingSpeak.setField(5, dustvalues1.Beyond5);
  ThingSpeak.setField(6, dustvalues1.Beyond10);
  ThingSpeak.setField(7, currLoudnessDBA);
  ThingSpeak.setField(8, co2);
  ThingSpeak.setCreatedAt(timestamp); //

  // write to the ThingSpeak channel B
  // thingspeak_B_key_c = thingspeak_B_key; // cast to const char *
  thingspeak_B_key_c = thingspeak_B_key_s.c_str();
  Serial.print("B Key is "); Serial.println(thingspeak_B_key_c);
  Serial.print("B Channel is "); Serial.println(thingspeak_B_channel_i);

  if (!standalone)
  {
    int xx = ThingSpeak.writeFields(thingspeak_B_channel_i, thingspeak_B_key_c);
    if (xx == 200) {
      Serial.println("Thingspeak channel 2 update successful.");
    }
    else {
      Serial.println("Problem updating channel. HTTP error code " + String(xx));
    }

    delay(500);
  }
 // else
  //{
  // write to LOG file
  
    myLog.append(BackpAQ_Logfile_B);
    delay(500);
    
 if (needHeader)
 {
    //print out column headers
    while (label_B) { //runs once
      // write log headers
      
      myLog.println("ccreated_at, Sensor ID,Beyond05, Beyond1, Beyond2.5, Beyond5, Beyond10, Loudness(DBA), CO2(ppm)");
      label_B = false;
    }
 }

  delay(200);
 
    Serial.print("Writing to log file ");
    Serial.println(BackpAQ_Logfile_B);

    myLog.print("");
    myLog.print(timestamp);
    myLog.print(",");
    myLog.print(sensorID);
    myLog.print(",");
    myLog.print(dustvalues1.Beyond05);
    myLog.print(",");
    myLog.print(dustvalues1.Beyond1);
    myLog.print(",");
    myLog.print(dustvalues1.Beyond2_5);
    myLog.print(",");
    myLog.print(dustvalues1.Beyond5);
    myLog.print(",");
    myLog.print(dustvalues1.Beyond10);
    myLog.print(",");
    myLog.print(currLoudnessDBA);
    myLog.print(",");
    myLog.println(co2);
    myLog.syncFile(); // flush

   delay(1000);
 // }
  
  //--------------------------------------------------------------------
  // CHANNEL 3 dataframe: Write track, lat, lon, PM, CO2 & comment data to Thingspeak
  // Note use of special fields (status, elevation lat, lon) to send sound values
  // -------------------------------------------------------------------
  String names = String(userID) + "," + trackName;
  //Serial.println("names = " + names);
  ThingSpeak.setField(1, session); // Session #
  ThingSpeak.setField(2, latI); // GPS pos
  ThingSpeak.setField(3, lonI);
  ThingSpeak.setField(4, dustvalues1.PM2_5Val_atm); // PM2.5
  ThingSpeak.setField(5, trackName); // session or track description
  ThingSpeak.setField(6, iconURL);  // URL for PM-color icon
  ThingSpeak.setField(7, co2); //  CO2 value (if activated)
  ThingSpeak.setField(8, track_comment_array[track_id]); // attach any comments
  ThingSpeak.setStatus(sensorName); // BackpAQ name
  ThingSpeak.setElevation(currLoudnessDBA.toFloat()); // sneaky!
  ThingSpeak.setLatitude(currLoudnessDBC.toFloat()); //
  ThingSpeak.setLongitude(currLoudnessDBZ.toFloat()); //
  ThingSpeak.setCreatedAt(timestamp); //

  // write to the ThingSpeak channel
  thingspeak_C_key_c = thingspeak_C_key_s.c_str();
  // thingspeak_C_key_c = thingspeak_C_key; // cast to const char
  Serial.print("C Key is "); Serial.println(thingspeak_C_key_c);
  Serial.print("C Channel is "); Serial.println(thingspeak_C_channel_i);

  if (!standalone)
  {
    int x = ThingSpeak.writeFields(thingspeak_C_channel_i, thingspeak_C_key_c);
    if (x == 200) {
      Serial.println("Thingspeak channel 3 update successful.");
    }
    else {
      Serial.println("Problem updating channel. HTTP error code " + String(x));
    }

    delay(500);
  }
 
  // write to LOG file
    myLog.append(BackpAQ_Logfile_C);
    delay(500);

  if (needHeader)
  {
    //print out column headers
    while (label_C) { //runs once
      // write log headers
   
      myLog.println("ccreated_at, Sensor ID, user, Latitude, Longitude, PM2.5 (ug/m3), TrackName, icon, CO2 (ppm), Comments");
      label_C =  false;
    }
  }
    Serial.print("Writing to log file ");
    Serial.println(BackpAQ_Logfile_C);

    myLog.print("");
    myLog.print(timestamp);
    myLog.print(",");
    myLog.print(sensorID);
    myLog.print(",");
    myLog.print(userID);
    myLog.print(",");
    myLog.print(latI,4);
    myLog.print(",");
    myLog.print(lonI,4);
    myLog.print(",");
    myLog.print(PM2_5);
    myLog.print(",");
    myLog.print(trackName);
    myLog.print(",");
    myLog.print(iconURL);
    myLog.print(",");
    myLog.print(co2);
    myLog.print(",");
    myLog.println(track_comment_array[track_id]);
    myLog.syncFile(); // flush

    Serial.println("Finished writing to log.");

  needHeader = false;
}

// Send to Thingspeak using Blynk Webhook from smartphone
void SendToThingspeakWH(void)
{
  Serial.println("Writing to Thingspeak, webhook.");
  /* Channel 1: Thingspeak Field:        1            2                      3                       4  5  6     7     8                 */
  Blynk.virtualWrite(V7, dustvalues1.PM01Val_atm, dustvalues1.PM2_5Val_atm, dustvalues1.PM10Val_atm, senTemp, senHumid, latI, lonI, airQualityIndex);
  Serial.println("Thingspeak channel 1 update successful.");
  delay(1000);
  /* Channel 2 Thingspeak Field:        1             2                   3                     4                    5                    6                   7       8      */
  Blynk.virtualWrite(V47, timeHack, dustvalues1.Beyond05, dustvalues1.Beyond1, dustvalues1.Beyond2_5, dustvalues1.Beyond5, dustvalues1.Beyond10, TVOC, eCO2);
  Serial.println("Thingspeak channel 2 update successful.");
  delay(1000);

}

void readLog()
{
  //Get size of A B C files
  int sizeOfFileA = myLog.size(BackpAQ_Logfile_A);
  int sizeOfFileB = myLog.size(BackpAQ_Logfile_B);
  int sizeOfFileC = myLog.size(BackpAQ_Logfile_C);

  if (sizeOfFileA > -1)
  {
    Serial.print(F("Size of file A: "));
    Serial.println(sizeOfFileA);
    Serial.print(F("Size of file B: "));
    Serial.println(sizeOfFileB);
    Serial.print(F("Size of file C: "));
    Serial.println(sizeOfFileC);

    //Read the contents of each log file by passing a buffer into .read()
    //Then printing the contents of that buffer

    byte myBufferSize = 500; // big enough
    byte myBuffer[myBufferSize];

    // A file

    //myLog.read(myBuffer, myBufferSize, myFileA, 4); //Doesn't yet work
   // myLog.read(myBuffer, myBufferSize, BackpAQ_Logfile_A); //Load myBuffer with contents of myFile

    // Parse CSV string
   // String data = Papa.parse(myBuffer);
   // Serial.print(myBuffer);

    //Print the contents of the buffer
    //Serial.println("File contents A:");
    //for (int x = 0 ; x < myBufferSize ; x++)
    //{
    //  Serial.write(myBuffer[x]);
    //}

   // Serial.println("\nDone with file A contents");

    // B file

    //myLog.read(myBuffer, myBufferSize, myFileA, 4); //Doesn't yet work
  //  myLog.read(myBuffer, myBufferSize, BackpAQ_Logfile_B); //Load myBuffer with contents of myFile

    //Print the contents of the buffer
   // Serial.println("File contents B:");
   // for (int x = 0 ; x < myBufferSize ; x++)
   ///  Serial.write(myBuffer[x]);
   // }
  //  Serial.println("\nDone with file B contents");

    // C file


    //myLog.read(myBuffer, myBufferSize, myFileA, 4); //Doesn't yet work
  //  myLog.read(myBuffer, myBufferSize, BackpAQ_Logfile_C); //Load myBuffer with contents of myFile

    //Print the contents of the buffer
    //Serial.println("File contents C:");
    // Serial.write(myBuffer[x]);
    //}
   // Serial.println("\nDone with file C contents");

  }
  else
  {
    Serial.println(F("Size error: Log file not found"));
  }

  Serial.println(F("All Done!"));

}

void updateThingspeak()
{
  Serial.println("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
  Serial.println("Updating Thingspeak with log data!");
  Serial.println("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
  
 String serverT = "18.204.224.126"; // ThingSpeak Server
 String channel = "1283582";
  const char* timestamp = "2022-12-14T22:4:40 -800";

}

void getRTCDate()
{

  DateTime now = rtc.now();

  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();

  Serial.print(" since midnight 1/1/1970 = ");
  Serial.print(now.unixtime());
  Serial.print("s = ");
  Serial.print(now.unixtime() / 86400L);
  Serial.println("d");
}

void SendToThingspeak() // send data to Thingspeak cloud, check first for offline mode
{


  SendToThingspeakWF(); // send data to Thingspeak using WiFi and API call

  // }
}

void mapGPS()
{

  // ---------------------------------------------------------------------------------------------
  // Display tracks on Blynk MAP page using GPS position and PM values
  // Lat and Lon are also stored in ThingSpeak via another function
  // ---------------------------------------------------------------------------------------------

  sprintf(pmLabel, "PM2.5:%4dug/m3\n", dustvalues1.PM2_5Val_atm); // use PM2.5 value for marker label

  Blynk.virtualWrite(V5, markerNum, latID, lonID, pmLabel, newColor); // update marker position=GPS, color=PMAQI (marker color hack for iOS)

  //Blynk.virtualWrite(V5, markerNum, latID, lonID, pmLabel); // update marker position=GPS, note no marker color!!!
  Blynk.virtualWrite(V7, lonID, latID); // this is for web console Map
  //   Blynk.virtualWrite(V7, 1, lonI, latI, "tag");

  int Index = 1;

  markerNum++; // increment marker number
  latISav[markerNum] = latI; // save lasy known pos
  lonISav[markerNum] = lonI;

  // Update Blynk
  Blynk.virtualWrite(V27, latID);
  Blynk.virtualWrite(V12, latID);
  Blynk.virtualWrite(V13, lonID);
  Blynk.virtualWrite(V23, speed);
  Blynk.virtualWrite(V24, heading);

}

// get GPS position from BackpAQ GPS
void getGPS() {

  String line1, line2;

  // Using Sparkfun GPS module
  myGNSS.checkUblox(); //See if new data is available. Process bytes as they come in.

  if (nmea.isValid() == true) // if valid data go get it
  {
    Serial.println("Valid GPS Data!");
    long latitude_mdeg = nmea.getLatitude();
    long longitude_mdeg = nmea.getLongitude();
    //  speed = nmea.getSpeed() / 1000. ;
    //  heading = nmea.getCourse() / 1000. ;
    speed = myGNSS.getGroundSpeed();
    heading = myGNSS.getHeading();

    latI = latitude_mdeg / 1000000.;
    lonI = longitude_mdeg / 1000000.;

    latID = latitude_mdeg / 1000000.;
    lonID = longitude_mdeg / 1000000.;

    line1 = String("lat: ") + String(latI, 6);
    line2 = String("lng: ") + String(lonI, 6);

    Serial.print("latID: ");
    Serial.println(latID);
    Serial.print(F(" Speed: "));
    Serial.print(speed);
    Serial.print(F(" (mm/s)"));
    Serial.print(F(" Heading: "));
    Serial.print(heading);
    Serial.println(F(" (degrees)"));

    //mapGPS(); //update location on Blynk map

    // Update Blynk
    // Blynk.virtualWrite(V27, latID);
    // Blynk.virtualWrite(V12, latID);
    // Blynk.virtualWrite(V13, lonID);
    // Blynk.virtualWrite(V23, speed);
    // Blynk.virtualWrite(V24, heading);


    nmea.clear(); // Clear the MicroNMEA storage to make sure we are getting fresh data
  }
  else
  {
    Serial.println("Waiting for fresh data");
  }

}
//This function gets called from the SparkFun u-blox Arduino Library
//As each NMEA character comes in you can specify what to do with it
//Useful for passing to other libraries like tinyGPS, MicroNMEA, or even
//a buffer, radio, etc.
void SFE_UBLOX_GNSS::processNMEA(char incoming)
{
  //Take the incoming char from the u-blox I2C port and pass it on to the MicroNMEA lib
  //for sentence cracking
  nmea.process(incoming);
}

// get, format current date and time via Blynk RTC
void getTimeDate()
{
  currentTime = String(hour()) + ":" + minute() + ":" + second();
  currentDate = String(day()) + " " + month() + " " + year();
  /* Serial.print("Current time: ");
    Serial.println(currentTime);
    Serial.print("Current date: ");
    Serial.println(currentDate); */

  timeHack = currentDate + " " + currentTime;

}
// ----------------------------------------------------------------------------------------
// get PM data from Plantower sensor
// ----------------------------------------------------------------------------------------
void getPMData(void) {
  PM25_AQI_Data data;

  // reading data via I2C (not serial/UART)
  if (! aqi.read(&data)) {
    Serial.println(F("Could not read PM sensor"));
    delay(500);  // try again in a bit!
    return;
  }
  Serial.println("PM sensor reading success");

  /* "data" structure */
  /* PM01Val_atm == Byte 10&11  (ATM is bytes 10-15)
    uint16_t pm10_standard,  ///< Standard PM1.0
       pm25_standard,       ///< Standard PM2.5
       pm100_standard;      ///< Standard PM10.0
    uint16_t pm10_env,       ///< Environmental PM1.0
       pm25_env,            ///< Environmental PM2.5
       pm100_env;           ///< Environmental PM10.0 */

  tempconcPM1_0_amb += data.pm10_env; // pick up basic PM1.0 ENV value, add to total
  tempconcPM2_5_amb += data.pm25_env; // pick up basic PM2.5 ENV value, add to total
  tempconcPM10_0_amb += data.pm100_env; // pick up basic PM10.0 ENV value, add to total

  // store concentration values for Blynk, ThingSpeak
  dustvalues1.Beyond03 = data.particles_03um;
  dustvalues1.Beyond05 = data.particles_05um;
  dustvalues1.Beyond1 =  data.particles_10um;
  dustvalues1.Beyond2_5 = data.particles_25um;
  dustvalues1.Beyond5 =  data.particles_50um;
  dustvalues1.Beyond10 = data.particles_100um;

  plantowercount++; // increment # of samples for avg
}

void doPMSensor_average() { //Sensor data Average over the number of readings taken

  if (plantowercount == 0) {
    //Error from Plantower with zero count do nothing
  }

  // compute average over samples
  if (plantowercount != 0) {
    // concPM1_0_CF1 = tempconcPM1_0_CF1 / plantowercount;
    // concPM2_5_CF1 = tempconcPM2_5_CF1 / plantowercount;
    // concPM10_0_CF1 = tempconcPM10_0_CF1 / plantowercount;
    concPM1_0_amb = double(tempconcPM1_0_amb) / double(plantowercount * 1.0);
    concPM2_5_amb = double(tempconcPM2_5_amb) / double(plantowercount * 1.0);
    concPM10_0_amb = double(tempconcPM10_0_amb) / double(plantowercount * 1.0);

    if (!calibration) // if not using calibrated values for PM data ("raw values")
    {
    /* {assign values "raw" without slope fit */
    
       Serial.println("Using raw PM values.");
       PM1_0 = concPM1_0_amb;
       PM2_5 = concPM2_5_amb;
       PM10_0 = concPM10_0_amb;
    }
    else
    {
    /* apply AIRBeam correction factors from slope fit */
    /* Algorithm courtesy of AirBeam Open Source V3    */
    
    Serial.println("Using slope fit calibration with PM values.");
    if (concPM1_0_amb < 22.14) {
      PM1_0 = (1.0566 * concPM1_0_amb);
    }
    else if (concPM1_0_amb > 22.13 && concPM1_0_amb < 37.95) {
      PM1_0 = (-30.9677 + (2.4554 * concPM1_0_amb));
    }
    else if (concPM1_0_amb > 37.94) {
      PM1_0 = (11.2026 + (1.3442 * concPM1_0_amb));
    }

    if (PM1_0 < 30.16) {
      PM2_5 = (1.0888 * PM1_0);
    }
    else if (PM1_0 > 30.15 && PM1_0 < 126.85) {
      PM2_5 = (-5.84 + (1.2826 * PM1_0));
    }
    else if (PM1_0 > 126.84) {
      PM2_5 = (-25.94 + (1.441 * PM1_0));
    }

    if (PM2_5 < 20.0) {
      PM10_0 = (1.05 * PM2_5);
    }
    else if (PM2_5 > 19.99) {
      PM10_0 = -19.0 + (2.0 * PM2_5);
    }
   } // calibration
  } // plantower count

  // print to console
  Serial.println();
  //Serial.println(F("---------------------------------------"));
  // Serial.print(F("PM 1.0: ")); Serial.print(data.pm10_standard);
  // Serial.print(F("\t\tPM 2.5: ")); Serial.print(data.pm25_standard);
  // Serial.print(F("\t\tPM 10: ")); Serial.println(data.pm100_standard);
  Serial.println(F("Concentration Units (environmental)"));
  Serial.println(F("---------------------------------------"));
  Serial.print(F("# of samples: "));
  Serial.println(plantowercount);
  Serial.print(F("PM 1.0: ")); Serial.print(int(round(PM1_0)));
  Serial.print(F("\t\tPM 2.5: ")); Serial.print(int(round(PM2_5)));
  Serial.print(F("\t\tPM 10: ")); Serial.println(int(round(PM10_0)));
  Serial.println(F("---------------------------------------"));
  // Serial.print(F("Particles > 0.5um / 0.1L air:")); Serial.println(data.particles_05um);
  // Serial.print(F("Particles > 1.0um / 0.1L air:")); Serial.println(data.particles_10um);
  // Serial.print(F("Particles > 2.5um / 0.1L air:")); Serial.println(data.particles_25um);
  // Serial.print(F("Particles > 5.0um / 0.1L air:")); Serial.println(data.particles_50um);
  // Serial.print(F("Particles > 10 um / 0.1L air:")); Serial.println(data.particles_100um);
  // Serial.println(F("---------------------------------------"));

  // assign dustvalues to support old structure
  dustvalues1.PM01Val_atm = PM1_0;
  dustvalues1.PM2_5Val_atm = PM2_5;
  dustvalues1.PM10Val_atm = PM10_0;

  dustvalues1.PM01Val_cf1 = PM1_0;
  dustvalues1.PM2_5Val_cf1 = PM2_5;
  dustvalues1.PM10Val_cf1 = PM10_0;

  //  Three “conversions” to adjust PM2.5 concentrations and corresponding AQI values for woodsmoke, an “AQandU” calibration (0.778 * PA + 2.65)
  //  to a long-term University of Utah study in Salt Lake City and an “LRAPA” calibration (0.5 * PA − 0.68) to a Lane Regional Air
  //  Pollution Agency study of PA sensors. USEPA: PM2.5 (µg/m³) = 0.534 x PA(cf_1) - 0.0844 x RH + 5.604
  switch (convFactor)
  {
    case 1: // NO conversion
      Serial.println(F("None selected"));
      break;
    case 2: // US EPA
      Serial.println(F("US EPA selected"));
      PM1_0 = 0.534 * PM1_0 - .0844 * senHumid + 5.604;
      PM2_5 = 0.534 * PM2_5 - .0844 * senHumid + 5.604;
      PM10_0 = 0.534 * PM10_0 - .0844 * senHumid + 5.604;
      break;
    case 3: // LRAPA
      Serial.println(F("LRAPA selected"));
      PM1_0 = 0.5 * PM1_0 - .68;
      PM2_5 = 0.5 * PM2_5 - .68;
      PM10_0 = 0.5 * PM10_0 - .68;

      break;
    case 4: // AQndU
      Serial.println(F("AQandU selected"));
      PM1_0 = 0.778 * PM1_0 + 2.65;
      PM2_5 = 0.778 * PM2_5 + 2.65;
      PM10_0 = 0.778 * PM10_0 + 2.65;

      break;
    default:
      Serial.println(F("Unknown item selected"));
  } // switch

  if (!standalone)
  {
    // write concentration values to Blynk
    Serial.println("->->->->-> Updating Blynk...");
    Blynk.setProperty(V1, "label", "PM2.5 (" + conversion + ")"); // update PM2.5 label
  //   Blynk.setProperty(V0, "color", newColor); // update  color
  //  Blynk.setProperty(V0, "label", newLabel);  // update  label
    Blynk.virtualWrite(V0, int(round(PM1_0)));  
  //   Blynk.setProperty(V1, "color", newColor); // update  color
  //  Blynk.setProperty(V1, "label", newLabel);  // update  label
    Blynk.virtualWrite(V1, int(round(PM2_5)));
   //  Blynk.setProperty(V2, "color", newColor); // update  color
  //  Blynk.setProperty(V2, "label", newLabel);  // update  label
    Blynk.virtualWrite(V2, int(round(PM10_0)));
    
    Blynk.virtualWrite(V34, dustvalues1.Beyond03);
    Blynk.virtualWrite(V35, dustvalues1.Beyond05);
    Blynk.virtualWrite(V36, dustvalues1.Beyond1);
    Blynk.virtualWrite(V37, dustvalues1.Beyond2_5);
    Blynk.virtualWrite(V38, dustvalues1.Beyond5);
    Blynk.virtualWrite(V39, dustvalues1.Beyond10);

    // calculate AQI and set colors for Blynk
    airQualityIndex = calculate_US_AQI25(PM2_5); // use PM2.5 ATM for AQI calculation
    calcAQIValue(); // set AQI colors

    // Update colors/labels for AQI gauge on Blynk
    Blynk.setProperty(V6, "color", newColor); // update AQI color
    Blynk.setProperty(V6, "label", newLabel);  // update AQI label
    Blynk.virtualWrite(V6, gaugeValue);  // update AQI value
  }

  tempconcPM1_0_amb = 0;
  tempconcPM2_5_amb = 0;
  tempconcPM10_0_amb = 0;
  plantowercount = 0;

}
void doCO2Sensor_average() { //CO2 Sensor data Average over the number of readings taken

  if (SCD4Xcount == 0) {
    //Error from SCD4X with zero count do nothing
  }

  // compute average over samples
  if (SCD4Xcount != 0) {

    co2 = double(tempCO2) / double(SCD4Xcount * 1.0);
    float temp = double(tempTemp) / double(SCD4Xcount * 1.0);
    float humid = double(tempHumid) / double(SCD4Xcount * 1.0);

    Serial.print("Co2:");
    Serial.print(co2);
    Serial.print("\t");
    Serial.print("Temperature:");
    Serial.print(temp * 1.8 + 32);
    // Serial.print(temperature * 175.0 / 65536.0 - 45.0);
    Serial.print("\t");
    Serial.print("Humidity:");
    // Serial.println(humid * 100.0 / 65536.0);
    Serial.println(humid);
    senTemp = temp * 1.8 + 32; // to deg F
    senHumid = humid;

    // Update CO2 values to Blynk app
    String newCol;
    if (co2 >= 400 && co2 <= 700) {newCol = AQI_GREEN;}
    else if (co2 > 700 && co2 <= 1099) {newCol = "#32cd32";}
    else if (co2 >1099 && co2 <=1599) {newCol = AQI_YELLOW;}
    else if (co2 >= 1600 && co2 < 2000) {newCol = AQI_ORANGE;}
    else if (co2 > 2000) {newCol = AQI_RED;}
    
    Blynk.setProperty(V31, "color", newCol); // update  color
    Blynk.virtualWrite(V31, co2); // CO2 in PPM
    Blynk.virtualWrite(V3, temp * 1.8 + 32); // send F temp to Blynk
    Blynk.virtualWrite(V4, humid ); // send humidity to Blynk
  }


  // experimental: predict next CO2 values
  doCO2Prediction(); // do LR prediction for CO2 (BETA!)

  SCD4Xcount = 0;
  tempCO2 = 0;
  tempHumid = 0;
  tempTemp = 0;

}

// get CO2, temp and humidity values
void getCO2Data() {

  uint16_t error;

  char errorMessage[256];

  // Read Measurement
  uint16_t co22; // co2 is a global var
  // float temperature;
  // float humidity;
  //const float dCorr = 13.0; // est. temp correction due to interior case heating
  error = scd4x.readMeasurement(co22, temperature, humidity);
  if (error) {
    Serial.print(F("Error trying to execute readMeasurement(): "));
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }  else if ((co22 == 0) || (co22 > 5000)) {
    Serial.println(F("Invalid sample detected, skipping."));
  } else
  { // good to go!

    tempCO2 += co22; // pick up basic CO2 value, add to total
    tempTemp += temperature - tempOffset; // subtract offset
    tempHumid += humidity;
    SCD4Xcount++; // increment # of samples for avg
  }

}

void doCO2Prediction() {
  // Experimental function to attempt to predict CO2 from 10 recent values
  // calc linear regression for x data points using Least Squares

  counter ++;
  lr.Data(co2);

  if (counter > 9) {
    // calculate Linear Regression of last 10 data points

    Serial.print(lr.Samples()); Serial.println(F(" Point Linear Regression Calculation..."));
    Serial.print("Correlation: "); Serial.println(lr.Correlation());
    Serial.print("Values: "); lr.Parameters(values); Serial.print("Y = "); Serial.print(values[0], 4); Serial.print("*X + "); Serial.println(values[1], 4);
    Serial.print("Values: "); lr.Parameters(values); Serial.print(" a = "); Serial.print(values[0], 4); Serial.print(" b = "); Serial.println(values[1], 4);
    Serial.print("Degree(°): "); Serial.print(57.2957795 * atan(values[0]), 2); Serial.println(""); // convert rad to deg
    Serial.print("Time(s): "); Serial.print((1000 - values[1]) / values[0] * 5, 2); Serial.println("");

    lr.Samples();
    correlation = lr.Correlation();
    lr.Parameters(values);

    // calc forecast for each signal "phase": time until next "color"

    if (co2 < 800) trafficLight = "green";
    else if (co2 > 800 && co2 < 1000) trafficLight = "yellow";
    else trafficLight = "red";

    if (trafficLight == "green") {
      forecast = int((yellowLevel - values[1]) / values[0] * 5 / 60); // forecast time until 800 ppm, steps of 5s, in minutes
      Serial.println(yellowLevel); //
    }
    if (trafficLight == "yellow") {
      forecast = int((redLevel - values[1]) / values[0] * 5 / 60); // forecast time until 1000 ppm, steps of 5s, in minutes
      Serial.println(redLevel); //
    }
    Serial.print(correlation); //
    Serial.print(": y = "); Serial.print(values[0], 2); Serial.print("*x + "); Serial.println(values[1], 2); //
    Serial.println(trafficLight); //
    Serial.print("Time(min): "); Serial.print(forecast); Serial.println("");

    Blynk.virtualWrite(V45, values[0], 2);
    Blynk.virtualWrite(V46, values[1], 2);
    Blynk.virtualWrite(V47, correlation);
    Blynk.virtualWrite(V48, forecast);

    // Reset
    lr.Reset();
    counter = 0;
  }

  // if (correlation > 0.4 && forecast < 99) { // we've got a stable forecast and time < 99 min
  //   canvas.print(String(forecast));
  // } else {
  //   canvas.print("--");
}

// Retrieve battery voltage from ESP on BackpAQ device, send to Blynk/SmartPhone
// Note: cannot do this when WiFi is operating due to ADC1 conflict
void getBatteryVoltsBackpAQ() {

  bool batteryIsCritical = false;
  int int_battery = 0;
  int pct_battery = 0;
  const int MAX_ANALOG_VAL = 4095;
  const float MAX_BATTERY_VOLTAGE = 4.2; // Max LiPoly voltage of a 3.7V battery is 4.2V
  pinMode(batteryPin, INPUT);

  int_battery = (int)(battery_K * (float)analogRead(batteryPin));

  int raw_battery = analogRead(batteryPin);
  float flt_battery = (raw_battery / 4095.0);
  flt_battery *= 2;    // we divided by 2, so multiply back
  flt_battery *= 1.1; // reference voltage
  flt_battery *= 3.3;  // Multiply by 3.3V, our reference voltage
  Serial.print("VBat: " ); Serial.println(flt_battery);
  float batteryFraction = flt_battery / MAX_BATTERY_VOLTAGE;
  int batteryPct = batteryFraction * 100;
  Serial.println((String)"Raw:" + raw_battery + " Voltage:" + flt_battery + "V Percent: " + (batteryFraction * 100) + "%");

  // set battery indicator color based on voltage

  if (batteryPct >= 80) {
    batteryColor = "#009966"; // Green
  } else if (batteryPct > 60 && batteryPct < 80) {
    batteryColor = "#ffde33"; // Yellow
  } else if (batteryPct > 40 && batteryPct < 60) {
    batteryColor = "#ff9933"; // Orange
  } else { // battery below 40%
    batteryColor = "#cc0033";  // Red
   // Blynk.notify("Battery getting low...suggest you recharge soon.");
  }

  if (flt_battery < 3.3) {
    batteryIsCritical = true;
  }

  // send to Blynk

  Blynk.virtualWrite(V20, flt_battery); // update battery level (%) indicator
  Blynk.setProperty(V21, "color", batteryColor) ;  // update battery color
  Blynk.virtualWrite(V21, batteryPct); // update battery level (%) indicator
  //Blynk.virtualWrite(V49, int_battery); // for debug

  // test for low battery...
  if (batteryIsCritical) {
    //Blynk.notify(F("Battery level critical..time to recharge!"));
    // goToDeepSleep(); // go to sleep
  }
}

void initGPS()
{
  // set up GPS
  if (myGNSS.begin() == false)
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring."));
    //while (1);
  }
  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA); //Set the I2C port to output both NMEA and UBX messages
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
  myGNSS.setProcessNMEAMask(SFE_UBLOX_FILTER_NMEA_ALL); // Make sure the library is passing all NMEA messages to processNMEA
  //  myGNSS.setProcessNMEAMask(SFE_UBLOX_FILTER_NMEA_GGA); // Or, we can be kind to MicroNMEA and _only_ pass the GGA messages to it
  //  myGNSS.setProcessNMEAMask(SFE_UBLOX_FILTER_NMEA_VTG); // Or, we can be kind to MicroNMEA and _only_ pass the VTG messages to it
  myGNSS.setNMEAOutputPort(Serial);
}

void goToDeepSleep()
{
  Serial.println("Going to sleep...");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  //btStop();

  // adc_power_off();
  esp_wifi_stop();
  // esp_bt_controller_disable();

  // Configure the timer to wake us up!
  esp_sleep_enable_timer_wakeup(DEEP_SLEEP_TIME * 60L * 1000000L);

  // Go to sleep! Zzzz
  Serial.println("Going to sleep!");
  esp_deep_sleep_start();
  delay(2000); // make sure we go to sleep
}
void initCO2()
{

  uint16_t error;
  char errorMessage[256];

  scd4x.begin(Wire); // start the SCD4X sensor

  /* stop potentially previously started measurement */
  error = scd4x.stopPeriodicMeasurement();
  if (error) {
    Serial.print(F("Error trying to execute stopPeriodicMeasurement(): "));
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }
  uint16_t serial0;
  uint16_t serial1;
  uint16_t serial2;
  error = scd4x.getSerialNumber(serial0, serial1, serial2);
  if (error) {
    Serial.print(F("Error trying to execute getSerialNumber(): "));
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else {
    printSerialNumber(serial0, serial1, serial2);
  }
  /* Start CO2 Measurement */
  error = scd4x.startPeriodicMeasurement();
  if (error) {
    Serial.print(F("Error trying to execute startPeriodicMeasurement(): "));
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }
}
void updateWiFiSignalStrength() {

  rssi = WiFi.RSSI(); // fetch the received WiFi signal strength:
  Serial.print(F("WiFi signal strength (RSSI): "));
  Serial.print(rssi);
  Serial.println(" DBM");
  Blynk.virtualWrite(V56, rssi); // Post WiFi signal strength on Blynk
}

void powerOnSensor() {  // leaving sensor on for now...power used is small
  //Switch on PMS sensor
  // digitalWrite(PINPMSGO, HIGH);

  /*Warm-up */
  unsigned long timeout = millis();
  timeout = MIN_WARM_TIME - (millis() - timeout);
  if (timeout > 0) {
    DEBUG_PRINT("sensor warm-up: ");
    DEBUG_PRINTLN(timeout);
    delay(timeout);
  }
}

void powerOffSensor() {
  //#ifdef USE_WIFI
  // WiFi.disconnect();
  //#endif
  //Switch off PMS sensor
  //  digitalWrite(PINPMSGO, LOW);

  DEBUG_PRINTLN("going to sleep zzz...");
  delay(SLEEP_TIME);
}

void checkBlynkStatus() { // called every 3 seconds by SimpleTimer

  bool isconnected = Blynk.connected();
  if (isconnected == false) {
    wifiFlag = 1;
    digitalWrite(wifiLed, LOW);
    Serial.println("Blynk Not Connected");
  }
  if (isconnected == true) {
    wifiFlag = 0;
    digitalWrite(wifiLed, HIGH);
    //Serial.println("Blynk Connected");
  }
}

//char * boolstring( _Bool b ) { return b ? true_a : false_a; }

void setup()
{

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  setCpuFrequencyMhz(160); // default is 240mHz
  /*
    //Serial.println(getCpuFrequencyMhz());
    USB = digitalRead(V_USB);
    if (USBflag == true && USB == HIGH) {
      USBflag = false;
      setCpuFrequencyMhz(240);
      //blue(); //Comment out the top two lines to test the blue LED with USB detect
    }
    if (USBflag == false && USB == LOW) {
      USBflag = true;
      setCpuFrequencyMhz(80);
      //red(); //Comment out the top one line to test the red LED with USB detect
    }
  */
  Serial.begin(115200); // console

  Wire.begin(); // start Wire for I2C

  Wire.setClock(400000); //Increase  I2C clock rate to 400 kHz

  delay(100);

  if (!standalone)
  {
    BlynkEdgent.begin(); // start Blynk IOT with WiFi Provisoning and OTA
  }

  OLED.begin(0x3C, true); // init Feather OLED

  OLED.clearDisplay();
  OLED.setRotation(1);                               // rotate screen to fit Pelican case
  OLED.drawBitmap(0, 0, BackpAQ_Bitmap, 128, 64, WHITE); // display BackpAQ Logo screen
  OLED.display();
  delay(4000); // let spalash screen stay for 4 seconds
  OLED.clearDisplay();
  OLED.setRotation(1);                               // rotate screen
  OLED.setTextSize(1);                               // Set OLED text size to small
  OLED.setTextColor(WHITE, BLACK);                   // Set OLED color to White
  OLED.setCursor(0, 0);                              // Set cursor to 0,0
  OLED.print("SSV BackpAQ ");                        // display name, version, copyright
  OLED.println(version);
  // OLED.println("(c) 2022 SSV");                   // Choose to not display for now
  OLED.display();                                    // Update display
  delay(2000); // let copyright stay for 2 secs
  
  if (!standalone) // if WiFi and not standalone
  {
    if (Blynk.connected()) {
      DEBUG_PRINTLN("BLYNK Connected");
      OLED.println("BLYNK Connected!");   
    }
  }
  Serial.println(F("Starting up..."));

  // start RTC
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }
  rtc.start();

  // Start logging
  myLog.begin(); //Open connection to OpenLog

  initGPS(); // Initialize GPS sensor

  OLED.println(F("GPS Initialized"));
  if (standalone)
  {
    OLED.println(" ");
    OLED.println("< Standalone Mode >");
    OLED.println(" ");
  }
  OLED.display();

  /* Set up PM sensor */
  if (! aqi.begin_I2C(&Wire)) {      // connect to the PMS sensor over I2C
    //if (! aqi.begin_UART(&Serial1)) { // HELTEC connect to the sensor over hardware serial
    //if (! aqi.begin_UART(&pmSerial)) { // HELTEC connect to the sensor over software serial
    Serial.println(F("Could not find PM sensor!"));
  } else {
    Serial.println("PM25 found!");  // ok we're good
  }

  initCO2(); // Initialize CO2 sensor

  /* initialize the LED digital pin as an output */
  pinMode(PIN_LED, OUTPUT);

  Serial.setTimeout(1500);
  privateMode = 0;
  noBlynk = false;

  /* update OLED display with connection info... */

  if (!standalone)
  {
    OLED.print("Connect to: ");
    OLED.println(configStore.wifiSSID); // from Blynk Edgent
    OLED.display();
  }
  Serial.println("Starting up...");

  /* get PMS sensor ready...*/
  //    pinMode(PINPMSGO, OUTPUT); // Define PMS 7003 pin
  OLED.println(F("Sensor warming up ..."));
  OLED.display();
  OLED.setTextSize(1);
  previousMillis = millis();

  /* Switch on PM Sensor */
  DEBUG_PRINTLN(F("Switching On PMS"));
  powerOnSensor(); // wake up!
  DEBUG_PRINTLN(F("Initialization finished"));
  OLED.clearDisplay();

  //-------------------------------------------------------------------------
  // Set timers for Blynk
  // Stagger the times so as not to stack up timer
  // Also important to allow enough time for each task to complete
  //-------------------------------------------------------------------------
  timer.setInterval(PMSENSORINT, getPMData); // get data from PM sensor
  timer.setInterval(PMAVERAGEINT, doPMSensor_average); // average PM sensor readings + update blynk
  timer.setInterval(CO2SENSORINT, getCO2Data); // get CO2, temp & humidity from data SCD4x
  timer.setInterval(CO2AVERAGEINT, doCO2Sensor_average); // average CO2 sensor readings + update blynk
  timer.setInterval(TIMEDATEINT, getTimeDate); // get current time, date from RTC
  timer.setInterval(GPSINT, getGPS); // get GPS coords.
  if (!standalone) {
    timer.setInterval( MAPGPSINT, mapGPS); // update GPS position on Map screen
  }
  // timer.setInterval(60000L, updateWiFiSignalStrength); // get and update WiFi signal strength
  timer.setInterval(OLEDUPDATEINT, updateOLED); // update local display
  timer.setInterval(GETBATTERYVINT, getBatteryVoltsBackpAQ); // get battery level from BackpAQ
  timer.setInterval(THINGSPEAKSENDINT, SendToThingspeak); // update sensor values to Thingspeak every Send_Interval seconds
  //  timer.setInterval(SOUNDSENSORINT, getSoundData); // get noise values from sound sensor
  timer.setInterval(HEARTBEATINT, check_status); // Heartbeat
  timer.setInterval(BLINKLEDINT, blinkLedWidget);
 // timer.setInterval(30000L, readLog); // send log to TP


} // setup

void loop() // keep loop lean & clean!!
{
  if (!standalone) // Run Blynk if connected to WiFi/mobile
  {
    BlynkEdgent.run(); // for Blynk IOT & OTA!
  }
  
  timer.run();
}
