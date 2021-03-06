/*****************************************************************
* Heart-rate and SPO2 display using the MAX30102 sensor. 
* Logs data with a configurable interval to thingspeak.com using your 
* private channel # and API key.
* You can view the charted data in "real-time" on the ThingSpeak website.
* 
* Sensor initialization and readout code from Sparkfun 
* https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library
*  
* SP02 & pulse rate analysis from 
* https://github.com/aromring/MAX30102_by_RF  
* (algorithm by  Robert Fraczkiewicz)
* 
* WiFiManager SSID/password and ThingSpeak configuration magic from 
* https://github.com/tzapu/WiFiManager
* 
* Arduino Json 5 library
* https://github.com/bblanchon/ArduinoJson/tree/5.x
* 
* U8G2lib graphics library
*https://github.com/olikraus/U8g2_Arduino
*
* ThingSpeak library
*https://github.com/mathworks/thingspeak-arduino
******************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <FS.h>                   //this needs to be first, or it all crashes and burns...
#include <ESP8266WiFi.h>          
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          
#include <ArduinoJson.h>          
#include <pgmspace.h>
#include "ThingSpeak.h"
#include "algorithm_by_RF.h"
#include "MAX30105.h"
#include "U8g2lib.h"

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE); 

extern "C" {
    #include "user_interface.h"  // Required for wifi_station_connect() to work
}

// this library works for MAX30102 as well
MAX30105 sensor;
WiFiClient  client;

// config button pin for on-demand Configuration portal
#define pinCfg         0 

// pmos transistor switched power for OLED display
// drive low to turn on
#define pinOLEDPwr    13


uint32_t  IRBuffer[RFA_BUFFER_SIZE]; //infrared LED sensor data
uint32_t  RedBuffer[RFA_BUFFER_SIZE];  //red LED sensor data
int32_t   HeartRate; 
float     SPO2;
int       NumSamples;

// If pulse not detected in one minute, unit goes to sleep to save power
// (each cycle is 5 seconds of sampling)
#define SENSOR_TIMEOUT_CYCLES 12
int SensorWatchdogCounter = 0;

// if unable to publish data to ThingSpeak with 3 attempts, go to sleep
int ThingSpeakWatchdogCounter = 0;

// IIR filtered data 
float SPO2_iir = 0.0f;
float HeartRate_iir = 0.0f;

float BatteryVoltage;

// These parameters are retrieved from the SPIFFS JSON file
// If not found, the portal configuration is automatically started.
// You have 90s to connect to the  WiFi access point with SSID 
// 'SPO2_HeartRate', and then open the portal page at http://192.168.4.1 
// (if the webpage does not pop up automatically).
// Specify the Internet access point/password, and the 
// Thingspeak parameters, and click on Save.

char SzThingSpeakChannel[10] = {0};
char SzThingSpeakWriteAPIKey[20] = {0};
char SzThingSpeakUpdateSecs[5] = {0};

unsigned long ThingSpeakChannel = 0;
unsigned long ThingSpeakUpdateSecs;

bool FlagInternetAccess = false;
bool FlagSaveConfig = false;

// ThingSpeak update interval marker
unsigned long ThingSpeakTimeMarker;

void saveConfigCallback () {
  Serial.println("Parameters were modified, need to save config.json");
  FlagSaveConfig = true;
  }

// clear screen buffer, use large font to print SPO2 and 
// HeartRate and send to display
void oled_displayData(char* format, ...) {  
  char sz[10];
  va_list args;
  va_start(args, format);
  vsprintf(sz, format, args);
  va_end(args);
  u8g2.setFont(u8g2_font_logisoso24_tr);
  u8g2.clearBuffer();  
  u8g2.drawStr(14,0,sz);

  // battery status
  u8g2.drawLine(2,0,4,0);
  u8g2.drawFrame(0,1,8,16);
  if (BatteryVoltage >= 4.0f)
    u8g2.drawBox(2,3,4,12);
  else
  if (BatteryVoltage >= 3.9f)
    u8g2.drawBox(2,5,4,10);
  else
  if (BatteryVoltage >= 3.7f)
    u8g2.drawBox(2,7,4,8);
  else
  if (BatteryVoltage >= 3.6f)
    u8g2.drawBox(2,9,4,6);
  else 
  if (BatteryVoltage >= 3.5f)
    u8g2.drawBox(2,12,4,3);

  // internet access
  if (FlagInternetAccess) {
    u8g2.drawCircle(0,31,8, U8G2_DRAW_UPPER_RIGHT);
    u8g2.drawCircle(0,31,5, U8G2_DRAW_UPPER_RIGHT);
    u8g2.drawDisc(0,31,2, U8G2_DRAW_UPPER_RIGHT);
    }

  u8g2.sendBuffer(); 
  }

// For printing information and status to the buffer
// Use .sendBuffer() after finished to display 
void oled_printBuf(bool clearBuf, int x, int y, const uint8_t* font, char* format, ...) {  
  char sz[20];
  va_list args;
  va_start(args, format);
  vsprintf(sz, format, args);
  va_end(args);
  u8g2.setFont(font);
  if (clearBuf) {
    u8g2.clearBuffer(); 
    }
  u8g2.drawStr(x,y,sz);
  }  

  
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("SPO2/Heart-Rate Logger");
  Serial.print("Code compiled on "); Serial.print(__DATE__); Serial.print(" at "); Serial.println(__TIME__);
  pinMode(pinOLEDPwr, OUTPUT);
  // drive pmos switch transistor gate low to switch on power to OLED
  digitalWrite(pinOLEDPwr, 0);
  delay(100);
  
  BatteryVoltage = battery_SampleVoltage();
  u8g2.begin();
  u8g2.setFontPosTop();
  //u8g2.setFontDirection(0);
  oled_displayData("SPO2 HR");
  delay(2000);
    
  Serial.println("Mounting FS");
  if (SPIFFS.begin()) {
    Serial.println("Mounted file system");
    if (SPIFFS.exists("/config.json")) {
      Serial.println("Reading config.json file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("Opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);
        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nParsed json");
          strcpy(SzThingSpeakChannel, json["ts_channel"]);
          strcpy(SzThingSpeakWriteAPIKey, json["ts_wr_api_key"]);
          strcpy(SzThingSpeakUpdateSecs, json["ts_update_secs"]);
          } 
      else {
          oled_printBuf(true, 0, 0, u8g2_font_t0_14_mr, "No config file");
          oled_printBuf(false, 0, 16, u8g2_font_t0_14_mr, "found in SPIFFS");
          u8g2.sendBuffer();        
          Serial.println("Failed to load config.json");
          }
      }
    }
  } 
 else {
    oled_printBuf(true, 0, 0, u8g2_font_t0_14_mr, "Could not mount");
    oled_printBuf(false, 0, 16, u8g2_font_t0_14_mr, "SPIFFS file system");
    u8g2.sendBuffer();        
    Serial.println("Failed to mount FS");
    }

  Serial.printf("ThingSpeak Channel = %s\r\n", SzThingSpeakChannel);
  Serial.printf("ThingSpeak Write API Key = %s\r\n", SzThingSpeakWriteAPIKey);
  Serial.printf("ThingSpeak Update Seconds = %s\r\n", SzThingSpeakUpdateSecs);
  pinMode(pinCfg, INPUT);

  // If you want to change Internet Access Point SSID/Password, or ThingSpeak credentials : 
  // When you see the prompt, press the config button and keep it pressed until 
  // you see the portal startup message
  oled_printBuf(true, 0, 0, u8g2_font_t0_14_mr, "Press button for");
  oled_printBuf(false, 0, 16, u8g2_font_t0_14_mr, "IAP/TS config");
  u8g2.sendBuffer();         
  delay(3000);

  // If config button is pressed, or the Thingspeak credentials have not been initialized :
  // Starts an access point with the name "SPO2_HeartRate"
  // and goes into a blocking loop for up to 90 seconds, waiting for user to access the portal.
  // If you do not access the unit's portal page at http://192.168.4.1 within 90s, 
  // the unit will disable internet access and continue on to sensor initialization.
  int flagConfigRequired = (digitalRead(pinCfg) == 0) | (strlen(SzThingSpeakChannel) == 0)  | (strlen(SzThingSpeakWriteAPIKey) == 0)  ;

  if ( flagConfigRequired ) {
    WiFiManager wifiManager;
    oled_printBuf(true, 0, 0, u8g2_font_t0_14_mr, "Connect to AP");
    oled_printBuf(false, 0, 16, u8g2_font_t0_14_mr, "SPO2_HeartRate");
    u8g2.sendBuffer();  
    Serial.println("Starting configuration access point\r\n");
    WiFiManagerParameter custom_thingspeak_channel("ts_channel", "ThingSpeak Channel", SzThingSpeakChannel, 10);
    WiFiManagerParameter custom_thingspeak_write_api_key("ts_wr_api_key", "ThingSpeak Write API Key", SzThingSpeakWriteAPIKey, 20);
    WiFiManagerParameter custom_thingspeak_update_secs("ts_update_secs", "ThingSpeak Update Seconds", SzThingSpeakUpdateSecs, 5);
    wifiManager.setSaveConfigCallback(saveConfigCallback);
    wifiManager.addParameter(&custom_thingspeak_channel);
    wifiManager.addParameter(&custom_thingspeak_write_api_key);
    wifiManager.addParameter(&custom_thingspeak_update_secs);
   // set minimum signal strength of IAP SSIDs to show in the list.
   // reduce this if your IAP is not displayed.
    wifiManager.setMinimumSignalQuality(60);
    // configuration portal times out in 90 seconds
    wifiManager.setConfigPortalTimeout(90);
    // on-demand configuration portal
    if (wifiManager.startConfigPortal("SPO2_HeartRate", "")) {
      // configuration successful, now connected to Internet Access Point in station mode  
      oled_printBuf(true, 0, 0, u8g2_font_t0_14_mr, "Config OK");
      oled_printBuf(false, 0, 16, u8g2_font_t0_14_mr, "IAP %s",WiFi.SSID().c_str());
      u8g2.sendBuffer();  
      delay(2000);
      FlagInternetAccess = true;
      Serial.println("Return from Config Portal, connected as Wifi station");
      strcpy(SzThingSpeakChannel, custom_thingspeak_channel.getValue());
      strcpy(SzThingSpeakWriteAPIKey, custom_thingspeak_write_api_key.getValue());
      strcpy(SzThingSpeakUpdateSecs, custom_thingspeak_update_secs.getValue());
      if (FlagSaveConfig) {
        Serial.println("Changes made, saving config.json");
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.createObject();
        json["ts_channel"] = SzThingSpeakChannel;
        json["ts_wr_api_key"] = SzThingSpeakWriteAPIKey;
        json["ts_update_secs"] = SzThingSpeakUpdateSecs;
        File configFile = SPIFFS.open("/config.json", "w");
        if (!configFile) {
          Serial.println("Failed to open config.json file for writing");
          }
        json.prettyPrintTo(Serial);
        json.printTo(configFile);
        configFile.close();
        }
      }    
    else { // configuration failed       
        FlagInternetAccess = false;
        Serial.println("Failed to connect to Internet Access Point, and WiFi configuration timed out");
        oled_printBuf(true, 0, 0, u8g2_font_t0_14_mr, "Config portal");
        oled_printBuf(false, 0, 16, u8g2_font_t0_14_mr, "timed out");
        u8g2.sendBuffer();  
        delay(2000);
        }
    }
 else {
    Serial.println("Connecting in station mode with Internet Access Point SSID and password retrieved from flash");
    WiFi.mode(WIFI_STA);
    WiFi.begin(); // use SSID and password retrieved from flash
    int counter = 0;
    while ((counter < 10) && (WiFi.status() != WL_CONNECTED)) {
      delay(500);
      counter++;
      Serial.print(".");
      }   
    if (counter >= 10) {
      oled_printBuf(true, 0, 0, u8g2_font_t0_14_mr, "No internet");
      oled_printBuf(false, 0, 16, u8g2_font_t0_14_mr, "connection");
      u8g2.sendBuffer();
      delay(2000);
      FlagInternetAccess = false;
      WiFiOff(); // saves power (current draw down from 75mA down to 25mA)
      Serial.println("Unable to connect as Wifi client, continuing without Internet access");
      }      
    else {
      FlagInternetAccess = true;
      oled_printBuf(true, 0, 0, u8g2_font_t0_14_mr, "Connected to IAP");
      oled_printBuf(false, 0, 16, u8g2_font_t0_14_mr, "%s", WiFi.SSID().c_str());
      u8g2.sendBuffer();
      delay(1500);
      Serial.println("Connected as Wifi client");
      }
    }

  // ESP8266 tx power output 20.5dBm by default
  // we can lower this to reduce power supply noise caused by tx bursts
  WiFi.setOutputPower(12); 
  char* ptr;
  ThingSpeakChannel = strtoul(SzThingSpeakChannel, &ptr, 10);
  ThingSpeakUpdateSecs = strtoul(SzThingSpeakUpdateSecs, &ptr, 10);
  Serial.printf("ThingSpeak Channel Number = %lu\r\n", ThingSpeakChannel);
  Serial.printf("ThingSpeak Update Seconds = %lu\r\n", ThingSpeakUpdateSecs);
  
  if (sensor.begin(Wire) == false) {
    oled_printBuf(true, 0, 0, u8g2_font_t0_14_mr, "MAX30102 connection");
    oled_printBuf(false, 0, 16, u8g2_font_t0_14_mr, "problem");
    u8g2.sendBuffer();  
    delay(3000);
    shutDown();
    }
    
  // ref Maxim AN6409, average dc value of signal should be within 0.25 to 0.75 18-bit range (max value = 262143)
  // You should test this as per the app note depending on application : finger, forehead, earlobe etc. 
  // It even depends on skin tone. I found that the optimum combination for my index finger was :
  // ledBrightness=30, adcRange=2048, to get max dynamic range in the waveform, and a dc level > 100,000
  byte ledBrightness = 30; // 0 = off,  255 = 50mA
  byte ledMode = 2; // 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green (MAX30105 only)
  // net sampling rate = sampleRate/sampleAverage = 25
  int sampleRate = 100; // 50, 100, 200, 400, 800, 1000, 1600, 3200
  byte sampleAverage = 4; // 1, 2, 4, 8, 16, 32
  int pulseWidth = 411; // 69, 118, 215, 411
  int adcRange = 2048; // 2048, 4096, 8192, 16384
  
  sensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); 
  sensor.getINT1(); // clear the status registers by reading
  sensor.getINT2();
  NumSamples = 0;
  if (FlagInternetAccess) {
    Serial.println("Starting loop with InternetAccess");
    ThingSpeak.begin(client); 
    // Wifi radio is turned off to save power, turned on again for updates
    WiFiOff();
    }
  else {
      Serial.println("Starting loop with NO InternetAccess");
    }
  // get current time for thingspeak update  
  ThingSpeakTimeMarker = millis();
  }




void loop() {
  float ratio,correl; 
  int8_t  flagSPO2Valid = 0;  
  int8_t  flagHRValid  = 0;  

  sensor.check();
  while (sensor.available())   {
      RedBuffer[NumSamples] = sensor.getFIFORed(); 
      IRBuffer[NumSamples] = sensor.getFIFOIR();
      NumSamples++;
      sensor.nextSample(); 
      if (NumSamples == RFA_BUFFER_SIZE) {
        // calculate heart rate and SpO2 using RF algorithm, after RFA_BUFFER_SIZE samples are read
        rf_heart_rate_and_oxygen_saturation(IRBuffer, RFA_BUFFER_SIZE, RedBuffer, &SPO2, &flagSPO2Valid, &HeartRate, &flagHRValid, &ratio, &correl);     
        NumSamples = 0;
		    // periodically check the battery voltage  
        BatteryVoltage = battery_SampleVoltage();
        if (BatteryVoltage < 3.3f) {
          oled_printBuf(true, 0, 0, u8g2_font_t0_14_mr, "Low Battery Voltage");
          u8g2.sendBuffer();  
          delay(3000);
          shutDown();
          }

        if (flagHRValid && flagSPO2Valid) {
          SensorWatchdogCounter = 0; // got good data from sensor, feed the watchdog
          // apply damping IIR filter to SPO2 and heart-rate readings
          SPO2_iir = SPO2_iir > 0.1f ? 0.6f * SPO2_iir + 0.4f * SPO2 : SPO2;
          HeartRate_iir = HeartRate_iir > 0.1f ?  0.6f * HeartRate_iir + 0.4f * (float)HeartRate : (float)HeartRate;
          oled_displayData("%2d%% %3d", SPO2_iir >= 99.0f ? 99 : (int)(SPO2_iir+0.5f), (int)(HeartRate_iir+0.5f));
          }
        else {
          SensorWatchdogCounter++;
          if (SensorWatchdogCounter >= SENSOR_TIMEOUT_CYCLES) {
            oled_printBuf(true, 0, 0, u8g2_font_t0_14_mr, "No SPO2/Pulse data");
            oled_printBuf(false, 0, 16, u8g2_font_t0_14_mr, "for last minute");
            u8g2.sendBuffer();     
            delay(3000);
            shutDown();
            }
          else {     
          	// blank display if spo2/heart-rate could not be computed from the data
            oled_displayData("        ");
            }
          } 
        Serial.printf("%c SP02_IIR %.2f, Pulse_IIR %.0f\r\n", flagSPO2Valid && flagHRValid ? ' ' : 'x', SPO2_iir, HeartRate_iir);
            
        if (FlagInternetAccess) {
            if ((millis() - ThingSpeakTimeMarker) >  (ThingSpeakUpdateSecs*1000)){ 
              ThingSpeakTimeMarker = millis();
              Serial.printf("SPO2_iir = %.1f, HeartRate_iir = %.0f BatteryVoltage %.2f\r\n", SPO2_iir, HeartRate_iir, BatteryVoltage);
              updateThingSpeak(SPO2_iir, HeartRate_iir, BatteryVoltage);
              }
            }
          }        
        }
      }
 
  

// turn on Wifi only for the duration of the ThingSpeak connection 
// current draw with OLED display and WiFi on = ~75mA
// with OLED display on, WiFi off = ~30mA
void updateThingSpeak(float spo2, float heartRate, float BatteryVoltage) {
  WiFiOn();
  ThingSpeak.setField(1, spo2);
  ThingSpeak.setField(2, (int)(heartRate+0.5f));
  //ThingSpeak.setField(3, BatteryVoltage);
  int result = ThingSpeak.writeFields( ThingSpeakChannel, SzThingSpeakWriteAPIKey);
  // check HTTP code
  if (result == 200){
    // feed the ThingSpeak update watchdog
    ThingSpeakWatchdogCounter = 0;
    Serial.println("Channel update successful.");
    }
  else{
    // if unable to update ThingSpeak channel with 3 consecutive attempts, flag InternetAccess as not available
    ThingSpeakWatchdogCounter++;
    if (ThingSpeakWatchdogCounter >= 3) {
      Serial.printf("Error updating ThingSpeak channel, HTTP code %d",result);
      oled_printBuf(true, 0, 0, u8g2_font_t0_14_mr, "ThingSpeak update");
      oled_printBuf(false, 0, 16, u8g2_font_t0_14_mr, "failure");
      u8g2.sendBuffer();     
      FlagInternetAccess = false; // stop trying to update Thingspeak
      }
    }
  WiFiOff();
  }


float battery_SampleVoltage(void) {
  int adcSample = 0;
  for (int inx = 0; inx < 4; inx++) {
    adcSample += analogRead(A0);
    delay(1);
    }
  adcSample /= 4;
  // voltage divider with 120K and 33K to scale 4.2V down to < 1.0V for the ESP8266 ADC
  // actual measurement 0.859V with battery voltage = 3.95V => actual scale up from 
  // resistive divider = 3.95/0.859 = 4.5983
  return (adcSample*4.395f)/1024.0f; 
  }  


void shutDown(void) {  
  Serial.println("Going to sleep");
  oled_printBuf(true, 0, 0, u8g2_font_t0_14_mr, "Going to sleep");
  u8g2.sendBuffer();     
  delay(2000);
  sensor.shutDown();
  digitalWrite(pinOLEDPwr, 1);
  ESP.deepSleep(0); // can only be woken up by reset/power cycle                      
  }

  
// credit https://arduino.stackexchange.com/questions/43376/can-the-wifi-on-esp8266-be-disabled

void WiFiOn() {
  wifi_fpm_do_wakeup();
  wifi_fpm_close();
  //Serial.println("Reconnecting");
  wifi_set_opmode(STATION_MODE);
  wifi_station_connect();
  }

#define FPM_SLEEP_MAX_TIME 0xFFFFFFF

void WiFiOff() {  
  wifi_station_disconnect();
  wifi_set_opmode(NULL_MODE);
  wifi_set_sleep_type(MODEM_SLEEP_T);
  wifi_fpm_open();
  wifi_fpm_do_sleep(FPM_SLEEP_MAX_TIME);  
  }
    
