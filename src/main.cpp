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
#include <Ticker.h>
#include "config.h"
#include "ThingSpeak.h"
#include "algorithm_by_RF.h"
#include "MAX30105.h"
#include "U8g2lib.h"


extern "C" {
    #include "user_interface.h"  // Required for wifi_station_connect() to work
}

// this library works for MAX30102 as well
MAX30105 sensor;
WiFiClient  client;
Ticker ticker;
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE); 

int       CircBufferIndex = 0; // pointer to oldest data in circular buffer
uint32_t  IRCircBuffer[RFA_BUFFER_SIZE]; //circular infrared LED sensor data buffer
uint32_t  RedCircBuffer[RFA_BUFFER_SIZE];  //circular red LED sensor data buffer

int32_t   HeartRate; 
float     SPO2;
int       NumSamples;
bool      FlagUpdate;
// IIR filtered data 
float SPO2_iir = 0.0f;
float HeartRate_iir = 0.0f;

// If pulse not detected in one minute, unit goes to sleep to save power
int SensorWatchdogCounter = 0;

// if unable to publish data to ThingSpeak with 3 consecutive attempts, 
// stop trying and disable wifi to save power
int ThingSpeakWatchdogCounter = 0;

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

void save_config_callback();
void oled_display_data(char* format, ...);
void oled_print_buffer(bool clearBuf, int x, int y, const uint8_t* font, char* format, ...);
float battery_sample_voltage(void);
void shut_down(void);  
void wifi_on();
void wifi_off();  
void update_thingspeak(float spo2, float heartRate, float BatteryVoltage);
void get_sample();


void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("SPO2/Heart-Rate Logger");
  Serial.print("Code compiled on "); Serial.print(__DATE__); Serial.print(" at "); Serial.println(__TIME__);
  pinMode(pinOLEDPwr, OUTPUT);
  // drive pmos switch transistor gate low to switch on power to OLED
  digitalWrite(pinOLEDPwr, 0);
  delay(100);
  
  BatteryVoltage = battery_sample_voltage();
  u8g2.begin();
  u8g2.setFontPosTop();
  //u8g2.setFontDirection(0);
  oled_display_data("SPO2 HR");
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
          oled_print_buffer(true, 0, 0, u8g2_font_t0_14_mr, "No config file");
          oled_print_buffer(false, 0, 16, u8g2_font_t0_14_mr, "found in SPIFFS");
          u8g2.sendBuffer();        
          Serial.println("Failed to load config.json");
          }
      }
    }
  } 
 else {
    oled_print_buffer(true, 0, 0, u8g2_font_t0_14_mr, "Could not mount");
    oled_print_buffer(false, 0, 16, u8g2_font_t0_14_mr, "SPIFFS file system");
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
  oled_print_buffer(true, 0, 0, u8g2_font_t0_14_mr, "Press button for");
  oled_print_buffer(false, 0, 16, u8g2_font_t0_14_mr, "IAP/TS config");
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
    oled_print_buffer(true, 0, 0, u8g2_font_t0_14_mr, "Connect to AP");
    oled_print_buffer(false, 0, 16, u8g2_font_t0_14_mr, "SPO2_HeartRate");
    u8g2.sendBuffer();  
    Serial.println("Starting configuration access point\r\n");
    WiFiManagerParameter customThingspeakChannel("ts_channel", "ThingSpeak Channel", SzThingSpeakChannel, 10);
    WiFiManagerParameter customThingspeakWriteAPIKey("ts_wr_api_key", "ThingSpeak Write API Key", SzThingSpeakWriteAPIKey, 20);
    WiFiManagerParameter customThingspeakUpdateSecs("ts_update_secs", "ThingSpeak Update Seconds", SzThingSpeakUpdateSecs, 5);
    wifiManager.setSaveConfigCallback(save_config_callback);
    wifiManager.addParameter(&customThingspeakChannel);
    wifiManager.addParameter(&customThingspeakWriteAPIKey);
    wifiManager.addParameter(&customThingspeakUpdateSecs);
   // set minimum signal strength of IAP SSIDs to show in the list.
   // reduce this if your IAP is not displayed.
    wifiManager.setMinimumSignalQuality(60);
    // configuration portal times out in 90 seconds
    wifiManager.setConfigPortalTimeout(90);
    // on-demand configuration portal
    if (wifiManager.startConfigPortal("SPO2_HeartRate", "")) {
      // configuration successful, now connected to Internet Access Point in station mode  
      oled_print_buffer(true, 0, 0, u8g2_font_t0_14_mr, "Config OK");
      oled_print_buffer(false, 0, 16, u8g2_font_t0_14_mr, "IAP %s",WiFi.SSID().c_str());
      u8g2.sendBuffer();  
      delay(2000);
      FlagInternetAccess = true;
      Serial.println("Return from Config Portal, connected as Wifi station");
      strcpy(SzThingSpeakChannel, customThingspeakChannel.getValue());
      strcpy(SzThingSpeakWriteAPIKey, customThingspeakWriteAPIKey.getValue());
      strcpy(SzThingSpeakUpdateSecs, customThingspeakUpdateSecs.getValue());
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
        oled_print_buffer(true, 0, 0, u8g2_font_t0_14_mr, "Config portal");
        oled_print_buffer(false, 0, 16, u8g2_font_t0_14_mr, "timed out");
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
      oled_print_buffer(true, 0, 0, u8g2_font_t0_14_mr, "No internet");
      oled_print_buffer(false, 0, 16, u8g2_font_t0_14_mr, "connection");
      u8g2.sendBuffer();
      delay(2000);
      FlagInternetAccess = false;
      wifi_off(); // saves power (current draw down from 75mA down to 25mA)
      Serial.println("Unable to connect as Wifi client, continuing without Internet access");
      }      
    else {
      FlagInternetAccess = true;
      oled_print_buffer(true, 0, 0, u8g2_font_t0_14_mr, "Connected to IAP");
      oled_print_buffer(false, 0, 16, u8g2_font_t0_14_mr, "%s", WiFi.SSID().c_str());
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
    oled_print_buffer(true, 0, 0, u8g2_font_t0_14_mr, "MAX30102 connection");
    oled_print_buffer(false, 0, 16, u8g2_font_t0_14_mr, "problem");
    u8g2.sendBuffer();  
    delay(3000);
    shut_down();
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
  CircBufferIndex = 0;
  if (FlagInternetAccess) {
    Serial.println("Starting loop with InternetAccess");
    ThingSpeak.begin(client); 
    // Wifi radio is turned off to save power, turned on again for updates
    wifi_off();
    }
  else {
      Serial.println("Starting loop with NO InternetAccess");
    }
  // get current time for thingspeak update  
  ThingSpeakTimeMarker = millis();
  FlagUpdate = false;
  ticker.attach(0.02, get_sample); // we actually expect a new sample every 40mS
  }




void loop() {
  float ratio,correl; 
  int8_t  flagSPO2Valid = 0;  
  int8_t  flagHRValid  = 0;  

  if (FlagUpdate == true) {
    FlagUpdate = false;
    rf_heart_rate_and_oxygen_saturation(CircBufferIndex,IRCircBuffer, RFA_BUFFER_SIZE, RedCircBuffer, &SPO2, &flagSPO2Valid, &HeartRate, &flagHRValid, &ratio, &correl);     
    // periodically check the battery voltage  
    BatteryVoltage = battery_sample_voltage();
    if (BatteryVoltage < 3.3f) {
      oled_print_buffer(true, 0, 0, u8g2_font_t0_14_mr, "Low Battery Voltage");
      u8g2.sendBuffer();  
      delay(3000);
      shut_down();
      }

    if (flagHRValid && flagSPO2Valid) {
      SensorWatchdogCounter = 0; // got good data from sensor, feed the watchdog
      // apply damping IIR filter to SPO2 and heart-rate readings
      SPO2_iir = SPO2_iir > 0.1f ? 0.9f * SPO2_iir + 0.1f * SPO2 : SPO2;
      HeartRate_iir = HeartRate_iir > 0.1f ?  0.9f * HeartRate_iir + 0.1f * (float)HeartRate : (float)HeartRate;
      oled_display_data("%2d%% %3d", SPO2_iir >= 99.0f ? 99 : (int)(SPO2_iir+0.5f), (int)(HeartRate_iir+0.5f));
      }
    else {
      SensorWatchdogCounter++;
      if (SensorWatchdogCounter >= SENSOR_TIMEOUT_CYCLES) {
        oled_print_buffer(true, 0, 0, u8g2_font_t0_14_mr, "No SPO2/Pulse data");
        oled_print_buffer(false, 0, 16, u8g2_font_t0_14_mr, "for last minute");
        u8g2.sendBuffer();     
        delay(3000);
        shut_down();
        }
      else {     
       	// blank display if spo2/heart-rate could not be computed from the data
        oled_display_data("        ");
        }
      } 
    Serial.printf("%c SP02_IIR %.2f, Pulse_IIR %.0f\r\n", flagSPO2Valid && flagHRValid ? ' ' : 'x', SPO2_iir, HeartRate_iir);
            
    if (FlagInternetAccess) {
      if ((millis() - ThingSpeakTimeMarker) >  (ThingSpeakUpdateSecs*1000)){ 
        ThingSpeakTimeMarker = millis();
        Serial.printf("SPO2_iir = %.1f, HeartRate_iir = %.0f BatteryVoltage %.2f\r\n", SPO2_iir, HeartRate_iir, BatteryVoltage);
        update_thingspeak(SPO2_iir, HeartRate_iir, BatteryVoltage);
        }
      }
    }        
  }
 
  

// turn on Wifi only for the duration of the ThingSpeak connection 
// current draw with OLED display and WiFi on = ~75mA
// with OLED display on, WiFi off = ~30mA
void update_thingspeak(float spo2, float heartRate, float BatteryVoltage) {
  wifi_on();
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
      oled_print_buffer(true, 0, 0, u8g2_font_t0_14_mr, "ThingSpeak update");
      oled_print_buffer(false, 0, 16, u8g2_font_t0_14_mr, "failure");
      u8g2.sendBuffer();     
      FlagInternetAccess = false; // stop trying to update Thingspeak
      }
    }
  wifi_off();
  }



float battery_sample_voltage(void) {
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


void shut_down(void) {  
  Serial.println("Going to sleep");
  oled_print_buffer(true, 0, 0, u8g2_font_t0_14_mr, "Going to sleep");
  u8g2.sendBuffer();     
  delay(2000);
  sensor.shutDown();
  digitalWrite(pinOLEDPwr, 1);
  ESP.deepSleep(0); // can only be woken up by reset/power cycle                      
  }



// credit https://arduino.stackexchange.com/questions/43376/can-the-wifi-on-esp8266-be-disabled

void wifi_on() {
  wifi_fpm_do_wakeup();
  wifi_fpm_close();
  //Serial.println("Reconnecting");
  wifi_set_opmode(STATION_MODE);
  wifi_station_connect();
  }

#define FPM_SLEEP_MAX_TIME 0xFFFFFFF

void wifi_off() {  
  wifi_station_disconnect();
  wifi_set_opmode(NULL_MODE);
  wifi_set_sleep_type(MODEM_SLEEP_T);
  wifi_fpm_open();
  wifi_fpm_do_sleep(FPM_SLEEP_MAX_TIME);  
  }
    
void save_config_callback () {
  Serial.println("Parameters were modified, need to save config.json");
  FlagSaveConfig = true;
  }

void get_sample(){
  sensor.check();
  if (sensor.available())   {
      RedCircBuffer[CircBufferIndex] = sensor.getFIFORed(); 
      IRCircBuffer[CircBufferIndex] = sensor.getFIFOIR();
      CircBufferIndex++;
      if (CircBufferIndex >= RFA_BUFFER_SIZE) CircBufferIndex = 0;  
      NumSamples++;
      sensor.nextSample();
      // Recompute every second with the last 5 seconds of samples (sliding window)
      // CircBufferIndex points to the oldest sample in the buffer
      if (NumSamples == FS) {
        FlagUpdate = true;
        NumSamples = 0;
        }
      }
  }

// clear screen buffer, use large font to print SPO2 and 
// HeartRate and send to display
void oled_display_data(char* format, ...) {  
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
void oled_print_buffer(bool clearBuf, int x, int y, const uint8_t* font, char* format, ...) {  
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
