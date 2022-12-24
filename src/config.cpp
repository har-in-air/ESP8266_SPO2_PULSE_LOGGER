#include <Arduino.h>
#include <FS.h>                  
#include <ESP8266WiFi.h>          
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          
#include "ArduinoJson-v5.13.5.h"         
#include "config.h"
#include "oled.h"

bool FlagInternetAccess = false;
bool FlagSaveConfig = false;

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

void save_config_callback();

void load_config_data() { 
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
}


void wifi_config() {
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


    
void save_config_callback() {
  Serial.println("Parameters were modified, need to save config.json");
  FlagSaveConfig = true;
  }