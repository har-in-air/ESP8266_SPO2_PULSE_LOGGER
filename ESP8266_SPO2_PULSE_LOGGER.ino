/*****************************************************************
* Heart-rate and SPO2 logger using the MAX30102. Logs data once
* a minute to thingspeak.com using your private channel # and API key.
* 
* Credits 
* 1. sensor initialization and readout code from Sparkfun 
* https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library
*  
*  2. spo2 & pulse rate analysis from 
* https://github.com/aromring/MAX30102_by_RF  
* (algorithm by  Robert Fraczkiewicz)
* I tweaked this to use 50Hz sample rate
* 
* 3. WiFiManager SSID/password and ThingSpeak configuration magic from 
* https://github.com/tzapu/WiFiManager
* 
* 4. Arduino Json 5 library
* https://github.com/bblanchon/ArduinoJson/tree/5.x
* 
* Note : if making changes to the stored parameters, or SPIFFS configuration, 
* make sure you choose the 'erase all flash' option, then flash the app again 
* with normal 'erase sketch only' option.
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

extern "C" {
    #include "user_interface.h"  // Required for wifi_station_connect() to work
}

// this library works for MAX30102 as well
MAX30105 sensor;
WiFiClient  client;

// config button pin for on-demand Configuration portal
#define pinCfg         0 

// LED pins
#define pinRED        14 
#define pinGRN        12 
#define pinBLU        13 

typedef enum COLOUR_T {
  MAGENTA, BLUE, TURQUOISE, GREEN, YELLOW, RED, WHITE
  } COLOUR;

COLOUR Colour;

#define BLINK_FAST  200
#define BLINK_SLOW  500

// If pulse not detected in one minute, unit goes to sleep to save power
// (each cycle is 4 seconds of sampling)
#define SENSOR_TIMEOUT_CYCLES 15


// RGB led with common anode, pins need to be grounded to turn on
#define LED_ON(pinLED)  {digitalWrite(pinLED, 0);}
#define LED_OFF(pinLED) {digitalWrite(pinLED, 1);}

uint32_t  aun_ir_buffer[RFA_BUFFER_SIZE]; //infrared LED sensor data
uint32_t  aun_red_buffer[RFA_BUFFER_SIZE];  //red LED sensor data
int32_t   n_heart_rate; 
float     n_spo2;
int       numSamples;

// These parameters are retrieved from the SPIFFS JSON file
// If not found, the portal configuration is automatically started
// The yellow LED will come on, you have 90s to connect to the 
// WiFi access point with SSID 'SPO2_HeartRate', and then open the portal
// page at http://192.168.4.1 
// Specify the Internet access point and password, and the 
// Thingspeak parameters, and click on Save.

char SzThingSpeakChannel[10] = {0};
char SzThingSpeakWriteAPIKey[20] = {0};
char SzThingSpeakUpdateSecs[5] = {0};

unsigned long ThingSpeakChannel = 0;
unsigned long ThingSpeakUpdateSecs;

bool FlagInternetAccess = false;
bool FlagSaveConfig = false;

void saveConfigCallback () {
  Serial.println("Parameters were modified, need to save config.json");
  FlagSaveConfig = true;
  }
  
void setup() {
  Serial.begin(115200);
  pinMode(pinRED, OUTPUT);
  pinMode(pinGRN, OUTPUT);
  pinMode(pinBLU, OUTPUT);
  LED_Off();
  Serial.println();
  Serial.println("SPO2/Heart-Rate Logger");
  Serial.print("Code compiled on "); Serial.print(__DATE__); Serial.print(" at "); Serial.println(__TIME__);
    
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
          Serial.println("Failed to load config.json");
          }
      }
    }
  } else {
    Serial.println("Failed to mount FS");
    }

  Serial.printf("ThingSpeak Channel = %s\r\n", SzThingSpeakChannel);
  Serial.printf("ThingSpeak Write API Key = %s\r\n", SzThingSpeakWriteAPIKey);
  Serial.printf("ThingSpeak Update Seconds = %s\r\n", SzThingSpeakUpdateSecs);
  pinMode(pinCfg, INPUT);
  
  // If you want to change Internet Access Point SSID/Password, or ThingSpeak credentials : 
  // When you see the battery indication RED led blinking,
  // press config button and keep it pressed until you see the blue led turn on

  float batVoltage = battery_SampleVoltage();
  // blink the RED led slowly one to five times (5 for fully charged battery, 1 for depleted battery)
  battery_IndicateVoltage(batVoltage, MAGENTA);
  delay(500);

  // If config button is pressed, or the Thingspeak credentials have not been initialized :
  // Starts an access point with the name "SPO2_HeartRate"
  // and goes into a blocking loop for up to 90 seconds, waiting for user to access the portal.
  // If you do not access the unit's portal page at http://192.168.4.1 within 90s, 
  // the unit will disable internet access and continue on to sensor initialization.
  int flagConfigRequired = (digitalRead(pinCfg) == 0) | (strlen(SzThingSpeakChannel) == 0)  | (strlen(SzThingSpeakWriteAPIKey) == 0)  ;

  if ( flagConfigRequired ) {
    LED_On(YELLOW);
    WiFiManager wifiManager;
    Serial.println("** CONFIGURATION REQUIRED  **\r\n");
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
    wifiManager.setConfigPortalTimeout(90);
    // on demand configuration portal
    if (wifiManager.startConfigPortal("SPO2_HeartRate", "")) {
      // configuration successful, now connected to Internet Access Point in station mode  
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
        }
    LED_Off();
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
      indicateFault("Unable to connect to Internet Access Point", YELLOW, BLINK_FAST);
      FlagInternetAccess = false;
      WiFiOff(); // saves power (current draw down from 75mA down to 25mA)
      Serial.println("Unable to connect as Wifi client, continuing without Internet access");
      }      
    else {
      FlagInternetAccess = true;
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
  
  if (sensor.begin(Wire, I2C_SPEED_FAST) == false) {
    indicateFault("MAX30102 not found", TURQUOISE, BLINK_SLOW);
    shutDown();
    }
    
  // ref Maxim AN6409, average dc value of signal should be within 0.25 to 0.75 18-bit range (max value = 262143)
  // You should test this as per the app note depending on application : finger, forehead, earlobe etc. It even
  // depends on skin tone.
  // I found that the optimum combination for my index finger was :
  // ledBrightness=30 and adcRange=2048, to get max dynamic range in the waveform, and a dc level > 100,000
  byte ledBrightness = 30; // 0 = off,  255 = 50mA
  byte sampleAverage = 4; // 1, 2, 4, 8, 16, 32
  byte ledMode = 2; // 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green (MAX30105 only)
  int sampleRate = 200; // 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; // 69, 118, 215, 411
  int adcRange = 2048; // 2048, 4096, 8192, 16384
  
  sensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); 
  sensor.getINT1(); // clear the status registers by reading
  sensor.getINT2();
  numSamples = 0;
  if (FlagInternetAccess) {
    Serial.println("Starting loop with InternetAccess");
    ThingSpeak.begin(client); 
    // Wifi radio is turned off to save power, turned on again for updates
    WiFiOff();
    }
  else {
      Serial.println("Starting loop with NO InternetAccess");
    }
  }


// counts from zero up to ThingSpeakUpdateSecs/4, then updates the ThingSpeak IOT website channel
unsigned long SampleCycleCounter = 0;

// if no valid heartrate/spo2 data found for 1 minute, go to sleep
int SensorWatchdogCounter = 0;

// if unable to publish data to ThingSpeak with 3 attempts, go to sleep
int ThingSpeakWatchdogCounter = 0;

// IIR filtered data 
float SPO2_iir = 0.0f;
float HeartRate_iir = 0.0f;

// the min/max range is from the RF algorithm
#define VALID_HEARTRATE_RANGE(h) (h > 40.0f && h < 150.0f ? 1 : 0)

void loop() {
  float ratio,correl; 
  int8_t  ch_spo2_valid = 0;  
  int8_t  ch_hr_valid  = 0;  

  sensor.check();
  while (sensor.available())   {
      aun_red_buffer[numSamples] = sensor.getFIFORed(); 
      aun_ir_buffer[numSamples] = sensor.getFIFOIR();
      numSamples++;
      sensor.nextSample(); 
      if (numSamples == RFA_BUFFER_SIZE) {
        // calculate heart rate and SpO2 after RFA_BUFFER_SIZE samples (ST seconds of samples) using Robert's method
        rf_heart_rate_and_oxygen_saturation(aun_ir_buffer, RFA_BUFFER_SIZE, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid, &ratio, &correl);     
        numSamples = 0;
  
        float batteryVoltage = battery_SampleVoltage();
        if (batteryVoltage < 3.3f) {
          indicateFault("Battery discharged", MAGENTA, BLINK_FAST);
          shutDown();
          }

        if (ch_hr_valid && ch_spo2_valid) {
          // Good measurement, LED colour indicates the pulse rate 
          SensorWatchdogCounter = 0; // got good data from sensor, feed the watchdog
          SPO2_iir = 0.7f * SPO2_iir + 0.3f * n_spo2;
          HeartRate_iir = HeartRate_iir > 0.1f ?  0.7f * HeartRate_iir + 0.3f * (float)n_heart_rate : (float)n_heart_rate;
          indicateHeartRate((int)(HeartRate_iir+0.5f));
          }
        else {
          // unable to compute spo2 or heartrate data from sensor data, turn off led for this cycle
          LED_Off();
          SensorWatchdogCounter++;
          if (SensorWatchdogCounter >= SENSOR_TIMEOUT_CYCLES) {
            indicateFault("No valid spo2/pulse readings for the past minute", TURQUOISE, BLINK_FAST);
            shutDown();
            }     
          } 
        Serial.printf("SP02 %.2f, Pulse %d, Pulse IIR %d\r\n", ch_spo2_valid ? n_spo2 : 0.0f, ch_hr_valid ? n_heart_rate : 0, (int)(HeartRate_iir+0.5f));
            
        if (FlagInternetAccess) {
            SampleCycleCounter++;
            // each cycle takes 4 seconds
            if (SampleCycleCounter >= ThingSpeakUpdateSecs/4){
              SampleCycleCounter = 0;
              if (VALID_HEARTRATE_RANGE(HeartRate_iir)){
                Serial.printf("SampleCycleCounter = %d SPO2_iir = %.1f, HeartRate_iir = %.0f BatteryVoltage %.2f\r\n",SampleCycleCounter, SPO2_iir, HeartRate_iir, batteryVoltage);
                updateThingSpeak(SPO2_iir, (int)(HeartRate_iir+0.5f), batteryVoltage);
                }
              }
            }
          }        
        }
      }
 
  

// turn on Wifi only for the duration of the ThingSpeak connection 
// current draw with WiFi on = ~70mA avg
// rest of the time, ~22mA avg with LED on
void updateThingSpeak(float spo2, int heartRate, float batteryVoltage) {
  WiFiOn();
  LED_Off();
  ThingSpeak.setField(1, spo2);
  ThingSpeak.setField(2, heartRate);
  ThingSpeak.setField(3, batteryVoltage);
  int result = ThingSpeak.writeFields( ThingSpeakChannel, SzThingSpeakWriteAPIKey);
  if (result == 200){
    // feed the watchdog
    ThingSpeakWatchdogCounter = 0;
    Serial.println("Channel update successful.");
    }
  else{
    // if unable to update ThingSpeak channel with 3 consecutive attempts, flag InternetAccess as not available
    ThingSpeakWatchdogCounter++;
    if (ThingSpeakWatchdogCounter >= 3) {
      char szMsg[60];
      sprintf(szMsg, "Error updating ThingSpeak channel, HTTP code %d",result);
      FlagInternetAccess = false; // stop trying to update Thingspeak
      }
    }
  // finished update, turn led back on with existing colour     
  LED_On(Colour);
  // turn off Wifi radio to save power
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



void battery_IndicateVoltage(float voltage, COLOUR colour) {
    int numFlashes;
    if (voltage >= 3.95f) numFlashes = 5;
    else
    if (voltage >= 3.8f) numFlashes = 4;
    else
    if (voltage >= 3.7f) numFlashes = 3;
    else
    if (voltage >= 3.6f) numFlashes = 2;
    else  numFlashes = 1;
    while (numFlashes--) {
        LED_On(colour);
        delay(300);
        LED_Off();
        delay(300);
        }
    } 

void flashLED(COLOUR c) {
  LED_Off();
  LED_On(c);
  delay(20);
  LED_Off();
  }

void LED_On(COLOUR c) {
  Colour = c;
  LED_Off();
	switch (c) {
		case MAGENTA :
		LED_ON(pinBLU);
		LED_ON(pinRED);
		break;
		case BLUE :
		LED_ON(pinBLU);
		break;
		case TURQUOISE :
		LED_ON(pinBLU);
		LED_ON(pinGRN);
		break;
		case GREEN:
		LED_ON(pinGRN);
		break;
		case YELLOW:
		LED_ON(pinRED);
		LED_ON(pinGRN);
		break;
		case RED :
		LED_ON(pinRED);
		break;
		case WHITE :
		LED_ON(pinRED);
		LED_ON(pinGRN);
		LED_ON(pinBLU);
		default:
		break;
		}
	}
 

void LED_Off(void) {
	LED_OFF(pinBLU);
	LED_OFF(pinRED);
	LED_OFF(pinGRN);
	}

    
void indicateHeartRate(int pulseRate) {
	if (pulseRate < 65) {
			LED_On(BLUE);     // < 65 bpm
			}
  else 
  if (pulseRate < 70) {
      LED_On(TURQUOISE); // 65 - 70 bpm
      }
	else 
	if (pulseRate < 75) {  // 70 - 75 bpm
			LED_On(GREEN);
			}
  else 
  if (pulseRate < 80) { // 75 - 80 bpm
      LED_On(YELLOW);
      }
  else 
  if (pulseRate < 85) { // 80 - 85 bpm
      LED_On(RED);
      }
  else
  if (pulseRate < 90) { // 85 - 90 bpm
      LED_On(MAGENTA);
      }
	else {
		LED_On(WHITE);  // > 90 bpm
		}
	}
	
void indicateFault(char* szMsg, COLOUR colour, int blinkDelayMs) {
  Serial.println(szMsg);
  for (int inx = 0; inx < 30; inx++){
    LED_On(colour);
    delay(blinkDelayMs);
    LED_Off();
    delay(blinkDelayMs);
    }
  }

void shutDown(void) {  
  Serial.println("Going to sleep");
  sensor.shutDown();
  LED_Off();
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
    
