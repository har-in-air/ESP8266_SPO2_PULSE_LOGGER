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
* 3. WiFiManager SSID and password configuration magic from 
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
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
//#include <DNSServer.h>
#include <ESP8266WebServer.h>
//#include <WiFiClient.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include <pgmspace.h>
#include "ThingSpeak.h"
#include "algorithm_by_RF.h"
#include "MAX30105.h"

// this library works for MAX30102 as well
MAX30105 sensor;
WiFiClient  client;

// config button pin
#define pinCfg         0 

// LED pins
#define pinRED        14 
#define pinGRN        12 
#define pinBLU        13 

typedef enum COLOUR_T {
  MAGENTA, BLUE, TURQUOISE, GREEN, YELLOW, RED, WHITE
  } COLOUR;

#define BLINK_FAST  200
#define BLINK_SLOW  500

// 4 second sample cycle, if pulse not detected in one minute, put unit 
// to sleep to save power
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
// Thingspeak parameters, and click on Save Settings.

char SzThingSpeakChannel[10] = {0};
char SzThingSpeakWriteAPIKey[20] = {0};

unsigned long ThingSpeakChannel = 0;

// static IP address doesn't work with some access points e.g.
// cellphone used as hotspot

#ifdef USE_STATIC_IP_ADDRESS  
char SzStaticIPAddr[16] = {0};
char SzStaticGateway[16] = {0};
char SzStaticSubnet[16] = {0};
char SzDNS[16]  = "8.8.8.8";
#endif

//String SzSSID;
//String SzPassword;

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
#ifdef USE_STATIC_IP_ADDRESS  
          strcpy(SzStaticIPAddr, json["ipaddr"]);
          strcpy(SzStaticGateway, json["gateway"]);
          strcpy(SzStaticSubnet, json["subnet"]);
#endif
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
#ifdef USE_STATIC_IP_ADDRESS  
  Serial.printf("Static IP Address = %s\r\n", SzStaticIPAddr);
  Serial.printf("Static Gateway = %s\r\n", SzStaticGateway);
  Serial.printf("Static Subnet = %s\r\n", SzStaticSubnet);
#endif    
  pinMode(pinCfg, INPUT);
  
  // If you want to change Internet Access Point SSID/Password, or ThingSpeak credentials : 
  // When you see the battery indication RED led blinking,
  // press config button and keep it pressed until you see the blue led turn on

  float batVoltage = battery_SampleVoltage();
  // blink the RED led slowly one to five times (5 for fully charged battery, 1 for depleted battery)
  battery_IndicateVoltage(batVoltage, MAGENTA);
  delay(500);

  // If config button is pressed, or the Thingspeak credential strings are empty :
  // Starts an access point with the name "SPO2_HeartRate"
  // and goes into a blocking loop for up to 90 seconds, waiting for user configuration
  // of IAP ssid and password. If you do not access the unit's portal page at http://192.168.4.1 within 90s, 
  // it indicates failure by blinking the blue LED rapidly, and then goes to sleep to 
  // save battery power. Turn off and turn on the unit, and try again.
  int flagConfigRequired = (digitalRead(pinCfg) == 0) | (strlen(SzThingSpeakChannel) == 0)  | (strlen(SzThingSpeakWriteAPIKey) == 0)
#ifdef USE_STATIC_IP_ADDRESS    
  |  (strlen(SzStaticIPAddr) == 0) | (strlen(SzStaticGateway) == 0) | (strlen(SzStaticSubnet) == 0)
#endif  
  ;

  if ( flagConfigRequired ) {
    LED_On(YELLOW);
    WiFiManager wifiManager;
    Serial.println("** CONFIGURATION REQUIRED  **\r\n");
    WiFiManagerParameter custom_thingspeak_channel("ts_channel", "TS Channel", SzThingSpeakChannel, 10);
    WiFiManagerParameter custom_thingspeak_write_api_key("ts_wr_api_key", "TS Write API Key", SzThingSpeakWriteAPIKey, 20);
#ifdef USE_STATIC_IP_ADDRESS    
    WiFiManagerParameter custom_static_ipaddr("ipaddr", "Static IPAddr", SzStaticIPAddr, 16);
    WiFiManagerParameter custom_static_gateway("gateway", "Static Gateway", SzStaticGateway, 16);
    WiFiManagerParameter custom_static_subnet("subnet", "Static Subnet", SzStaticSubnet, 16);
#endif
    wifiManager.setSaveConfigCallback(saveConfigCallback);
#ifdef USE_STATIC_IP_ADDRESS        
    IPAddress ipaddr, gateway, subnet;
    ipaddr.fromString(SzStaticIPAddr);
    gateway.fromString(SzStaticGateway);
    subnet.fromString(SzStaticSubnet);
    wifiManager.setSTAStaticIPConfig(ipaddr, gateway, subnet);  
#endif    
    wifiManager.addParameter(&custom_thingspeak_channel);
    wifiManager.addParameter(&custom_thingspeak_write_api_key);
#ifdef USE_STATIC_IP_ADDRESS    
    wifiManager.addParameter(&custom_static_ipaddr);
    wifiManager.addParameter(&custom_static_gateway);
    wifiManager.addParameter(&custom_static_subnet);
#endif    
   // set minimum signal strength of IAP SSIDs to show in the list.
   // reduce this if your IAP is not displayed.
    wifiManager.setMinimumSignalQuality(60);
    wifiManager.setConfigPortalTimeout(90);
    if (!wifiManager.startConfigPortal("SPO2_HeartRate", "")) {
      handleFault("Failed to connect to Internet Access Point, and WiFi configuration timed out", YELLOW, BLINK_FAST);
      }
    // configuration successful, now connected to Internet Access Point in station mode  
    //SzSSID = wifiManager.getSSID();
    //SzPassword = wifiManager.getPassword();
    //Serial.printf("SSID = %s\r\n", SzSSID.c_str()); 
    //Serial.printf("Password = %s\r\n", SzPassword.c_str()); 
    Serial.println("Return from Config Portal, connected as Wifi station");
    strcpy(SzThingSpeakChannel, custom_thingspeak_channel.getValue());
    strcpy(SzThingSpeakWriteAPIKey, custom_thingspeak_write_api_key.getValue());
#ifdef USE_STATIC_IP_ADDRESS    
    strcpy(SzStaticIPAddr, custom_static_ipaddr.getValue());
    strcpy(SzStaticGateway, custom_static_gateway.getValue());
    strcpy(SzStaticSubnet, custom_static_subnet.getValue());
#endif
    if (FlagSaveConfig) {
      Serial.println("Changes made, saving config.json");
      DynamicJsonBuffer jsonBuffer;
      JsonObject& json = jsonBuffer.createObject();
      json["ts_channel"] = SzThingSpeakChannel;
      json["ts_wr_api_key"] = SzThingSpeakWriteAPIKey;
#ifdef USE_STATIC_IP_ADDRESS    
      json["ipaddr"] = SzStaticIPAddr;
      json["gateway"] = SzStaticGateway;
      json["subnet"] = SzStaticSubnet;
#endif
      File configFile = SPIFFS.open("/config.json", "w");
      if (!configFile) {
        Serial.println("Failed to open config.json file for writing");
        }
      json.prettyPrintTo(Serial);
      json.printTo(configFile);
      configFile.close();
      }    
    LED_Off();
    }
 else {
    Serial.println("Connecting in station mode with Internet Access Point SSID and password retrieved from flash");
#ifdef USE_STATIC_IP_ADDRESS    
    IPAddress ipaddr, gateway, subnet, dns;
    ipaddr.fromString(SzStaticIPAddr);
    gateway.fromString(SzStaticGateway);
    subnet.fromString(SzStaticSubnet);
    dns.fromString(SzDNS);
    const char* deviceName = "ESP8266_SPO2_PULSE_LOG.001";
    WiFi.hostname(deviceName); // DHCP Hostname
    WiFi.config(ipaddr, subnet, gateway, dns);
#endif
    WiFi.mode(WIFI_STA);
    WiFi.begin(); // use SSID and password retrieved from flash
    int counter = 0;
    while ((counter < 20) && (WiFi.status() != WL_CONNECTED)) {
      delay(500);
      counter++;
      Serial.print(".");
      }   
    if (counter >= 20) {
      handleFault("Unable to connect to Internet Access Point", YELLOW, BLINK_SLOW);
      }      
    Serial.println("Connected as Wifi client");
    }

  // ESP8266 tx power output 20.5dBm by default
  // we can lower this to reduce power supply noise caused by tx bursts
  WiFi.setOutputPower(12); 
  char* ptr;
  ThingSpeakChannel = strtoul(SzThingSpeakChannel, &ptr, 10);
  Serial.printf("ThingSpeakChannel number = %lu\r\n", ThingSpeakChannel);
  
  if (sensor.begin(Wire, I2C_SPEED_FAST) == false) {
    handleFault("MAX30102 not found", TURQUOISE, BLINK_SLOW);
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

  ThingSpeak.begin(client); 
  }


// minimum ThingSpeak update interval is 15 seconds, we will update once every 20 seconds
int SampleCycleCounter = 0;

// if no valid heartrate/spo2 data found for 1 minute, go to sleep
int SensorWatchdogCounter = 0;

// if unable to publish data to ThingSpeak with 3 attempts, go to sleep
int ThingSpeakWatchdogCounter = 0;

// accumulators for averaging measurements
float spo2_accum = 0.0;
int heart_rate_accum = 0;

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
        Serial.printf("SP02 ");
        if (ch_spo2_valid) Serial.print(n_spo2); else Serial.print("x");
        Serial.print(", Pulse ");
        if (ch_hr_valid) Serial.print(n_heart_rate); else Serial.print("x");
        Serial.println();
        numSamples = 0;
  
        float battery_voltage = battery_SampleVoltage();
        if (battery_voltage < 3.3f) {
          handleFault("Battery discharged", MAGENTA, BLINK_FAST);
          }
      
        if (ch_hr_valid && ch_spo2_valid) {
          // flash the LED for a good measurement. This should happen every ST (= 4) seconds if MAX30102 has been configured correctly
		      // LED colour indicates the pulse rate : MAGENTA < 65bpm , BLUE 65-70bpm, TURQUOISE 70-75bpm, GREEN 75-80bpm, YELLOW 80-85bpm,  RED > 85bpm
          indicateHeartRate(n_heart_rate);
          SensorWatchdogCounter = 0; // feed the watchdog
          spo2_accum += n_spo2;
          heart_rate_accum += n_heart_rate;
          SampleCycleCounter++;
          Serial.printf("SampleCycleCounter = %d spo2_accum = %.1f, heartRateAccum = %d\r\n",SampleCycleCounter, spo2_accum, heart_rate_accum);
          // average last five good readings and update Thingspeak (so 20s interval if all readings were good)
          if (SampleCycleCounter >= 5){
            spo2_accum /= SampleCycleCounter;
            heart_rate_accum /= SampleCycleCounter;
            Serial.printf("Avg : SampleCycleCounter = %d spo2_accum = %.1f, heartRateAccum = %d batteryVoltage %.2f\r\n",SampleCycleCounter, spo2_accum, heart_rate_accum, battery_voltage);
            updateThingSpeak(spo2_accum, heart_rate_accum, battery_voltage);
            SampleCycleCounter = 0;
            spo2_accum = 0.0;
            heart_rate_accum = 0;            
            }
          }
        else {
          SensorWatchdogCounter++;
          if (SensorWatchdogCounter >= SENSOR_TIMEOUT_CYCLES) {
            handleFault("No valid spo2/pulse readings for the past minute", TURQUOISE, BLINK_FAST);
            }
          }        
        }
    }
  
  }
  

void updateThingSpeak(float spo2, int heartRate, float batteryVoltage) {
  ThingSpeak.setField(1, spo2);
  ThingSpeak.setField(2, heartRate);
  ThingSpeak.setField(3, batteryVoltage);
  int result = ThingSpeak.writeFields( ThingSpeakChannel, SzThingSpeakWriteAPIKey);
  if (result == 200){
    ThingSpeakWatchdogCounter = 0;
    Serial.println("Channel update successful.");
    // flash white LED to show successful update
    flashLED(WHITE);
    }
  else{
    ThingSpeakWatchdogCounter++;
    if (ThingSpeakWatchdogCounter >= 3) {
      char szMsg[60];
      sprintf(szMsg, "Error updating ThingSpeak channel, HTTP code %d",result);
      handleFault(szMsg, WHITE, BLINK_FAST);
      }
  }  
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

void flashLED(COLOUR colour) {
  LED_On(colour);
  delay(20);
  LED_Off();
  }

void LED_On(COLOUR colour) {
	switch (colour) {
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
	if (pulseRate < 70) {
			flashLED(BLUE);
			}
	else 
	if (pulseRate < 85) {
			flashLED(GREEN);
			}
	else {
		flashLED(RED);
		}
	}
	
void handleFault(char* szMsg, COLOUR colour, int blinkDelayMs) {
  Serial.println(szMsg);
  for (int inx = 0; inx < 50; inx++){
    LED_On(colour);
    delay(blinkDelayMs);
    LED_Off();
    delay(blinkDelayMs);
    }
  Serial.println("Going to sleep");
  sensor.shutDown();
  ESP.deepSleep(0); // can only be woken up by reset/power cycle                      
  }
    
