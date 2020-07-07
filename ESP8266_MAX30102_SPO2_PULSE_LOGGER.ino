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
******************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h> 
#include <WiFiManager.h> 
#include <pgmspace.h>
#include "ThingSpeak.h"
#include "algorithm_by_RF.h"
#include "MAX30105.h"

MAX30105 sensor;

WiFiClient  client;

#define LEDYellow    13 // internet access indicator
#define LEDBlue      12 // sensor data indicator

uint32_t  aun_ir_buffer[RFA_BUFFER_SIZE]; //infrared LED sensor data
uint32_t  aun_red_buffer[RFA_BUFFER_SIZE];  //red LED sensor data
int32_t   n_heart_rate; 
float     n_spo2;
int       numSamples;


void setup() {
  pinMode(LEDYellow, OUTPUT);
  digitalWrite(LEDYellow, 0);
  pinMode(LEDBlue, OUTPUT);
  digitalWrite(LEDBlue, 0);
  
  Serial.begin(230400);
  Serial.println();
  Serial.println("SPO2/Heart-Rate Logger");

  float bv = battery_SampleVoltage();
  // flash the blue led one to five times (5 for fully charged battery, 1 for depleted battery)
  battery_IndicateVoltage(bv, LEDBlue);

  // ESP8266 tx power output 20.5dBm by default
  // we can lower this to reduce power supply noise caused by tx bursts
  WiFi.setOutputPower(12); 

  Serial.println();
  Serial.print("Code compiled on "); Serial.print(__DATE__); Serial.print(" at "); Serial.println(__TIME__);
  //String macID = WiFi.macAddress();
  //Serial.print("Module mac address ; "); Serial.println(macID);

  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(90);
  // Fetches ssid and password from eeprom and tries to connect
  // If it does not connect it starts an access point with the name "SPO2_HeartRate"
  // and goes into a blocking loop for up to 90 seconds, waiting for user configuration
  // of ssid and password. If you do not access the portal page at 192.168.4.1 within 90s, 
  // goes to sleep to save battery power.
  if (!wifiManager.autoConnect("SPO2_HeartRate", "kablooie")) {
    handleFault("Failed to connect to Internet Access Point, and WiFi configuration timed out", LEDYellow);
    }
    
  Serial.println("Connected as Wifi client");
  digitalWrite(LEDYellow, 1);
  delay(1000);
  digitalWrite(LEDYellow, 0);
  
  if (sensor.begin(Wire, I2C_SPEED_FAST) == false) {
    handleFault("MAX30102 not found", LEDBlue);
    }
    
  // ref Maxim AN6409, average dc value of signal should be within 0.25 to 0.75 18-bit range (max value = 262143)
  // You should test this as per the app note depending on application : finger, forehead, earlobe etc. It even
  // depends on skin tone.
  // I found that the optimum combination for my index finger was :
  // ledBrightness=30 and adcRange=2048, to get max dynamic range in the waveform, and a dc level > 100000
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
int ThingSpeakCounter = 0;

// if no valid heartrate/spo2 data found for 2 minutes, go to sleep
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
        // flash the blue LED. This should happen every ST (= 4) seconds if MAX30102 has been configured correctly
        digitalWrite(LEDBlue,1);
        delay(20);
        digitalWrite(LEDBlue,0);
  
        float battery_voltage = battery_SampleVoltage();
        if (battery_voltage < 3.4f) {
          handleFault("Battery discharged", LEDBlue);
          }
      
        // average last five good readings and update Thingspeak
        // = 20s interval if all readings good
        if (ch_hr_valid && ch_spo2_valid) {
          SensorWatchdogCounter = 0; // feed the watchdog
          spo2_accum += n_spo2;
          heart_rate_accum += n_heart_rate;
          ThingSpeakCounter++;
          Serial.printf("ThingSpeakCounter = %d spo2_accum = %.1f, heartRateAccum = %d\r\n",ThingSpeakCounter, spo2_accum, heart_rate_accum);
          if (ThingSpeakCounter >= 5){
            spo2_accum /= ThingSpeakCounter;
            heart_rate_accum /= ThingSpeakCounter;
            Serial.printf("Avg : ThingSpeakCounter = %d spo2_accum = %.1f, heartRateAccum = %d batteryVoltage %.2f\r\n",ThingSpeakCounter, spo2_accum, heart_rate_accum, battery_voltage);
            updateThingSpeak(spo2_accum, heart_rate_accum, battery_voltage);
            ThingSpeakCounter = 0;
            spo2_accum = 0.0;
            heart_rate_accum = 0;            
            }
          }
        else {
          SensorWatchdogCounter++;
          if (SensorWatchdogCounter >= 30) {
            handleFault("No valid spo2/pulse readings for the past 2 minutes", LEDBlue);
            }
          }        
        }
    }
  
  }
  

// use your Thingspeak channel number and  write api key
unsigned long myChannelNumber = 1234567;
const char * myWriteAPIKey = "ABCDEFGHIJKLMN";


void updateThingSpeak(float spo2, int heartRate, float batteryVoltage) {
  ThingSpeak.setField(1, spo2);
  ThingSpeak.setField(2, heartRate);
  ThingSpeak.setField(3, batteryVoltage);
  int result = ThingSpeak.writeFields( myChannelNumber, myWriteAPIKey);
  if (result == 200){
    ThingSpeakWatchdogCounter = 0;
    Serial.println("Channel update successful.");
    // flash yellow LED to show successful update
    digitalWrite(LEDYellow, 1);
    delay(20);
    digitalWrite(LEDYellow, 0);
    }
  else{
    ThingSpeakWatchdogCounter++;
    if (ThingSpeakWatchdogCounter >= 3) {
      char szMsg[60];
      sprintf(szMsg, "Error updating ThingSpeak channel, HTTP code %d",result);
      handleFault(szMsg, LEDYellow);
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
  return (adcSample*4.5983f)/1024.0f; 
  }  



void battery_IndicateVoltage(float bv, int pinLED) {
    int numFlashes;
    if (bv >= 3.95f) numFlashes = 5;
    else
    if (bv >= 3.8f) numFlashes = 4;
    else
    if (bv >= 3.7f) numFlashes = 3;
    else
    if (bv >= 3.6f) numFlashes = 2;
    else  numFlashes = 1;
    while (numFlashes--) {
        digitalWrite(pinLED, 1);
        delay(50);
        digitalWrite(pinLED, 0);
        delay(300);
        }
    } 


void handleFault(char* szMsg, int pinLED) {
  Serial.println(szMsg);
  for (int inx = 0; inx < 50; inx++){
    digitalWrite(pinLED, !digitalRead(pinLED));
    delay(100);
    }
  Serial.println("Going to sleep");
  ESP.deepSleep(0); // can only be woken up by reset/power cycle                      
  }
    
