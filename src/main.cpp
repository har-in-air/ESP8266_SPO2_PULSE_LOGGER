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
#include <ESP8266WiFi.h>          
#include <ThingSpeak.h>
#include <Ticker.h>
#include "config.h"
#include "algorithm_by_RF.h"
#include "MAX30105.h"
#include "kalmanfilter.h"
#include "oled.h"
#include "config.h"

extern "C" {
    #include "user_interface.h"  // Required for wifi_station_connect() to work
}

//#define USE_KF

// config button pin for on-demand Configuration portal
#define pinCfg         0 
// PMOS transistor-switched power for OLED display
// drive low to turn on
#define pinOLEDPwr    13
// If pulse not detected in one minute, unit goes to sleep to save power
#define SENSOR_TIMEOUT_CYCLES  (60*FS)
// First order IIR damping filter coefficient
#define IIR_COEFF      0.98f

#ifdef USE_KF
#define RED_NOISE_VARIANCE  150.0f
#define IR_NOISE_VARIANCE   150.0f

#define RED_SIGNAL_MEAN     175000.0f
#define IR_SIGNAL_MEAN      215000.0f
#define RED_SIGNAL_VARIANCE 20000.0f
#define IR_SIGNAL_VARIANCE  50000.0f
#endif

MAX30105 sensor;// this library works for MAX30102 as well
WiFiClient  client;
Ticker ticker;
KalmanFilter kfRed;
KalmanFilter kfIR;

int    CircBufferIndex = 0; // pointer to oldest data in circular buffer
float  IRCircBuffer[RFA_BUFFER_SIZE]; //circular infrared LED sensor data buffer
float  RedCircBuffer[RFA_BUFFER_SIZE];  //circular red LED sensor data buffer

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
uint32_t ThingSpeakTimeMarker;// ThingSpeak update interval marker
float BatteryVoltage;

static float battery_sample_voltage(void);
static void shut_down(void);  
static void update_thingspeak(float spo2, float heartRate, float BatteryVoltage);
static void read_sensor_sample();
static void sensor_init();
#ifdef USE_KF
static void measure_mean_variance(float sampleBuf[], int numSamples, float* pMean, float* pVariance);
#endif

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
  pinMode(pinCfg, INPUT);

  // retrieve wifi and thingspeak configuration data from json file in SPIFFS
  load_config_data();

  Serial.printf("ThingSpeak Channel = %s\r\n", SzThingSpeakChannel);
  Serial.printf("ThingSpeak Write API Key = %s\r\n", SzThingSpeakWriteAPIKey);
  Serial.printf("ThingSpeak Update Seconds = %s\r\n", SzThingSpeakUpdateSecs);

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
    wifi_config();
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
  
  sensor_init();

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
  NumSamples = 0;
  CircBufferIndex = 0;
  FlagUpdate = false;
#ifdef USE_KF  
  kfRed.configure(RED_NOISE_VARIANCE, RED_SIGNAL_VARIANCE, RED_SIGNAL_MEAN, 0.0f);  
  kfIR.configure(IR_NOISE_VARIANCE, IR_SIGNAL_VARIANCE, IR_SIGNAL_MEAN, 0.0f);  
#endif
  ticker.attach(0.02, read_sensor_sample); // new sample expected every ~40mS  
  }


#if 0
// Measure the variance of the red and iir signal, needed to optimally configure the kalman filter
// cover the sensor with some dark cloth or wadding, to get the sensor noise variance
// place your finger on the sensor to measure the sensor signal variance
void loop() {
    read_sensor_sample();
    if (FlagUpdate == true) {
      FlagUpdate = false;
      NumSamples++;
      }
  float mean, var;
  if (NumSamples >= RFA_BUFFER_SIZE) {
    kfRed.measureMeanVariance(RedCircBuffer, RFA_BUFFER_SIZE, &mean, &var);
    Serial.printf("Red mean = %f variance = %f\n", mean, var);
    kfIR.measureMeanVariance(IRCircBuffer, RFA_BUFFER_SIZE, &mean, &var);
    Serial.printf("IR mean = %f variance = %f\n\n", mean, var);
    NumSamples = 0;
    delay(1000);  
    }
}
#else

void loop() {
  bool flagSPO2Valid, flagHRValid;

  if (FlagUpdate == true) {
    FlagUpdate = false;
    NumSamples++;
    flagSPO2Valid = false;
    flagHRValid  = false;
    rf_heart_rate_and_oxygen_saturation(CircBufferIndex, IRCircBuffer, RedCircBuffer, RFA_BUFFER_SIZE, &SPO2, &flagSPO2Valid, &HeartRate, &flagHRValid);     

    if ((flagHRValid == false) && (flagSPO2Valid == false)){
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
    else {
      SensorWatchdogCounter = 0; // got good data from sensor, feed the watchdog
      if (flagHRValid == true) {
        // apply damping IIR filter 
        HeartRate_iir = HeartRate_iir > 0.1f ?  (IIR_COEFF * HeartRate_iir) + ((1.0f - IIR_COEFF) * (float)HeartRate) : (float)HeartRate;
        }
      if (flagSPO2Valid == true) {
        // apply damping IIR filter 
        SPO2_iir = SPO2_iir > 0.1f ? (IIR_COEFF * SPO2_iir) + ((1.0f - IIR_COEFF) * SPO2) : SPO2;
        }
      // Update display every 2 seconds 
      if (NumSamples >= (2*FS)) {
        NumSamples = 0;
        // check the battery voltage  
        BatteryVoltage = battery_sample_voltage();
        if (BatteryVoltage < 3.3f) {
          oled_print_buffer(true, 0, 0, u8g2_font_t0_14_mr, "Low Battery Voltage");
          u8g2.sendBuffer();  
          delay(3000);
          shut_down();
          }
        oled_display_data("%2d%% %3d", SPO2_iir >= 99.0f ? 99 : (int)(SPO2_iir+0.5f), (int)(HeartRate_iir+0.5f));
        Serial.printf("SP02_IIR %.2f, Pulse_IIR %.0f\r\n", SPO2_iir, HeartRate_iir);
        }
      }

    if (FlagInternetAccess) {
      if ((millis() - ThingSpeakTimeMarker) >  (ThingSpeakUpdateSecs*1000)){ 
        ThingSpeakTimeMarker = millis();
        Serial.printf("SPO2_iir = %.1f, HeartRate_iir = %.0f BatteryVoltage %.2f\r\n", SPO2_iir, HeartRate_iir, BatteryVoltage);
        update_thingspeak(SPO2_iir, HeartRate_iir, BatteryVoltage);
        }
      }
    } 
  }
 #endif
  

static void sensor_init() {
  if (sensor.begin(Wire, I2C_SPEED_FAST) == false) {
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
  uint8_t ledBrightness = 30; // 0 = off,  255 = 50mA
  uint8_t ledMode = 2; // 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green (MAX30105 only)
  // net sampling rate = sampleRate/sampleAverage = 25
  int sampleRate = 100; // 50, 100, 200, 400, 800, 1000, 1600, 3200
  uint8_t sampleAverage = 4; // 1, 2, 4, 8, 16, 32
  int pulseWidth = 411; // 69, 118, 215, 411
  int adcRange = 2048; // 2048, 4096, 8192, 16384
  
  sensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); 
  sensor.getINT1(); // clear the status registers by reading
  sensor.getINT2();
  }
    

static void read_sensor_sample(){
#ifdef USE_KF  
  float redFiltered, irFiltered, dummy;
#endif  
  sensor.check();
  if (sensor.available())   {
      float redSample = (float)sensor.getFIFORed();
      float irSample = (float)sensor.getFIFOIR();
#ifdef USE_KF      
      kfRed.update(redSample, RED_SIGNAL_VARIANCE, 0.04, &redFiltered, &dummy);
      kfIR.update(irSample, IR_SIGNAL_VARIANCE, 0.04, &irFiltered, &dummy);
      RedCircBuffer[CircBufferIndex] = redFiltered; 
      IRCircBuffer[CircBufferIndex] = irFiltered; 
#else
      RedCircBuffer[CircBufferIndex] = redSample; 
      IRCircBuffer[CircBufferIndex] = irSample; 
#endif      
      CircBufferIndex++;
      if (CircBufferIndex >= RFA_BUFFER_SIZE) CircBufferIndex = 0;  
      sensor.nextSample();
      FlagUpdate = true;
      }
  }

#ifdef USE_KF
static void measure_mean_variance(float sampleBuf[], int numSamples, float* pMean, float* pVariance) {
	float mean = 0.0f;
	float variance = 0.0f;
	for (int inx = 0; inx < numSamples; inx++) {
		mean += sampleBuf[inx];
		}
	mean /= numSamples;
	for (int inx = 0; inx < numSamples; inx++) {
		float d = (sampleBuf[inx] - mean);
		variance += d*d;
		}
	variance /= (numSamples-1);
	*pMean = mean;
	*pVariance = variance;
	return;
}
#endif

// turn on Wifi only for the duration of the ThingSpeak connection 
// current draw with OLED display and WiFi on = ~75mA
// with OLED display on, WiFi off = ~30mA
static void update_thingspeak(float spo2, float heartRate, float BatteryVoltage) {
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


static float battery_sample_voltage(void) {
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


static void shut_down(void) {  
  Serial.println("Going to sleep");
  oled_print_buffer(true, 0, 0, u8g2_font_t0_14_mr, "Going to sleep");
  u8g2.sendBuffer();     
  delay(2000);
  sensor.shutDown();
  digitalWrite(pinOLEDPwr, 1);
  ESP.deepSleep(0); // can only be woken up by reset/power cycle                      
  }



