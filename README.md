# ESP8266_MAX30102_SPO2_PULSE_LOGGER

The ESP8266 collects raw sensor data from a MAX30102 
sensor, analyzes it and computes SP02 and heart-rate (bpm) readings, every 4 seconds. 
The last 5 good readings are averaged and published (along with the battery voltage) to your 
channel on the IOT website Thingspeak.

## Development Environment

* Protoype board with ESP8285 (ESP8266 with built-in 1MB flash), MCP73831 Lipoly charger, 500mAh Lipoly battery.
* Home-brew MAX30102 breakout board. Modules are available on AliExpress.
* Arduino 1.8.12 on Ubuntu 20.04 amdx64

## Prototype

<img src="prototype_hardware.jpg" />


## Credits

* MAX30102 sensor initialization and FIFO readout code from 
[Sparkfun](https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library)

* SPO2 & pulse rate measurement code by Robert Fraczkiewicz from 
[aromring's repository](https://github.com/aromring/MAX30102_by_RF). I tweaked RF's implementation to use 50Hz sample rate. 

* WiFiManager SSID and password configuration magic by [tzapu](https://github.com/tzapu/WiFiManager)

## Usage

* On power up, the ESP8285 connects to an existing Internet access point. If the SSID and password 
stored in EEPROM do not work (first time programming, or SSID/password have changed), it acts as a 
WiFi access point with SSID "SPO2_HeartRate". Connect to this access point within 90 seconds,
and open the webpage 192.168.4.1 to access the Wifi configuration page where you enter the 
Internet access point SSID and password. Then reset or cycle power to the module.
* Wiith normal sensor operation, the blue LED will flash every 4-second measurement cycle
* The yellow LED will flash every time an update to Thingspeak channel has been published. The updates are averaged values of the last 5 good measurements. This should happen every ~20s if all measurement cycles produced good data. 

<img src="screenshot.png"/>

## Fault handling

In case of error conditions, the unit will indicate the fault with a rapidly blinking LED for 5 seconds,
and then go to sleep, to save battery power. You will have to switch the unit off and on again to recover.

* Unable to connect to an existing Internet access point, and the Wifi configuration portal timed out (YELLOW LED)
* Unable to publish data to ThingSpeak for the last 3 attempts (YELLOW LED)
* Battery voltage is too low (BLUE LED)   
* Unable to connect to or configure the MAX30102 sensor (BLUE LED)
* Unable to detect valid spo2/pulse readings for 2 minutes (BLUE LED)




