# ESP8266_MAX30102_SPO2_PULSE_LOGGER

The ESP8266 collects raw sensor data from a MAX30102 
sensor, analyzes it and computes SP02 and heart-rate (bpm) readings, every 4 seconds. 
The last 5 good readings are averaged and transmitted to the IOT website Thingspeak
along with the battery voltage.

## Development Environment

* Protoype board with ESP8285 (ESP8266 with built-in 1MB flash), MCP73831 Lipoly charger, 500mAh Lipoly battery.
* home-brew MAX30102 breakout board (MAX30102 modules are available on AliExpress)
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
* The yellow LED will flash every time an update to Thingspeak channel has been published. This should
happen every ~20s if all measurement cycles produced good data. The updates are averaged values of the last
5 good measurements.

<img src="screenshot.png"/>

## Saving power
In case of error conditions, the module will indicate internet access issues with a rapidly blinking yellow
LED, and sensor reading issues with a rapidly blinking blue LED, and then go to sleep. This is to 
save battery power. You will have to switch off and on the module again for normal usage.

* If unable to connect to an existing Internet access point, and the Wifi configuration portal times out,
the module will flash the yellow LED rapidly for 5 seconds and then go to sleep.
* If unable to configure the MAX30102 sensor, the blue LED will blink rapidly for 5 seconds and then go to sleep.
* If unable to detect valid spo2/pulse readings for 2 minutes, the module will flash the blue LED
rapidly for 5 seconds and then go to sleep.




