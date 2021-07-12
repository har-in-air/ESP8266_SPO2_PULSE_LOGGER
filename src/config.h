#ifndef CONFIG_H_
#define CONFIG_H_

// config button pin for on-demand Configuration portal
#define pinCfg         0 

// pmos transistor switched power for OLED display
// drive low to turn on
#define pinOLEDPwr    13

// If pulse not detected in one minute, unit goes to sleep to save power
#define SENSOR_TIMEOUT_CYCLES 60


#endif