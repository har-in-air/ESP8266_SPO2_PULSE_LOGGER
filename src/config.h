#ifndef CONFIG_H_
#define CONFIG_H_

extern bool FlagInternetAccess;
 
extern char SzThingSpeakChannel[];
extern char SzThingSpeakWriteAPIKey[];
extern char SzThingSpeakUpdateSecs[];

extern unsigned long ThingSpeakChannel;
extern unsigned long ThingSpeakUpdateSecs;

void load_config_data();
void wifi_config();
void wifi_on();
void wifi_off();

#endif