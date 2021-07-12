#include <Arduino.h>
#include "oled.h"

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE); 

extern float BatteryVoltage;
extern bool FlagInternetAccess;

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

