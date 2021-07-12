#ifndef OLED_H_
#define OLED_H_

#include <U8g2lib.h>

extern U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2;
void oled_display_data(char* format, ...);
void oled_print_buffer(bool clearBuf, int x, int y, const uint8_t* font, char* format, ...);

#endif