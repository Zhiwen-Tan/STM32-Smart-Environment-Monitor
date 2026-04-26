#ifndef __OLED_H
#define __OLED_H

#include "main.h"

void OLED_Init(void);
void OLED_StartRefresh(void);
void OLED_Clear(void);
void OLED_DisplayOn(void);
void OLED_DisplayOff(void);
void OLED_ShowString(uint8_t x, uint8_t y, char *chr, uint8_t size);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
void OLED_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t int_len, uint8_t dec_len, uint8_t size);

#endif
