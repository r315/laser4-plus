/**
 * ==============================================
 * @file mpanel.cpp
 * @brief Message panel
 * ==============================================
 * */

#include "board.h"
#include <font.h>
#include "mpanel.h"
#include <stdarg.h>
#include <strfunc.h>

#ifdef ENABLE_DISPLAY
const uint8_t font_seven_seg_data[] = {
0x7f,0xe0,0xff,0xf0,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xff,0xf0,0x7f,0xe0,
0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,
0xff,0xe0,0xff,0xf0,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0xff,0xf0,0xff,0xf0,0xc0,0x00,0xc0,0x00,0xc0,0x00,0xc0,0x00,0xc0,0x00,0xc0,0x00,0xc0,0x00,0xff,0xf0,0x7f,0xf0,
0xff,0xe0,0xff,0xf0,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0xff,0xf0,0xff,0xf0,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0xff,0xf0,0xff,0xe0,
0x00,0x00,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xff,0xf0,0xff,0xf0,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,
0x7f,0xf0,0xff,0xf0,0xc0,0x00,0xc0,0x00,0xc0,0x00,0xc0,0x00,0xc0,0x00,0xc0,0x00,0xc0,0x00,0xff,0xf0,0xff,0xf0,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0xff,0xf0,0xff,0xe0,
0x7f,0xf0,0xff,0xf0,0xc0,0x00,0xc0,0x00,0xc0,0x00,0xc0,0x00,0xc0,0x00,0xc0,0x00,0xc0,0x00,0xff,0xe0,0xff,0xf0,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xff,0xf0,0x7f,0xe0,
0xff,0xe0,0xff,0xf0,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,
0x3f,0xe0,0xff,0xf0,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xff,0xf0,0xff,0xf0,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xff,0xf0,0x7f,0xe0,
0x7f,0xe0,0xff,0xf0,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xc0,0x30,0xff,0xf0,0xff,0xf0,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x7f,0xf0,0x7f,0xe0,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
};

font_t font_seven_seg = {
    .w = 12,
    .h = 20,
    .data = font_seven_seg_data,
    .data_len = sizeof(font_seven_seg_data),
    .bpl = 2,
    .offset = '0',
    .spacing = 1
};

static uint16_t seven_seg_dp(uint16_t x, uint16_t y){
    LCD_Fill(x-1, y, 4, 20, BLACK);
    LCD_Fill(x, y + 20 - 2, 2, 2, WHITE);
    return x + 3; 
}

/**
 * data format: w,h,data...
 * 
 */
void MPANEL_drawIcon(uint16_t x, uint16_t y, idata_t *data){
    uint8_t *p = data->data;

    for(uint8_t h = y; h < y + data->hight; h++, p++){
        uint8_t line = *p;
        uint8_t mask = (data->width-1) % 8;
        mask = (1 << mask);
        for(uint8_t w = x; w < x + data->width; w++, mask >>= 1){
            if(mask == 0){
                mask = 0x80;
                line = *(++p);
            }
            if(line & mask)
                LCD_Pixel(w, h, WHITE);
            else
                LCD_Pixel(w, h, BLACK);
            
        }
    }
}

/**
 * @brief draws a character using a font table.
 * the font table must be 1bpp
 */
static uint8_t MPANEL_drawChar(uint16_t x, uint16_t y, uint8_t c, font_t *font){
    const uint8_t *p;

    if(c == '.' && font == &font_seven_seg){
        return seven_seg_dp(x,y);
    }
    
    c -= font->offset;
    p = font->data + (c * font->h * font->bpl);

    if(p > font->data + font->data_len)
        return x;

    for(uint16_t h = y; h < y +font->h; h++, p++){
        uint8_t line = *p;
        for(uint16_t w = x, bc = 0; w < x + font->w; w++, bc++){
            if(bc == 8){
                bc = 0;
                line = *(++p);
            }
            if(line & (0x80>>bc))
                LCD_Pixel(w, h,WHITE);
            else
                LCD_Pixel(w, h,BLACK);
        }
    }
    return x + font->w + font->spacing;
}

void MPANEL_print(uint16_t x, uint16_t y, font_t *font, const char *fmt, ...){
    char buf[PANEL_MAX_LEN];
    uint8_t i = 0;
	va_list arp;
	va_start(arp, fmt);
	strformater(buf, fmt, arp);
	va_end(arp);

    while(buf[i] != '\0'){
        x = MPANEL_drawChar(x, y, buf[i++], font);
    }
}

void MpanelDro::update(float value){
    this->value = value;
    MPANEL_print(this->posx, this->posy, this->font, this->fmt, value);
}

void MpanelDro::update(uint32_t value){
    this->value = value;
    MPANEL_print(this->posx, this->posy, this->font, this->fmt, value);
}

void MpanelDro::draw(void){
    update(this->value);
    if(this->icon != NULL){
        MPANEL_drawIcon(this->icon->posx, this->icon->posy, this->icon->data);
    }
}

void MpanelDro::setIcon(mpanelicon_t *icon){
    icon->posx += this->posx;
    icon->posy += this->posy;
    this->icon = icon;
}

#endif