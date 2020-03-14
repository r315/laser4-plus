#ifndef _mpanel_h_
#define _mpanel_h_

#include <stdint.h>
#include <font.h>

#define PANEL_MAX_LEN   (128/8)

typedef struct mpanleitemdata{
    uint8_t width;
    uint8_t hight;
    uint8_t data[];
}idata_t;

typedef struct mpanelicon{
    uint16_t posx;
    uint16_t posy;
    struct mpanleitemdata *data;
}mpanelicon_t;

class MpanelItem{
protected:
    idata_t *idata;
    uint16_t posx;
    uint16_t posy;
public:
    virtual void draw(void);
};

class MpanelDro : MpanelItem{
    font_t *font;
    float value;
    mpanelicon_t *icon;
public:
    void update(float value);
    void draw(void);
    void setIcon(mpanelicon_t *icon);
    MpanelDro(uint16_t posx, uint16_t posy, font_t *font){
        this->posx = posx;
        this->posy = posy;
        this->font = font;
        this->value = 0.0f;
    }    
};

extern font_t font_seven_seg;

#endif /* _mpanel_h_ */