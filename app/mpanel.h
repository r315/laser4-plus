#ifndef _mpanel_h_
#define _mpanel_h_

#include <stdint.h>
#include <font.h>

#define PANEL_MAX_LEN   (128/8)

typedef struct idata{
    uint8_t width;
    uint8_t hight;
    uint8_t data[];
}idata_t;

typedef struct mpanelicon{
    uint16_t posx;
    uint16_t posy;
    const idata_t *data;
}mpanelicon_t;

class MpanelItem{
protected:
    const idata_t *idata = nullptr;
    uint16_t posx = 0;
    uint16_t posy = 0;
public:
    virtual ~MpanelItem() = default;  // important for polymorphic base
    virtual void draw(void) = 0;
};

class MpanelDro : public MpanelItem{
private:
    const font_t *font;
    mpanelicon_t *icon;
    const char *fmt;
    float value;
public:
    void draw(void) override;
    void update(float value);
    void update(uint32_t value);
    void setIcon(mpanelicon_t *icon);
    MpanelDro(uint16_t posx, uint16_t posy, const char *fmt, const font_t *font);
};

extern const font_t font_seven_seg;

void MPANEL_drawIcon(uint16_t x, uint16_t y, const idata_t *data);
void MPANEL_print(uint16_t x, uint16_t y, const font_t *font, const char *fmt, ...);

#endif /* _mpanel_h_ */