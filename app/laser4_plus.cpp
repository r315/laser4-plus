#include "board.h"
#include "app.h"
#include "laser4_plus.h"
#include "multiprotocol.h"
#include "mpanel.h"
#include "tone.h"
#include "debug.h"

#if defined(ENABLE_VCP) || defined(ENABLE_GAME_CONTROLLER)
#include "usb_device.h"
#endif

#ifdef ENABLE_DEBUG_APP
#define DBG_TAG     "APP : "
#define DBG_APP_INF(...) DBG_INF(DBG_TAG __VA_ARGS__)
#define DBG_APP_WRN(...) DBG_WRN(DBG_TAG __VA_ARGS__)
#define DBG_APP_ERR(...) DBG_ERR(DBG_TAG __VA_ARGS__)
#else
#define DBG_APP_INF(...)
#define DBG_APP_WRN(...)
#define DBG_APP_ERR(...)
#endif


#define APP_FLAGS               app_flags
#define APP_FLAG_BATLOW         (1 << 0)
#define APP_FLAG_BATICON        (1 << 1)
#define APP_FLAG_ERR            (1 << 2)
#define APP_FLAG_BINDICON       (1 << 3)
#define APP_FLAG_DISPLAY        (1 << 4)
#define APP_FLAG_DISPLAY_UP     (1 << 5)

#define APP_FLAG_CHECK(_F)      (APP_FLAGS & (_F))
#define IS_BAT_LOW              APP_FLAG_CHECK(APP_FLAG_BATLOW)
#define IS_BAT_ICO_VISIBLE      APP_FLAG_CHECK(APP_FLAG_BATICON)
#define IS_ERR_ICO_VISIBLE      APP_FLAG_CHECK(APP_FLAG_ERR)
#define IS_BIND_ICO_ON          APP_FLAG_CHECK(APP_FLAG_BINDICON)
#define IS_DISPLAY_ENABLED      APP_FLAG_CHECK(APP_FLAG_DISPLAY)
#define IS_DISPLAY_UP_PENDING   APP_FLAG_CHECK(APP_FLAG_DISPLAY_UP)

#define APP_FLAG_BAT_LOW_SET    APP_FLAGS = (APP_FLAGS | APP_FLAG_BATLOW)
#define APP_FLAG_BAT_LOW_CLR    APP_FLAGS = (APP_FLAGS & ~APP_FLAG_BATLOW)

#define APP_FLAG_BAT_ICO_SET    APP_FLAGS = (APP_FLAGS | APP_FLAG_BATICON)
#define APP_FLAG_BAT_ICO_CLR    APP_FLAGS = (APP_FLAGS & ~APP_FLAG_BATICON)

#define APP_FLAG_ERR_ICO_SET    APP_FLAGS = (APP_FLAGS | APP_FLAG_ERR)
#define APP_FLAG_ERR_ICO_CLR    APP_FLAGS = (APP_FLAGS & ~APP_FLAG_ERR)

#define APP_FLAG_BIND_ICO_SET   APP_FLAGS = (APP_FLAGS | APP_FLAG_BINDICON)
#define APP_FLAG_BIND_ICO_CLR   APP_FLAGS = (APP_FLAGS & ~APP_FLAG_BINDICON)

#define APP_FLAG_DISPLAY_UP_SET APP_FLAGS = (APP_FLAGS | APP_FLAG_DISPLAY_UP)
#define APP_FLAG_DISPLAY_UP_CLR APP_FLAGS = (APP_FLAGS & ~APP_FLAG_DISPLAY_UP)

#define APP_FLAG_DISPLAY_SET    APP_FLAGS = (APP_FLAGS | APP_FLAG_DISPLAY)
#define APP_FLAG_DISPLAY_CLR    APP_FLAGS = (APP_FLAGS & ~APP_FLAG_DISPLAY)


static volatile uint8_t app_state;
static uint32_t app_flags;

#ifdef ENABLE_BUZZER
static tone_t chime[] = {
    {493,200},
    {932,200},
    {1244,200},
    {1864,200},
    {0,0}
};
#endif

#if defined(ENABLE_CLI)
static Console con;
#endif

#ifdef ENABLE_DISPLAY

#define ICO_LOWBAT_POS_X    1
#define ICO_35MHZ_POS_X     (ICO_LOWBAT_POS_X + 29 + 2)
#define ICO_2_4GHZ_POS_X    (ICO_35MHZ_POS_X + 15 + 2)
#define ICO_USB_POS_X       (ICO_2_4GHZ_POS_X + 15 + 2)

#define DRO_BAT_POS     4, 10
#define DRO_AMPH_POS    74, 10
#define DRO_MA_POS      88, 1
#define ICO_LOWBAT_POS  ICO_LOWBAT_POS_X, 1
#define ICO_35MHZ_POS   ICO_35MHZ_POS_X, 1
#define ICO_2_4GHZ_POS  ICO_2_4GHZ_POS_X, 1
#define ICO_USB_POS     ICO_USB_POS_X, 1
#define ICO_ERROR_POS   62, 11
#define ICO_BIND_POS    62, 20

#define VERSION_POS     8,8

#define ICO_CLR_START   ICO_35MHZ_POS
#define ICO_CLR_SIZE    15+15+13+4, 8

#define APP_DRAW_ICON(ICO)      MPANEL_drawIcon(ICO.posx, ICO.posy, (const idata_t*)ICO.data)
#define APP_ERASE_ICON(ICO)     LCD_FillRect(ICO.posx, ICO.posy, ICO.data->width, ICO.data->hight, BLACK);

/**
 * Icons bitmaps
 * */
static const uint8_t ico_volt_data[] = {7, 8,
    0x7f,0x5d,0x5d,0x5d,0x5d,0x6b,0x77,0x7f
};

static const uint8_t ico_amph_data[] = {10, 8,
    0x03,0xff,0x03,0x3f,0x02,0xd7,0x02,0xd7,0x02,0x13,0x02,0xd5,0x02,0xd5,0x03,0xff
};

static const uint8_t ico_lowbat_data[] = {29, 7,
    0x1f,0xff,0xff,0xff,0x17,0xff,0xff,0xff,0x17,0x9b,0xb7,0xb1,0x17,0x6b,
    0xb3,0x5b,0x17,0x6a,0xb5,0x1b,0x11,0x9d,0x71,0x5b,0x1f,0xff,0xff,0xff
};

static const uint8_t ico_35mhz_data[] = {15, 7,
    0x7f,0xff,0x44,0x5d,0x75,0xc9,0x44,0x55,0x77,0x5d,0x44,0x5d,0x7f,0xff
};

static const uint8_t ico_2_4ghz_data[] = {15, 7,
    0x7f,0xff,0x46,0xb3,0x76,0xaf,0x46,0x29,0x5f,0xad,0x45,0xb1,0x7f,0xff
};

static const uint8_t ico_usb_data[] = {13, 7,
    0x1f,0xff,0x15,0x13,0x15,0x75,0x15,0x13,0x15,0xd5,0x11,0x13,0x1f,0xff
};

static const uint8_t ico_error_data[] = {7, 7,
    0x08,0x14,0x1c,0x2a,0x22,0x49,0x7f,
};

static const uint8_t ico_bind_data[] = {7,7,
    0x04,0x0a,0x01,0x2a,0x40,0x28,0x10,
};

static const uint8_t ico_ma_data[] = {12,5,
    0x00,0x3f,0x00,0x21,0x0d,0xbf,0x0a,0xa1,0x0a,0xa1
};

/**
 * Icons structures
 * */
static mpanelicon_t ico_volt = {
    (uint16_t)(font_seven_seg.w * 3 + 7),
    (uint16_t)(font_seven_seg.h/2),
    (const idata_t*)ico_volt_data
};

static mpanelicon_t ico_amph = {
    (uint16_t)(font_seven_seg.w * 3 + 7),
    (uint16_t)(font_seven_seg.h/2),
    (const idata_t*)ico_amph_data
};

static const mpanelicon_t ico_35mhz = {
    ICO_35MHZ_POS,
    (const idata_t*)ico_35mhz_data
};

static mpanelicon_t ico_2_4ghz = {
    ICO_2_4GHZ_POS,
    (const idata_t*)ico_2_4ghz_data
};

static mpanelicon_t ico_usb = {
    ICO_USB_POS,
    (const idata_t*)ico_usb_data
};

static mpanelicon_t ico_low_bat = {
    ICO_LOWBAT_POS,
    (const idata_t*)ico_lowbat_data
};

static mpanelicon_t ico_error = {
    ICO_ERROR_POS,
    (const idata_t*)ico_error_data
};

static mpanelicon_t ico_bind = {
    ICO_BIND_POS,
    (const idata_t*)ico_bind_data
};

static mpanelicon_t ico_ma = {
    24, 1, // offset in relation to dro_ma position
    (const idata_t*)ico_ma_data
};

static MpanelDro dro_bat(DRO_BAT_POS, "%.2f",&font_seven_seg);
static MpanelDro dro_amph(DRO_AMPH_POS, "%.2f",&font_seven_seg);
static MpanelDro dro_ma(DRO_MA_POS, "%3u", &pixelDustFont);

#ifdef ENABLE_BATTERY_MONITOR
static uint8_t bat_low_tim;
#endif

void appToggleLowBatIco(void);
#endif

#ifdef ENABLE_BATTERY_MONITOR
static float bat_consumed;
#endif

/**
 * EEPROM ram copy
 * */
static const meep_t eeprom_default_data = {
    .bind = 0,
    .buz_vol = BUZ_DEFAULT_VOLUME,
    .rfu1 = 0xFE,
    .rfu2 = 0xFF,
    .uid = DEFAULT_ID,
    .vdiv = DEFAULT_VOLTAGE_DIV,
    .rsense = DEFAULT_SENSE_RESISTOR,
    .servo_max_100 = CHANNEL_MAX_100,
    .servo_min_100 = CHANNEL_MIN_100,
    .servo_max_125 = CHANNEL_MAX_125,
    .servo_min_125 = CHANNEL_MIN_125,
    .switch_on = CHANNEL_SWITCH,
    .switch_off = CHANNEL_MIN_100,
    .ppm_max_100 = PPM_MAX_100,
    .ppm_min_100 = PPM_MIN_100,
    .cksum = 0
};

/**
 * @brief
 *
 * @param data
 * @param len
 * @return
 */
static uint8_t crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;   // initial value (can also be 0xFF depending on protocol)

    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];   // XOR-in the next input byte

        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;   // polynomial 0x07
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Usb connect callback, called when usb cable is connected
 * or if the system is power on with the usb cable plugged in
 *
 * @param ptr : pointer passed when the callback is registered
 * */
void usbConnectCB(void *ptr)
{
    (void)ptr;
    appChangeModeReq(app_state, MODE_HID);
#if defined(ENABLE_DEBUG) && defined(EANBLE_VCP)
    dbg_init(&vcp);
#endif

#if defined(ENABLE_CLI) && defined(EANBLE_VCP)
    // redirect cli to vcom
    con.setOutput(&vcp);
#endif
}

/**
 * @brief Usb disconnect callback, called when usb cable is removed.
 *
 * @param ptr : pointer passed when the callback is registered
 * */
void usbDisconnectCB(void *ptr)
{
    (void)ptr;
    appChangeModeReq(app_state, MODE_CC2500);
#if defined(ENABLE_DEBUG) && defined(ENABLE_UART)
    dbg_init(&pcom);
#endif

#if defined(ENABLE_CLI) && defined(ENABLE_UART)
    // redirect cli to physical com port
    con.setOutput(&pcom);
#endif
}

/**
 * @brief Get current operation mode
 * @return  MODE_CHANGE_REQ
 *          MODE_CC2500
 *          MODE_HID
 *          MODE_PPM
 *          MODE_NONE
 *
 * */
uint8_t appGetCurrentMode(void)
{
    return app_state & STATE_MASK;
}

/**
 * @brief Get number of seconds elapsed sinse powerup
 * @param
 * @return
 */
uint32_t appGetUpTime(void)
{
    return millis() / 1000UL;
}

/**
 * @brief Operating mode change request
 *
 * */
void appChangeModeReq(uint8_t prev_mode, uint8_t new_mode)
{
    uint8_t cur_state = app_state & STATE_MASK;

    // Do nothing if requesting the current mode or invalid
    if(cur_state == new_mode || new_mode > MODE_NONE){
        return;
    }
    // Request in progress, if same return
    if(cur_state == MODE_CHANGE_REQ){
        if(((app_state & MODE_MASK) >> MODE_BIT_POS) == new_mode){
            return;
        }
    }

    switch(new_mode){
        default:
            app_state = prev_mode;
            // default to last mode on invalid new mode
            return;
        case MODE_HID:
        case MODE_PPM:
        case MODE_SERIAL:
        case MODE_CC2500:
            multiprotocol_mode_set(new_mode);
        break;
    }

    // set the requested mode by overwriting the previous
    app_state = (new_mode << MODE_BIT_POS) | MODE_CHANGE_REQ;
}

/**
 * @brief Change mode request handler
 *
 * */
static void appChangeMode(uint8_t new_mode)
{
#ifdef ENABLE_DISPLAY
    if(IS_DISPLAY_ENABLED){
        LCD_FillRect(ICO_CLR_START, ICO_CLR_SIZE, BLACK);
        APP_FLAG_DISPLAY_UP_SET;
    }
#endif

    switch(new_mode){
        case MODE_CC2500:
        case MODE_PPM:
#ifdef ENABLE_BUZZER
            buzPlayTone(400,150);
#endif
#ifdef ENABLE_DISPLAY
            if(IS_DISPLAY_ENABLED){
                if(new_mode == MODE_PPM){
                    APP_DRAW_ICON(ico_35mhz);
                }else{
                    APP_DRAW_ICON(ico_2_4ghz);
                }
            }
#endif
            break;
        case MODE_HID:
#ifdef ENABLE_GAME_CONTROLLER
            DBG_APP_INF("\n ***** Starting game controller *****");
    #ifdef ENABLE_BUZZER
            buzPlayTone(2000,150);
    #endif
    #ifdef ENABLE_DISPLAY
            if(IS_DISPLAY_ENABLED){
                APP_DRAW_ICON(ico_usb);
            }
    #endif
#endif /* ENABLE_GAME_CONTROLLER */
            LED_OFF;
            break;
        default:
            return;
    }

    multiprotocol_setup();
}

#ifdef ENABLE_BATTERY_MONITOR
/**
 * @brief Get battery consumed in mA/h
 * @return mA/h
 */
uint32_t appGetBatConsumed(void)
{
    return bat_consumed;
}

/**
 * @brief Called every 10s to update battery consumption.
 * If display feature is enabled and display is detected
 * display it.
 *
 * */
void appCheckBattery(void)
{
    vires_t res;
    if (batteryReadVI(&res)) {
#ifdef ENABLE_BATTERY_MONITOR
        bat_consumed += (float)(res.cur / (float)(3600 / (TIMER_BATTERY_TIME / 1000)));
#endif
        if (res.vbat < BATTERY_VOLTAGE_MIN) {
            if(!IS_BAT_LOW){
                APP_FLAG_BAT_LOW_SET;
                DBG_APP_WRN("!! Battery Low (%dmV) !!", res.vbat);
#ifdef ENABLE_DISPLAY
                // Start blinking timer
                bat_low_tim = startTimer(TIMER_LOWBAT_TIME, SWTIM_AUTO_RELOAD, appToggleLowBatIco);
            }
        } else if(IS_BAT_LOW){
            // Vbat has recover, disable low battery icon
            APP_FLAG_BAT_LOW_CLR;
            if(IS_DISPLAY_ENABLED){
                stopTimer(bat_low_tim);
                if (IS_BAT_ICO_VISIBLE) {
                    appToggleLowBatIco();
                }
            }
        }

        if(IS_DISPLAY_ENABLED){
            // update battery voltage DRO
            dro_bat.update(res.vbat / 1000.0f);
            // [Ah] are given by the periodic call
            dro_amph.update(bat_consumed / 1000.0f);
            dro_ma.update(res.cur);
            APP_FLAG_DISPLAY_UP_SET;
        }
#else
            }
        }else{
            // Vbat has recover, clear flag
            APP_FLAG_BAT_LOW_CLR;
        }
#endif /* ENABLE_DISPLAY */
    }
}
#endif /* ENABLE_BATTERY_MONITOR */


#ifdef ENABLE_DISPLAY
/**
 * @brief blink low battery icon
 * called from timer and only if
 * display is enabled
 * */
void appToggleLowBatIco(void){
    if(!IS_DISPLAY_ENABLED){
        return;
    }

    if(!(IS_BAT_ICO_VISIBLE)){
        APP_FLAG_BAT_ICO_SET;
        APP_DRAW_ICON(ico_low_bat);
    }else{
        APP_FLAG_BAT_ICO_CLR;
        APP_ERASE_ICON(ico_low_bat);
    }

    APP_FLAG_DISPLAY_UP_SET;
}

/**
 * @brief check multiprotocol flags and place icons
 * accordingly
 * */
void appCheckProtocolFlags(void)
{
    uint32_t flags = multiprotocol_flags_get();

    if(!(flags & FLAG_INPUT_SIGNAL)){
        if(!(IS_ERR_ICO_VISIBLE)){
            APP_FLAG_ERR_ICO_SET;
            APP_DRAW_ICON(ico_error);
            APP_FLAG_DISPLAY_UP_SET;
        }
    }else{
         if(IS_ERR_ICO_VISIBLE){
            APP_FLAG_ERR_ICO_CLR;
            APP_ERASE_ICON(ico_error);
            APP_FLAG_DISPLAY_UP_SET;
        }
    }

    if(!(flags & FLAG_BIND)){
         if(!(IS_BIND_ICO_ON)){
            APP_FLAG_BIND_ICO_SET;
            APP_DRAW_ICON(ico_bind);
            APP_FLAG_DISPLAY_UP_SET;
        }
    }else{
         if(IS_BIND_ICO_ON){
            APP_FLAG_BIND_ICO_CLR;
            APP_ERASE_ICON(ico_bind);
            APP_FLAG_DISPLAY_UP_SET;
        }
    }
}

#endif /* ENABLE_DISPLAY */


/**
 * @brief Get auxiliary channels servo values available on board
 *
 * @param channel_aux Output pointer for servo data
 * @param nchannel  Output number os channels
 */
void appGetAuxChannels(uint16_t *channel_aux, uint8_t *nchannel)
{
    uint8_t nch = 0;

#ifdef ENABLE_AUX_ENCODER
    int16_t diff = auxGetEncoder();
    if(diff != 0){
        uint16_t tmp = channel_aux[nch];        // Read current servo value
        tmp += diff * 10;                       // increment/decrement
        if(tmp > eeprom->servo_max_100){
            tmp = eeprom->servo_max_100;
        }else if(tmp < eeprom->servo_min_100){
            tmp = eeprom->servo_min_100;
        }
        channel_aux[nch] = tmp;
        DBG_APP_INF("Encoder %d", diff);
    }

    nch += AUX_ENC_NUM;
#endif

#ifdef ENABLE_AUX_SWITCHES
    uint8_t switches = auxGetSwitches();
    uint16_t channel_value;

    for(uint8_t i = 0; i < AUX_SWITCH_NUM; i++){
        if((switches & (1 << i))){
            channel_value = eeprom->switch_on;
        }else{
            channel_value = eeprom->switch_off;
        }

        if(channel_value != channel_aux[i + nch]){
            channel_aux[i + nch] = channel_value;
            // This will always print on first call due to default values
            DBG_APP_INF("Aux channel %d changed", i + 1);
        }
    }

    nch += AUX_SWITCH_NUM;
#endif

    *nchannel = nch;
}

/**
 * @brief Loads eeprom data from board eeprom/flash to
 * application eeprom data structure and check if data
 * is valid using crc.
 *
 * If loaded data is somehow corrupted, default data is
 * then loaded
 *
 */
void appInitEEPROM(void)
{
    uint8_t crc;
    // Get pointer to eeprom bytes, EEPROM_Init
    // always returns valid pointer
    eeprom = (meep_t*)EEPROM_Init(sizeof(meep_t));

    crc = crc8((uint8_t*)eeprom, sizeof(meep_t) - 1);

    if(eeprom->cksum == crc){
        DBG_APP_INF("Data loaded from EEPROM");
    }else{
        DBG_APP_ERR("EEPROM checksum fail");
        appDefaultEEPROM();
        DBG_APP_INF("EEPROM defaults loaded");
    }
}

/**
 * @brief Save the ram eeprom content to flash memory
 * eeprom structure mustbe packed, CRC is added before
 * saving eeprom
 * */
void appSaveEEPROM(void)
{
    if(!eeprom){
        DBG_APP_WRN("EEPROM Not initialized");
        return;
    }

    uint8_t crc = crc8((uint8_t*)eeprom, sizeof(meep_t) - 1);

    EEPROM_Write((uint16_t)((uint8_t*)&eeprom->cksum - (uint8_t*)eeprom), &crc, 1);

    if(!EEPROM_Sync()){
        DBG_APP_ERR("!! Fail to sync EEPROM !!");
    }else{
        DBG_APP_INF("EEPROM Saved");
    }
}

/**
 * @brief
 *
 * @param
 */
void appDefaultEEPROM(void)
{
    for(uint32_t i = 0; i < sizeof(eeprom_default_data); i++){
        ((uint8_t*)eeprom)[i] = ((uint8_t*)&eeprom_default_data)[i];
    }
}

/**
 * @brief Application setup call
 * */
extern "C" void setup(void)
{
    app_state = MODE_NONE;
    app_flags = 0;

#if defined(ENABLE_DEBUG)
    #if defined(ENABLE_VCP)
    dbg_init(&vcp);
    #elif defined(ENABLE_UART)
    dbg_init(&pcom);
    #endif
#endif

#if defined(ENABLE_VCP) || defined(ENABLE_GAME_CONTROLLER)
    USB_DEVICE_Init();
    USB_DEVICE_RegisterSuspendCallback(usbDisconnectCB, NULL);
    USB_DEVICE_RegisterResumeCallback(usbConnectCB, NULL);
#endif

    DBG_PRINT("\n Laser4+ version: %d.%d.%d \n", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);

#if defined(ENABLE_CLI)
    #if defined(ENABLE_VCP)
    #define CONSOLE_STDINOUT &vcp
    #elif defined(ENABLE_UART)
    #define CONSOLE_STDINOUT &pcom
    #else
    #error "No stdinout defined for console"
    #endif
    con.init(CONSOLE_STDINOUT, "laser4+ >");
    con.registerCommandList(laser4_commands);
    con.cls();
#endif
    // Load eeprom data
    appInitEEPROM();
#ifdef ENABLE_BUZZER
    // Set volume from stored value
    buzSetLevel(eeprom->buz_vol);
    // Play som random tone
    buzPlay(chime);
#endif

#ifdef ENABLE_BATTERY_MONITOR
    // Configure adc calibration values
    f2u_u tmp;
    tmp.u = eeprom->vdiv;
    adcSetVdivRacio(tmp.f);
    tmp.u = eeprom->rsense;
    adcSetSenseResistor(tmp.f);

    bat_consumed = 0;  //mAh

    /* Get battery voltage */
    DBG_APP_INF("Battery voltage: %dmV", batteryGetVoltage());

    startTimer(TIMER_BATTERY_TIME, SWTIM_AUTO_RELOAD, appCheckBattery);
    appCheckBattery();
#endif

#ifdef ENABLE_DISPLAY
    if(displayInit()){
        APP_FLAG_DISPLAY_SET;
        MPANEL_print(VERSION_POS, &pixelDustFont, "V%u.%u.%u", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
        LCD_Update();
        DelayMs(1000);
        LCD_FillRect(VERSION_POS, 64, pixelDustFont.h, BLACK); // Erase version from display

        dro_bat.setIcon(&ico_volt);
        dro_amph.setIcon(&ico_amph);
        dro_bat.draw();
        dro_amph.draw();
        dro_ma.setIcon(&ico_ma);
        dro_ma.draw();

        startTimer(TIMER_PPM_TIME, SWTIM_AUTO_RELOAD, appCheckProtocolFlags);
        APP_FLAG_DISPLAY_UP_SET;
    }
#endif

#ifdef ENABLE_BUZZER
    // wait for melody to finish
    buzWaitEnd();
#endif
    // Configure watchdog
    enableWatchDog(WATCHDOG_TIME);
}

/**
 * @brief Application main loop
 * */
extern "C" void loop(void)
{
    switch(app_state & STATE_MASK){
        case MODE_CC2500:
        case MODE_PPM:
        case MODE_HID:
            multiprotocol_loop();
            break;

        case MODE_CHANGE_REQ:
            app_state = app_state >> MODE_BIT_POS;
            appChangeMode(app_state);
            break;

        case MODE_NONE:
        // Set based on hardware switch, usualy at startup
    #if defined(TX35_MHZ_INSTALLED) && defined(CC2500_INSTALLED)
            appChangeModeReq(app_state, IS_HW_SW_AUX3_PRESSED ? MODE_PPM : MODE_CC2500);
    #elif defined(CC2500_INSTALLED)
            appChangeModeReq(app_state, MODE_CC2500);
    #elif defined(TX35_MHZ_INSTALLED)
            appChangeModeReq(app_state, MODE_PPM);
    #else
            appChangeModeReq(app_state, MODE_SERIAL);
    #endif
            break;

        default:
            break;
    }

#if defined(ENABLE_CLI) && (defined(ENABLE_VCP) || defined(ENABLE_UART))
    con.process();
#endif

    processTimer();

#ifdef ENABLE_DISPLAY
   if(IS_DISPLAY_ENABLED){
        if(IS_DISPLAY_UP_PENDING){
            LCD_Update();
            APP_FLAG_DISPLAY_UP_CLR;
        }
    }
#endif

    reloadWatchDog();
}

