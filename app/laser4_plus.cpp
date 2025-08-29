#include "board.h"
#include "app.h"
#include "laser4_plus.h"
#include "multiprotocol.h"
#include "mpanel.h"
#include "tone.h"
#include "debug.h"

#if defined(ENABLE_VCP) || defined(ENABLE_GAME_CONTROLLER)
#include "usb_device.h"
#include "usbd_cdc_if.h"
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
#define IS_BAT_LOW              (APP_FLAGS & (1<<0))
#define IS_BAT_ICO_ON           (APP_FLAGS & (1<<1))
#define IS_ERROR_ICO_ON         (APP_FLAGS & (1<<2))
#define IS_BIND_ICO_ON          (APP_FLAGS & (1<<3))
#define IS_LCD_UPDATE           (APP_FLAGS & (1<<4))

#define SET_BAT_LOW             APP_FLAGS = (APP_FLAGS | (1<<0))
#define CLR_BAT_LOW             APP_FLAGS = APP_FLAGS & ~(1<<0)

#define SET_BAT_ICO             APP_FLAGS = (APP_FLAGS | (1<<1))
#define CLR_BAT_ICO             APP_FLAGS = APP_FLAGS & ~(1<<1)

#define SET_ERROR_ICO           APP_FLAGS = (APP_FLAGS | (1<<2))
#define CLR_ERROR_ICO           APP_FLAGS = APP_FLAGS & ~(1<<2)

#define SET_BIND_ICO            APP_FLAGS = (APP_FLAGS | (1<<3))
#define CLR_BIND_ICO            APP_FLAGS = APP_FLAGS & ~(1<<3)

#define SET_LCD_UPDATE          APP_FLAGS = (APP_FLAGS | (1<<4))
#define CLR_LCD_UPDATE          APP_FLAGS = APP_FLAGS & ~(1<<4)


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

#define DRO_BAT_POS     4, 10
#define DRO_AMPH_POS    74, 10
#define ICO_LOWBAT_POS  1, 1  // 29x7
#define ICO_ERROR_POS   31, 1 // 7x7
#define ICO_BIND_POS    55, 9 // 7x7
#define ICO_35MHZ_POS   38, 1 // 15x7
#define ICO_2_4GHZ_POS  53, 1 // 17x7
#define ICO_USB_POS     70, 1 // 13x7
#define DRO_MA_POS      88, 1

#define VERSION_POS     8,8

#define ICO_CLR_START   ICO_35MHZ_POS
#define ICO_CLR_SIZE    15+17+13, 8

#define APP_DRAW_ICON(ICO)      MPANEL_drawIcon(ICO.posx, ICO.posy, (const idata_t*)ICO.data)
#define APP_ERASE_ICON(ICO)     LCD_FillRect(ICO.posx, ICO.posy, ICO.data->width, ICO.data->hight, BLACK);

/**
 * Icons bitmaps
 * */
const uint8_t ico_volt_data[] = {7,8,
    0x7f,0x5d,0x5d,0x5d,0x5d,0x6b,0x77,0x7f
};

const uint8_t ico_amph_data[] = {10, 8,
    0x03,0xff,0x03,0x3f,0x02,0xd7,0x02,0xd7,0x02,0x13,0x02,0xd5,0x02,0xd5,0x03,0xff
};

const uint8_t ico_lowbat_data[] = {29, 7,
    0x1f,0xff,0xff,0xff,0x17,0xff,0xff,0xff,0x17,0x9b,0xb7,0xb1,0x17,0x6b,
    0xb3,0x5b,0x17,0x6a,0xb5,0x1b,0x11,0x9d,0x71,0x5b,0x1f,0xff,0xff,0xff
};

const uint8_t ico_35mhz_data[] = {15, 7,
    0x7f,0xff,0x44,0x5d,0x75,0xc9,0x44,0x55,0x77,0x5d,0x44,0x5d,0x7f,0xff
};

const uint8_t ico_2_4ghz_data[] = {15, 7,
    0x7f,0xff,0x46,0xb3,0x76,0xaf,0x46,0x29,0x5f,0xad,0x45,0xb1,0x7f,0xff
};

const uint8_t ico_usb_data[] = {13, 7,
    0x1f,0xff,0x15,0x13,0x15,0x75,0x15,0x13,0x15,0xd5,0x11,0x13,0x1f,0xff
};

const uint8_t ico_error_data[] = {7, 7,
    0x08,0x14,0x1c,0x2a,0x22,0x49,0x7f,
};

const uint8_t ico_bind_data[] = {7,7,
    0x04,0x0a,0x01,0x2a,0x40,0x28,0x10,
};

/**
 * Icons structures
 * */
static mpanelicon_t ico_volt = {
    (uint16_t)(font_seven_seg.w * 3 + 7),
    (uint16_t)(font_seven_seg.h/2),
    (idata_t*)ico_volt_data
};

static mpanelicon_t ico_amph = {
    (uint16_t)(font_seven_seg.w * 3 + 7),
    (uint16_t)(font_seven_seg.h/2),
    (idata_t*)ico_amph_data
};

static mpanelicon_t ico_35mhz = {
    ICO_35MHZ_POS,
    (idata_t*)ico_35mhz_data
};

static mpanelicon_t ico_2_4ghz = {
    ICO_2_4GHZ_POS,
    (idata_t*)ico_2_4ghz_data
};

static mpanelicon_t ico_usb = {
    ICO_USB_POS,
    (idata_t*)ico_usb_data
};

static mpanelicon_t ico_low_bat = {
    ICO_LOWBAT_POS,
    (idata_t*)ico_lowbat_data
};

static mpanelicon_t ico_error = {
    ICO_ERROR_POS,
    (idata_t*)ico_error_data
};

static mpanelicon_t ico_bind = {
    ICO_BIND_POS,
    (idata_t*)ico_bind_data
};

static MpanelDro dro_bat(DRO_BAT_POS, "%.2f",&font_seven_seg);
static MpanelDro dro_amph(DRO_AMPH_POS, "%.2f",&font_seven_seg);
static MpanelDro dro_ma(DRO_MA_POS, "%3uMA", &pixelDustFont);

static uint8_t bat_low_tim;
static float bat_consumed = 0;  //mAh

void appToggleLowBatIco(void);
#endif

/**
 * EEPROM ram copy
 * */

static const uint16_t eeprom_default_data[EEPROM_SIZE / 2] = {
    (uint16_t)DEFAULT_ID, (uint16_t)(DEFAULT_ID>>16),
    (uint16_t)DEFAULT_VOLTAGE_DIV,(uint16_t)(DEFAULT_VOLTAGE_DIV>>16),
    (uint16_t)DEFAULT_SENSE_RESISTOR,(uint16_t)(DEFAULT_SENSE_RESISTOR>>16),
    CHANNEL_MAX_100, CHANNEL_MIN_100,
    CHANNEL_MAX_125, CHANNEL_MIN_125,
    PPM_MAX_100, PPM_MIN_100,
    CHANNEL_SWITCH, PPM_DEFAULT_VALUE,
    BUZ_DEFAULT_VOLUME
};

/**
 * @brief
 * */
uint8_t appGetCurrentMode(void){
    return app_state & STATE_MASK;
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

#ifdef ENABLE_DISPLAY
    // Erase mode icon
    LCD_FillRect(ICO_CLR_START, ICO_CLR_SIZE, BLACK);
#endif

    // set the requested mode by overwriting the previous
    app_state = (new_mode << MODE_BIT_POS) | MODE_CHANGE_REQ;
}

/**
 * @brief Change mode request handler
 *
 * */
static void appChangeMode(uint8_t new_mode)
{
    switch(new_mode){
        case MODE_CC2500:
        case MODE_PPM:
#ifdef ENABLE_BUZZER
            buzPlayTone(400,150);
#endif
#ifdef ENABLE_DISPLAY
            if(new_mode == MODE_PPM){
                APP_DRAW_ICON(ico_35mhz);
            }else{
                APP_DRAW_ICON(ico_2_4ghz);
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
            APP_DRAW_ICON(ico_usb);
    #endif /* ENABLE_DISPLAY */
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
 * @brief Periodic called function to display battery voltage
 *
 * */
void appCheckBattery(void)
{
    vires_t res;
    if (batteryReadVI(&res)) {
        if (res.vbat < BATTERY_VOLTAGE_MIN && !IS_BAT_LOW) {
            SET_BAT_LOW;
            DBG_APP_WRN(DBG_TAG "!!Low battery !! (%dmV)", res.vbat);
#ifdef ENABLE_DISPLAY
            // Start blinking timer
            bat_low_tim = startTimer(TIMER_LOWBAT_TIME, SWTIM_AUTO_RELOAD, appToggleLowBatIco);
        } else {
            // Vbat has recover, disable low battery icon
            if(IS_BAT_LOW) {
                CLR_BAT_LOW;

                stopTimer(bat_low_tim);
                if (IS_BAT_ICO_ON) {
                    appToggleLowBatIco();
                }
            }
        }

        // update battery voltage DRO
        dro_bat.update(res.vbat / 1000.0f);
        // [Ah] are given by the periodic call
        bat_consumed += (float)(res.cur / (float)(3600 / (TIMER_BATTERY_TIME / 1000)));   //1h/30s
        dro_amph.update(bat_consumed / 1000.0f);
        dro_ma.update(res.cur);
        SET_LCD_UPDATE;
#else
        }else{
            // Vbat has recover, clear flag
            CLR_BAT_LOW;
        }
#endif /* ENABLE_DISPLAY */
    }
}
#endif /* ENABLE_BATTERY_MONITOR */


#ifdef ENABLE_DISPLAY
/**
 * @brief blink low battery icon
 * */
void appToggleLowBatIco(void){
    if(!(IS_BAT_ICO_ON)){
        SET_BAT_ICO;
        APP_DRAW_ICON(ico_low_bat);
    }else{
        CLR_BAT_ICO;
        APP_ERASE_ICON(ico_low_bat);
    }
    SET_LCD_UPDATE;
}

/**
 * @brief check multiprotocol flags and place icons
 * accordingly
 * */
void appCheckProtocolFlags(void)
{
    uint32_t flags = multiprotocol_flags_get();

    if(!(flags & FLAG_INPUT_SIGNAL)){
        if(!(IS_ERROR_ICO_ON)){
            SET_ERROR_ICO;
            APP_DRAW_ICON(ico_error);
            SET_LCD_UPDATE;
        }
    }else{
         if(IS_ERROR_ICO_ON){
            CLR_ERROR_ICO;
            APP_ERASE_ICON(ico_error);
            SET_LCD_UPDATE;
        }
    }

    if(!(flags & FLAG_BIND)){
         if(!(IS_BIND_ICO_ON)){
            SET_BIND_ICO;
            APP_DRAW_ICON(ico_bind);
            SET_LCD_UPDATE;
        }
    }else{
         if(IS_BIND_ICO_ON){
            CLR_BIND_ICO;
            APP_ERASE_ICON(ico_bind);
            SET_LCD_UPDATE;
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
        if(tmp > eeprom_data[IDX_CHANNEL_MAX_100]){
            tmp = eeprom_data[IDX_CHANNEL_MAX_100];
        }else if(tmp < eeprom_data[IDX_CHANNEL_MIN_100]){
            tmp = eeprom_data[IDX_CHANNEL_MIN_100];
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
            channel_value = eeprom_data[IDX_CHANNEL_SWITCH];
        }else{
            channel_value = eeprom_data[IDX_CHANNEL_MIN_100];
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

static uint32_t eepromLoad(uint8_t *buf, uint16_t size)
{
    return EEPROM_Read(0, buf, size);
}

/**
 * @brief
 * @param buf
 * @param defaults
 * @param size
 */
static void eepromDefault(uint8_t *buf, const uint8_t *defaults, uint16_t size)
{
    memcpy(buf, defaults, size);
}

/**
 * @brief
 *
 * */
static uint8_t* eepromInit(const uint8_t *defaults, uint16_t size)
{
    uint8_t bind_flag;
    uint8_t *buf;

    buf = EEPROM_Init(size);

    if(EEPROM_Read(EEPROM_BIND_FLAG, &bind_flag, 1) != 1){
        DBG_APP_ERR("Error reading EEPROM");
        goto load_defaults;
    }

    if(bind_flag == BIND_FLAG_VALUE){
        if(EEPROM_Read(0, buf, size) != size){
            DBG_APP_ERR("Error reading EEPROM");
            goto load_defaults;
        }
        DBG_APP_INF("Data loaded from EEPROM");
        return buf;
    }

load_defaults:
    eepromDefault(buf, defaults, size);
    DBG_APP_INF("EEPROM defaults loaded");

    return buf;
}

/**
 * @brief
 * @param
 */
void appLoadEEPROM(void)
{
    if(!eepromLoad((uint8_t*)eeprom_data, EEPROM_SIZE)){
        DBG_APP_ERR("Error reading EEPROM");
    }
}

/**
 * @brief Save the ram eeprom content to flash memory
 * Note: In order to save eeprom
 *
 * */
void appSaveEEPROM(void)
{
    // FIXME: Really should implement CRC
    *((uint8_t*)eeprom_data + EEPROM_BIND_FLAG) = BIND_FLAG_VALUE;

    EEPROM_Write(EEPROM_ID_OFFSET, (uint8_t*)eeprom_data, EEPROM_SIZE);

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
    eepromDefault((uint8_t*)eeprom_data, (const uint8_t*)eeprom_default_data, EEPROM_SIZE);
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
    USB_DEVICE_RegisterCallback(HAL_PCD_SUSPEND_CB_ID, usbDisconnectCB, NULL);
    USB_DEVICE_RegisterCallback(HAL_PCD_RESUME_CB_ID, usbConnectCB, NULL);
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
    eeprom_data = (uint16_t*)eepromInit((const uint8_t*)eeprom_default_data, EEPROM_SIZE);
#ifdef ENABLE_BUZZER
    // Set volume from stored value
    buzSetLevel(*((uint8_t*)eeprom_data + IDX_BUZ_VOLUME));
    // Play som random tone
    buzPlay(chime);
#endif

#ifdef ENABLE_BATTERY_MONITOR
    // Configure adc calibration values
    f2u_u tmp;
    tmp.u = (uint32_t)(eeprom_data[IDX_BAT_VOLTAGE_DIV] | (eeprom_data[IDX_BAT_VOLTAGE_DIV + 1] << 16));
    adcSetVdivRacio(tmp.f);
    tmp.u = (uint32_t)(eeprom_data[IDX_SENSE_RESISTOR] | (eeprom_data[IDX_SENSE_RESISTOR + 1] << 16));
    adcSetSenseResistor(tmp.f);

    /* Get battery voltage */
    DBG_APP_INF("Battery voltage: %dmV", batteryGetVoltage());
#endif

#ifdef ENABLE_DISPLAY

    MPANEL_print(VERSION_POS, &pixelDustFont, "V%u.%u.%u", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
    LCD_Update();
    DelayMs(1000);
    LCD_FillRect(VERSION_POS, 64, pixelDustFont.h, BLACK); // Erase version from display

    dro_bat.setIcon(&ico_volt);
    dro_amph.setIcon(&ico_amph);
    dro_bat.draw();
    dro_amph.draw();
    dro_ma.draw();
    startTimer(TIMER_BATTERY_TIME, SWTIM_AUTO_RELOAD, appCheckBattery);

    appCheckBattery();

    startTimer(TIMER_PPM_TIME, SWTIM_AUTO_RELOAD, appCheckProtocolFlags);
    SET_LCD_UPDATE;
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
    #ifdef ENABLE_DISPLAY
            SET_LCD_UPDATE;
    #endif
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
    if(IS_LCD_UPDATE){
        if(requestLcdUpdate()){
            CLR_LCD_UPDATE;
        }
    }
#endif
    reloadWatchDog();
}

