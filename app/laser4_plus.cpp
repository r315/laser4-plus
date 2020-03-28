#include "app.h"
#include "multiprotocol.h"
#include "usb_device.h"
#include "mpanel.h"


volatile uint8_t state;
static uint8_t bat_low;
static float bat_consumed = 0;  //mAh
static uint8_t bat_low_tim;
static uint8_t bat_low_ico_state = OFF;

tone_t chime[] = {
    {493,200},
    {932,200},
    {1244,200},
    {1864,200},
    {0,0}
};

#ifdef ENABLE_CLI 
Console con;
#endif

#ifdef ENABLE_DISPLAY
#define DRO_BAT_POS     4, 10
#define DRO_AMPH_POS    74, 10
#define DRO_MA_POS      88, 1
#define ICO_35MHZ_POS   34, 1
#define ICO_2_4GHZ_POS  52, 1
#define ICO_USB_POS     70, 1
#define ICO_LOWBAT_POS  1, 1

#define ICO_CLR_START   ICO_35MHZ_POS
#define ICO_CLR_SIZE    56, 8

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

MpanelDro dro_bat(DRO_BAT_POS, "%.2f",&font_seven_seg);
MpanelDro dro_amph(DRO_AMPH_POS, "%.2f",&font_seven_seg);
MpanelDro dro_ma(DRO_MA_POS, "%3uMA", &pixelDustFont);

void appToggleLowBatIco(void);
#endif

/**
 * EEPROM ram copy
 * */
uint16_t eeprom_data[EEPROM_SIZE / 2] = {
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
    return state;
}

/**
 * @brief Usb connect callback, called when usb cable is connected
 * or if the system is power on with the usb cable plugged in
 * 
 * @param ptr : pointer passed when the callback is registered
 * */
void usbConnectCB(void *ptr){
    appReqModeChange(MODE_HID);
#if defined(ENABLE_DEBUG) && defined(ENABLE_VCOM)
    dbg_init(&vcom);
#endif

#ifdef ENABLE_CLI
    // redirect cli to vcom
    con.setOutput(&vcom);
#endif
}

/**
 * @brief Usb disconnect callback, called when usb cable is removed.
 * 
 * @param ptr : pointer passed when the callback is registered
 * */
void usbDisconnectCB(void *ptr){
    appReqModeChange(MODE_MULTIPROTOCOL);
#if defined(ENABLE_DEBUG) && defined(ENABLE_USART)
    dbg_init(&pcom);
#endif
 
#ifdef ENABLE_CLI
    // redirect cli to physical com port
    con.setOutput(&pcom);
#endif
}

/**
 * @brief Operating mode change request
 * 
 * */
void appReqModeChange(uint8_t new_mode){
uint8_t cur_state = state & STATE_MASK;
    // Do nothing if requesting the current mode
    if(cur_state == new_mode){
        return;
    }
    // Request in progress, if same return
    if(cur_state == REQ_MODE_CHANGE){
        if((state >> STATE_BITS) == new_mode){
            return;
        } 
    }
#ifdef ENABLE_DISPLAY
    LCD_Fill(ICO_CLR_START, ICO_CLR_SIZE, BLACK);
#endif
    // set the requested mode by overwriting the previous
    state = (new_mode << STATE_BITS) | REQ_MODE_CHANGE;
}

/**
 * @brief Change mode request handler
 * 
 * */
static void changeMode(uint8_t new_mode){
    switch(new_mode){
        case MODE_MULTIPROTOCOL:
            DBG_PRINT("\n ***** Starting Multiprotocol *****\n");
            multiprotocol_setup();
            buzPlayTone(400,150);
#ifdef ENABLE_DISPLAY
            if(radio.mode_select == 14){
                MPANEL_drawIcon(ico_35mhz.posx, ico_35mhz.posy, ico_35mhz.data);  
            }else{
                MPANEL_drawIcon(ico_2_4ghz.posx, ico_2_4ghz.posy, ico_2_4ghz.data);   
            }
#endif
            break;
        case MODE_HID:
#ifdef ENABLE_GAME_CONTROLLER
            DBG_PRINT("\n ***** Starting game controller ***** \n");
            CONTROLLER_Init();
            buzPlayTone(2000,150);
#ifdef ENABLE_DISPLAY
            MPANEL_drawIcon(ico_usb.posx, ico_usb.posy, ico_usb.data);
#endif /* ENABLE_DISPLAY */
#endif /* ENABLE_GAME_CONTROLLER */
            LED_OFF;
            break;
        default:
            return;
    }
}

/**
 * @brief Periodic called function to display battery voltage
 * 
 * */
void appCheckBattery(void){
vires_t res;
    if(batteryReadVI(&res)){
        if(res.vbat < BATTERY_VOLTAGE_MIN && bat_low == NO){
            bat_low = YES;
            DBG_PRINT("!!Low battery !! (%dmV)\n", res.vbat);
#ifdef ENABLE_DISPLAY
            bat_low_tim = startTimer(TIMER_LOWBAT_TIME, SWTIM_AUTO_RELOAD, appToggleLowBatIco);
        }else{
            if(bat_low == YES){
                bat_low = NO;
                stopTimer(bat_low_tim);
                if(bat_low_ico_state == ON){
                    appToggleLowBatIco();
                }
            }
        }
        dro_bat.update(res.vbat/1000.0f);
        // [Ah] are given by the periodic call        
        bat_consumed += (float)(res.cur/(float)(3600/30));   //1h/30s
        dro_amph.update(bat_consumed / 1000.0f); 
        dro_ma.update(res.cur);
        LCD_Update();
#else
        }
#endif /* ENABLE_DISPLAY */
    }    
}

#ifdef ENABLE_DISPLAY
/**
 * @brief blink low battery icon
 * */
void appToggleLowBatIco(void){
    if(bat_low_ico_state == OFF){
        MPANEL_drawIcon(ico_low_bat.posx, ico_low_bat.posy, ico_low_bat.data);
        bat_low_ico_state = ON;
    }else{
        LCD_Fill(ico_low_bat.posx, ico_low_bat.posy, ico_low_bat.data->width, ico_low_bat.data->hight, BLACK);
        bat_low_ico_state = OFF;
    }
    LCD_Update();
}
#endif /* ENABLE_DISPLAY */
/**
 * @brief
 * 
 * */
void appInitEEPROM(uint8_t *dst){
uint8_t bind_flag;
    
    if(EEPROM_Read(EEPROM_BIND_FLAG, &bind_flag, 1) != 1){
        DBG_PRINT("Error reading EEPROM\n");
        return;
    }
        
    if(bind_flag == BIND_FLAG_VALUE){    
        if(NV_Restore(dst, EEPROM_SIZE) != EEPROM_SIZE){
            DBG_PRINT("Error reading EEPROM\n");
            return;
        }
        DBG_PRINT("Data loaded from EEPROM\n");
    }
}


/**
 * @brief Save the ram eeprom content to flash memory
 * Note: In order to save eeprom
 * 
 * */
void appSaveEEPROM(void){

    // FIXME: Really should implement CRC
    *((uint8_t*)eeprom_data+EEPROM_BIND_FLAG) = BIND_FLAG_VALUE;

    EEPROM_Write(EEPROM_ID_OFFSET, (uint8_t*)eeprom_data, EEPROM_SIZE);

    if(!EEPROM_Sync()){
        DBG_PRINT("!! Fail to sync EEPROM !!\n");
    }else{
        DBG_PRINT("EEPROM Saved\n");
    }
}

/**
 * @brief Application setup call
 * */
void setup(void){    

    state = STARTING;
    bat_low = NO;
    
    laser4Init();
    NV_Init();

#if defined(ENABLE_USART) && defined(ENABLE_DEBUG)
    usart_init();
    dbg_init(&pcom);
#endif

#if defined(ENABLE_VCOM) || defined(ENABLE_GAME_CONTROLLER)
    USB_DEVICE_Init();
    USB_DEVICE_RegisterCallback(HAL_PCD_SUSPEND_CB_ID, usbDisconnectCB, NULL);
    USB_DEVICE_RegisterCallback(HAL_PCD_RESUME_CB_ID, usbConnectCB, NULL);
#endif

#ifdef ENABLE_GAME_CONTROLLER
    CONTROLLER_Init();
#endif

#ifdef ENABLE_CLI
    con.init(&pcom, "laser4+ >");
    con.registerCommandList(laser4_commands);
    con.cls();
#endif    
    // Load eeprom data
    appInitEEPROM((uint8_t*)eeprom_data);
    // Set volume from stored value
    buzSetLevel(*((uint8_t*)eeprom_data + IDX_BUZ_VOLUME));
    // Play som random tone
    buzPlay(chime);   
    // Configure adc calibration values
    f2u_u tmp;
    tmp.u = (uint32_t)(eeprom_data[IDX_BAT_VOLTAGE_DIV] | (eeprom_data[IDX_BAT_VOLTAGE_DIV + 1] << 16));
    adcSetVdivRacio(tmp.f);
    tmp.u = (uint32_t)(eeprom_data[IDX_SENSE_RESISTOR] | (eeprom_data[IDX_SENSE_RESISTOR + 1] << 16));
    adcSetSenseResistor(tmp.f);

    /* Get battery voltage */
    DBG_PRINT("Battery voltage: %dmV\n", batteryGetVoltage());   

#ifdef ENABLE_DISPLAY

    MPANEL_print(8,8,&pixelDustFont, "V%u.%u.%u", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
    LCD_Update();
    delayMs(1000);
    LCD_Fill(8,8,64,pixelDustFont.h, BLACK);

    dro_bat.setIcon(&ico_volt);
    dro_amph.setIcon(&ico_amph);
	dro_bat.draw();
    dro_amph.draw();
    dro_ma.draw();
    startTimer(TIMER_BATTERY_TIME, SWTIM_AUTO_RELOAD, appCheckBattery);

    appCheckBattery();
#endif 
    // wait for melody to finish
    buzWaitEnd();    
    // Configure watchdog
    enableWatchDog(WATCHDOG_TIME);
}

/**
 * @brief Application main loop
 * */
void loop(void){

    switch(state & STATE_MASK){
        case MODE_MULTIPROTOCOL:
            multiprotocol_loop();
            break;

        case MODE_HID:
#ifdef ENABLE_GAME_CONTROLLER
            CONTROLLER_Process();
#endif
            break;

        case REQ_MODE_CHANGE:
            state = state >> STATE_BITS;
            changeMode(state);
#ifdef ENABLE_DISPLAY
            LCD_Update();
#endif            
            break;

        case STARTING:
            appReqModeChange(MODE_MULTIPROTOCOL);            
            break;

        default:
            break;
    }

#ifdef ENABLE_CLI
    con.process();
#endif

    processTimers();
    reloadWatchDog();
    //DBG_PIN_TOGGLE;
}


int main(void){
    setup();
    while(1){
        loop();
    }
    return 0;
}