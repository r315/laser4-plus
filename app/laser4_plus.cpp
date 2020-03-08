#include "app.h"
#include "multiprotocol.h"
#include "usb_device.h"


volatile uint8_t state;

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

/**
 * EEPROM ram copy
 * */
uint16_t eeprom_data[EEPROM_SIZE / 2] = {
    (uint16_t)DEFAULT_ID, (uint16_t)(DEFAULT_ID>>16),
    (uint16_t)BAT_VOLTAGE_DIV,(uint16_t)(BAT_VOLTAGE_DIV>>16),
    CHANNEL_MAX_100, CHANNEL_MIN_100,
    CHANNEL_MAX_125, CHANNEL_MIN_125, CHANNEL_SWITCH, 
    PPM_MAX_100, PPM_MIN_100, PPM_DEFAULT_VALUE,
};


/**
 * */
void appSaveEEPROM(void){
    EEPROM_Write(EEPROM_ID_OFFSET, (uint8_t*)eeprom_data, EEPROM_SIZE);

    if(!EEPROM_Sync()){
        DBG_PRINT("!! Fail to sync EEPROM !!\n");
    }else{
        DBG_PRINT("EEPROM Saved\n");
    }
}

/**
 * @brief
 * */
uint8_t getCurrentMode(void){
    return state;
}

void usbConnectCB(void *ptr){
   reqModeChange(MODE_HID);
#if defined(ENABLE_DEBUG) && defined(ENABLE_VCOM)
    dbg_init(&vcom);
#endif

#ifdef ENABLE_CLI
    con.setOutput(&vcom);
#endif
}

void usbDisconnectCB(void *ptr){
    reqModeChange(MODE_MULTIPROTOCOL);
#if defined(ENABLE_DEBUG) && defined(ENABLE_USART)
    dbg_init(&pcom);
#endif
 
#ifdef ENABLE_CLI
    con.setOutput(&pcom);
#endif
}

void reqModeChange(uint8_t new_mode){
uint8_t cur_state = state & STATE_MASK;
    // Do nothing if requesting the current mode
    if(cur_state == new_mode){
        return;
    }
    // Request in progress, if same return
    if(cur_state == REQ_MODE_CHANGE){
        if((state >> STATE_BITS) == cur_state){
            return;
        } 
    }
    // set the requested mode by overlaping the previous
    state = (new_mode << STATE_BITS) | REQ_MODE_CHANGE;
}

static void changeMode(uint8_t new_mode){
    switch(new_mode){
        case MODE_MULTIPROTOCOL:
            DBG_PRINT("\n ***** Starting Multiprotocol *****\n");
            multiprotocol_setup();
            buzPlayTone(500,100);
            break;
        case MODE_HID:
#ifdef ENABLE_GAME_CONTROLLER
            DBG_PRINT("\n ***** Starting game controller ***** \n");
            CONTROLLER_Init();
#endif
            LED_OFF;
            buzPlayTone(400,100);
            break;
        default:
            return;
    }
}

void checkBattery(void){
uint32_t vbat;
    if(batteryReadVoltage(&vbat)){
        DBG_PRINT("Battery voltage: %dmV\n", vbat);
    }    
}


void init_eeprom_data(uint8_t *dst){
uint8_t bind_flag;
    
    if(EEPROM_Read(EEPROM_BIND_FLAG, &bind_flag, 1) != 1){
        DBG_PRINT("Error reading EEPROM\n");
        return;
    }
        
    if(bind_flag == 0xF0){    
        if(NV_Restore(dst, EEPROM_SIZE) != EEPROM_SIZE){
            DBG_PRINT("Error reading EEPROM\n");
            return;
        }
        DBG_PRINT("Data loaded from EEPROM\n");
    }
}

void setup(void){    

    state = STARTING;
    
    laser4Init();
    NV_Init();

#ifdef ENABLE_USART
    usart_init();
#endif  

#if defined(ENABLE_VCOM) || defined(ENABLE_GAME_CONTROLLER)
    USB_DEVICE_Init();
    USB_DEVICE_RegisterCallback(HAL_PCD_SUSPEND_CB_ID, usbDisconnectCB, NULL);
    USB_DEVICE_RegisterCallback(HAL_PCD_RESUME_CB_ID, usbConnectCB, NULL);
#endif

#ifdef ENABLE_GAME_CONTROLLER
    CONTROLLER_Init();
#endif

#ifdef ENABLE_DEBUG
    dbg_init(&pcom);
#endif

#ifdef ENABLE_CLI
    con.init(&pcom, "laser4+ >");
    con.registerCommandList(laser4_commands);
    con.cls();
#endif    
        
    reqModeChange(MODE_MULTIPROTOCOL);

    buzPlay(chime);   
    
    // Load eeprom data
    init_eeprom_data((uint8_t*)eeprom_data);
    // Configure adc calibration values
    f2u_u tmp;
    tmp.u = (uint32_t)(eeprom_data[IDX_BAT_VOLTAGE_DIV] | (eeprom_data[IDX_BAT_VOLTAGE_DIV + 1] << 16));
    adcSetVdivRacio(tmp.f);

    /* Get battery voltage */
    DBG_PRINT("Battery voltage: %dmV\n", batteryGetVoltage());
    // wait for melody to finish
    buzWaitEnd();
    startTimer(30000, SWTIM_AUTO, checkBattery);
    // 3 seconds watchdog
    enableWatchDog(3000);
}

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
            break;

        case STARTING:
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