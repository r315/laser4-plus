#include "app.h"
#include "multiprotocol.h"



volatile uint8_t state;

#ifdef ENABLE_CONSOLE 
Console con;
#endif

void reqModeChange(uint8_t new_mode){
    state = (new_mode << STATE_BITS) | REQ_MODE_CHANGE;
}

static void changeMode(uint8_t new_mode){
    switch(new_mode){
        case MODE_MULTIPROTOCOL:
            DBG_PRINT("Starting Multiprotocol\n");
            multiprotocol_setup();
            break;
        case MODE_HID:
#ifdef ENABLE_GAME_CONTROLLER
            DBG_PRINT("Starting game controller\n");
            CONTROLLER_Init();
            LED_OFF;
            break;
#endif
        default:
            return;
    }
}

void setup(void){    

    laser4Init();

#ifdef ENABLE_USART
    usart_init();
#endif  
    //MCO_EN;
#ifdef ENABLE_VCOM
    CDC_Init();
#endif

#ifdef ENABLE_GAME_CONTROLLER
    HID_Init();
    CONTROLLER_Init();
#endif

#ifdef ENABLE_CONSOLE
    con.init(IO_CHAR, "laser4+ >");
    con.registerCommandList(laser4_commands);
    con.cls();
#endif

    NV_Init();
        
    reqModeChange(MODE_MULTIPROTOCOL);
    enableWatchDog(3000);   // 3 seconds
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

        default:
            break;
    }

    #ifdef ENABLE_CONSOLE
    con.process();
    #endif
    reloadWatchDog();
}


int main(void){
    setup();
    while(1){
        loop();
    }
    return 0;
}