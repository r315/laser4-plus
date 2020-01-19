#include "app.h"
#include "multiprotocol.h"



static uint8_t mode;
volatile uint8_t state;

#ifdef ENABLE_CONSOLE 
Console con;
#endif

void reqModeChange(void *ptr){
    mode = (uint32_t)ptr;
    state = REQ_MODE_CHANGE;
}

static void changeMode(uint8_t new_mode){
    switch(new_mode){
        case MULTIPROTOCOL:
            DBG_PRINT("Starting Multiprotocol\n");
            multiprotocol_setup();
            break;
        case HID:
#ifdef ENABLE_GAME_CONTROLLER
            DBG_PRINT("Starting game controller\n");
            CONTROLLER_Init();
            break;
#endif
        default:
            return;
    }
    mode = state = new_mode;
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
        
    reqModeChange((void*)MULTIPROTOCOL);
    enableWatchDog(3000);   // 3 seconds
}

void loop(void){

    switch(state){
        case MULTIPROTOCOL:
            multiprotocol_loop();
            break;
        case HID:
#ifdef ENABLE_GAME_CONTROLLER
            CONTROLLER_Process();
#endif
            break;
        case REQ_MODE_CHANGE:
            changeMode(mode);
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