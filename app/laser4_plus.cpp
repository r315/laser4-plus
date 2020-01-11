#include "app.h"
#include "multiprotocol.h"



static uint8_t mode;

#ifdef ENABLE_CONSOLE 
Console con;
#endif

void changeMode(void *ptr){
uint32_t new_mode = (uint32_t)ptr;

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
    mode = new_mode;    
}

void setup(void){
    GPIO_ENABLE;
    DBG_PIN_INIT;
    CC25_CS_INIT;
    HW_BIND_BUTTON_INIT;
    LED_INIT;

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
    con.print("\b\b\b\b\b\b\b\b\b\b");
    con.registerCommandList(laser4_commands);
#endif

    NV_Init();

    //changeMode((void*)HID);
    changeMode((void*)MULTIPROTOCOL);
    enableWatchDog(3000);   // 3 seconds
}

void loop(void){

    switch(mode){
        case MULTIPROTOCOL:
            multiprotocol_loop();
            break;
        case HID:
#ifdef ENABLE_GAME_CONTROLLER
            CONTROLLER_Process();
#endif
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