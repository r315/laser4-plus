#include "app.h"
#include "multiprotocol.h"



volatile uint8_t state;

tone_t chime[] = {
    {1000,200},
    {2000,300},
    {500,100},
    {0,0}
};

#ifdef ENABLE_CONSOLE 
Console con;
#endif

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
            DBG_PRINT("Starting Multiprotocol\n");
            multiprotocol_setup();
            playTone(500,100);
            break;
        case MODE_HID:
#ifdef ENABLE_GAME_CONTROLLER
            DBG_PRINT("Starting game controller\n");
            CONTROLLER_Init();
            LED_OFF;
            playTone(400,100);
            break;
#endif
        case MODE_PPM:
            playTone(600, 100);
            break;
        default:
            return;
    }
}

void setup(void){    

    state = STARTING;
    
    laser4Init();
    NV_Init();

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
        
    reqModeChange(MODE_MULTIPROTOCOL);

    playMelody(chime);
    // wait for melody to finish
    delayMs(1500);
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

        case MODE_PPM:
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

    #ifdef ENABLE_CONSOLE
    con.process();
    #endif
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