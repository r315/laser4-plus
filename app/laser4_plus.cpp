#include "app.h"
#include "multiprotocol.h"

#ifdef ENABLE_CONSOLE 
Console con;
#endif

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

    // Configure PPM pin input pull-up
    HW_PPM_INPUT_INIT;
    // Attach falling edge interrupt
    BOARD_GPIO_Interrupt(GPIOB, HW_PPM_INPUT_PIN, 0, PPM_decode);

    NV_Init();

    multiprotocol_setup();

    enableWatchDog(3000);   // 3 seconds
}

void loop(void){
    //multiprotocol_loop();
    #ifdef ENABLE_CONSOLE
    con.process();
    #endif
    #ifdef ENABLE_GAME_CONTROLLER
    CONTROLLER_Process();
    #endif
    multiprotocol_loop();
    reloadWatchDog();
}


int main(void){
    setup();
    while(1){
        loop();
    }
    return 0;
}