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

    BOARD_Init();

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

    HW_PPM_INPUT_INIT;
    BOARD_GPIO_Interrupt(GPIOB, HW_PPM_INPUT_PIN, 0, PPM_decode);

    NV_Init();

    multiprotocol_setup();

    enableWatchDog(2000);
}

void loop(void){
    multiprotocol_loop();
    #ifdef ENABLE_CONSOLE
    con.process();
    #endif
}


int main(void){
    setup();
    while(1){
        loop();
    }
    return 0;
}