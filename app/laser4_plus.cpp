#include "app.h"
#include "multiprotocol.h"

#ifdef ENABLE_CONSOLE 
Console con;
#endif

void setup(void){
    BOARD_Init();
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

    multiprotocol_setup();

    enableWatchDog(2000);
}

void loop(void){
    multiprotocol_loop();
    con.process();
    reloadWatchDog();
}


int main(void){
    setup();
    while(1){
        loop();
    }
    return 0;
}