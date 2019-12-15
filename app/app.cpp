
#include "app.h"

static Console con;

#ifdef USE_FREERTOS

void app_setup(void){
    BOARD_Init();
    //MCO_EN;
    CDC_Init();

    con.init(&vcom, "laser4+ >");
    con.registerCommandList(laser4_commands);
}

void app_loop(void *ptr){
//static int i;

    DBG_LED_TOGGLE;
    //DBG_PIN_TOGGLE;
    vTaskDelay(10);

    con.process();
}
#else
void app_main(void){
volatile uint32_t i;

    GPIO_ENABLE;
    DBG_LED_INIT;
    DBG_PIN_INIT;
    //MCO_EN;

    while(1){        
        DBG_LED_TOGGLE;
        for(i = 0x20000; i > 0; i--);
    }
}

void SysTick_Handler(void){

    DBG_PIN_TOGGLE;
}
#endif

