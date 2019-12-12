
#include "app.h"

#ifdef USE_FREERTOS
void app_setup(void){
    GPIO_ENABLE;
    DBG_LED_INIT;
    DBG_PIN_INIT;
    //MCO_EN;
}

void app_loop(void *ptr){
    DBG_LED_TOGGLE;
    DBG_PIN_TOGGLE;
    vTaskDelay(200);

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

