#include <stdint.h>
#include "board.h"

void app_setup(void){
    GPIO_ENABLE;
    DBG_LED_INIT;
    //MCO_EN;
}

void app_loop(void *ptr){

}

void app_main(void){
volatile uint32_t i;

    GPIO_ENABLE;
    DBG_LED_INIT;
    //MCO_EN;

    while(1){        
        DBG_LED_TOGGLE;
        for(i = 0x20000; i > 0; i--);
    }
}
