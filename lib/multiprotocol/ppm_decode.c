#include "multiprotocol.h"
#include "board.h"

static volatile uint16_t ppm_data[MAX_CHN_NUM];
static void (*ppmFrameCB)(volatile uint16_t *, uint8_t);

static void ppm_decode(void);

uint16_t ppm_tx(void){
    ppmOut((uint16_t *)ppm_data);
    return 9000;
}

/**
 * @brief
 * */
uint16_t *ppm_getData(void){
    return (uint16_t*)ppm_data;
}

/**
 * @brief Configure callback for PPM input pin interrupt
 * 
 * @param cb : callback function
 * 
 * */
void ppm_setCallBack(void(*cb)(volatile uint16_t*, uint8_t)){
     if(cb == NULL){
        return;
    }
    gpioRemoveInterrupt(HW_PPM_INPUT_PORT, HW_PPM_INPUT_PIN);    
    ppmFrameCB = cb;
    gpioAttachInterrupt(HW_PPM_INPUT_PORT, HW_PPM_INPUT_PIN, 0, ppm_decode);        
}

/**
 * @brief PPM_decode from multiprotocol project.
 * This function is calledfrom PPM pin input interrupt handler
 * */
RAM_CODE static void ppm_decode(void){	
    static int8_t chan = 0, bad_frame = 1;
    static uint16_t Prev_TCNT1 = 0;
    uint16_t Cur_TCNT1;
    // Capture current Timer value
    Cur_TCNT1 = TIMER_BASE->CNT - Prev_TCNT1;
    if(Cur_TCNT1 < PPM_MIN_PERIOD){
        bad_frame = 1;					// bad frame
    }else if(Cur_TCNT1 > PPM_MAX_PERIOD){
        //start of frame
        if(chan >= MIN_PPM_CHANNELS){
            //DBG_PIN_TOGGLE;                
            ppmFrameCB(ppm_data, chan);
        }
        chan = 0;						// reset channel counter
        bad_frame = 0;
    }else if(bad_frame == 0){			// need to wait for start of frame
        //servo values between 800us and 2200us will end up here
        ppm_data[chan] = Cur_TCNT1;
        if(chan++ >= MAX_PPM_CHANNELS)
            bad_frame = 1;		// don't accept any new channels
    }
    Prev_TCNT1 += Cur_TCNT1;
}