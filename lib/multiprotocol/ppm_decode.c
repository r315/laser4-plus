#include "multiprotocol.h"
#include "board.h"


#define PPM_TX_INTERVAL     20000 /* ms */

#if defined(ENABLE_PPM)
/**
 * @brief ppm_data contains ppm frame in units of 0.5us
 * using 0.5us units give more precision when maping to servo data.
 * Becose of this it makes sense that ppm_data bing static here
 */
static volatile uint16_t ppm_data[MAX_CHN_NUM];
static void (*ppmFrameCB)(const uint16_t *, uint8_t);

/**
 * @brief
 * @param radio
 * @return time in ms for next call
 */
uint16_t ppm_tx(struct radio *radio)
{
#ifdef ENABLE_PPM_OUTPUT
    ppmOut(radio->channel_data, 4); // TODO: replace 4 by radio->nchannels
#else
    (void)radio;
#endif
    return PPM_TX_INTERVAL;
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

/**
 * @brief Configure callback for PPM input pin interrupt
 *
 * @param cb : callback function
 *
 * */
void ppm_setCallBack(void(*cb)(const uint16_t*, uint8_t)){
    if(cb == NULL){
       return;
   }
   gpioRemoveInterrupt(HW_PPM_INPUT_PORT, HW_PPM_INPUT_PIN);
   ppmFrameCB = cb;
   gpioAttachInterrupt(HW_PPM_INPUT_PORT, HW_PPM_INPUT_PIN, 0, ppm_decode);
}
#endif