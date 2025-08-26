#include "multiprotocol.h"
#include "board.h"

#if defined(ENABLE_PPM)
static volatile uint16_t ppm_data[MAX_CHN_NUM]; // TODO: move this to radio structure?
static void (*ppmFrameCB)(volatile uint16_t *, uint8_t);

static void ppm_decode(void);

/**
 * @brief
 * @param radio
 * @return
 */
uint16_t ppm_tx(struct radio *radio)
{
#ifdef ENABLE_PPM_OUTPUT
    ppmOut((uint16_t *)radio->channel_data);
#else
    (void)radio;
#endif
    return 9000;
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
#endif