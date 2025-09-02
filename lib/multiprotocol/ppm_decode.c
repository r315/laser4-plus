#include "multiprotocol.h"
#include "board.h"

#if defined(ENABLE_PPM)
static int8_t chan = 0, bad_frame = 1;
static uint16_t prev_tick;

/**
 * @brief ppm_data contains ppm frame in units of 0.5us
 * using 0.5us units give more precision when maping to servo data.
 * Because of this it makes sense for making ppm_data static here
 */
static uint16_t ppm_data[MAX_CHN_NUM];
static void (*ppm_frame_ready)(void);

/**
 * @brief
 * @param radio
 * @return time in ms for next call
 */
uint16_t ppm_tx(struct radio *radio)
{
#ifdef ENABLE_PPM_OUTPUT
    ppmOut(radio->channel_data, radio->nchannels);
#else
    (void)radio;
#endif
    return PPM_TX_CALLBACK_INTERVAL;
}

/**
 * @brief
 *
 * @param buf
 * @param chan
 */
void ppm_channel_data_get(const uint16_t **buf, uint8_t *chan)
{
    *buf = ppm_data;
    *chan = MIN_PPM_CHANNELS;
}

/**
 * @brief  Interrupt handler for falling edge of ppm input pin
 * Channel data is obtained by measuring time between falling edges
 * */
RAM_CODE static void ppm_handler(void){
    uint16_t cur_tick;
    // Get current ticks
    cur_tick = ticksGetElapsed(prev_tick);

    if(cur_tick < US_TO_TICKS(PPM_MIN_PERIOD)){
        bad_frame = 1;					// bad frame
    }else if(cur_tick > US_TO_TICKS(PPM_MAX_PERIOD)){
        //start of frame
        if(chan >= MIN_PPM_CHANNELS){
            ppm_frame_ready();
        }
        chan = 0;						// reset channel counter
        bad_frame = 0;
    }else if(bad_frame == 0){			// need to wait for start of frame
        //servo values between 800us and 2200us will end up here
        ppm_data[chan] = cur_tick;
        if(chan++ >= MAX_PPM_CHANNELS)
            bad_frame = 1;		        // don't accept any new channels
    }
    prev_tick += cur_tick;
}

/**
 * @brief Configure callback for PPM input pin interrupt
 *
 * @param cb : callback function
 *
 * */
void ppm_setCallBack(void(*cb)(void)){
    if(cb == NULL){
       return;
   }
   gpioRemoveInterrupt(HW_PPM_INPUT_PORT, HW_PPM_INPUT_PIN);
   ppm_frame_ready = cb;
   gpioAttachInterrupt(HW_PPM_INPUT_PORT, HW_PPM_INPUT_PIN, 0, ppm_handler);
}
#endif