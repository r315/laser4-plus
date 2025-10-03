#include "multiprotocol.h"
#include "board.h"

#if defined(ENABLE_PPM)
static uint8_t chan, bad_frame, nch;
static uint16_t prev_tick;

/**
 * @brief ppm_data contains ppm frame in units of 0.5us
 * using 0.5us units give more precision when maping to servo data.
 * Because of this it makes sense for making ppm_data static here
 */
static uint16_t ppm_data[MIN_PPM_CHANNELS];
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
 * @brief Get number of channels of last
 * captured ppm frame
 */
uint8_t ppm_nchannel_get(void)
{
    return nch;
}

/**
 * @brief Get ppm channel pulse in us
 * @param ch Channel for which value should be returned
 * @return pulse duration in us
 */
uint16_t ppm_value_get(uint8_t ch)
{
    return (ch < nch) ? ppm_data[ch] : 0;
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
            nch = chan;
            ppm_frame_ready();
        }
        chan = 0;						// reset channel counter
        bad_frame = 0;
    }else if(bad_frame == 0){			// need to wait for start of frame
        //servo values between 800us and 2200us will end up here
        ppm_data[chan] = TICKS_TO_US(cur_tick);
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
 * @return pointer to captured ppm ddata
 * */
void ppm_init(void(*cb)(void))
{
    if(cb != NULL){
        gpioRemoveInterrupt(HW_PPM_INPUT_PORT, HW_PPM_INPUT_PIN);
        ppm_frame_ready = cb;
        prev_tick = 0;
        chan = 0;
        nch = 0;
        bad_frame = 1;
        gpioAttachInterrupt(HW_PPM_INPUT_PORT, HW_PPM_INPUT_PIN, 0, ppm_handler);
    }
}

void ppm_sim_handler(void)
{
    if(ppm_frame_ready){
        nch = MIN_PPM_CHANNELS;
        ppm_frame_ready();
    }
}

void ppm_sim_set_channel_data(uint8_t ch, uint16_t data)
{
    if(ch < MAX_PPM_CHANNELS){
        ppm_data[ch] = data;
    }
}
#endif