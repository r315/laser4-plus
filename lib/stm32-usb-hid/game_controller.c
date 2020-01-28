#include "board.h"
#include "game_controller.h"
#include "usbd_hid_if.h"
#include "math.h"
#include "multiprotocol.h"

//#define DEMO_CONTROLLER

static controller_t laser4;
volatile uint16_t ppm_data[MIN_PPM_CHANNELS], last_tim;
volatile uint32_t gflags;
static uint8_t *channel_map;

#ifdef DEMO_CONTROLLER
#undef ENABLE_PPM
static float angle = 0;
#else
static int16_t map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max){
  return ((x - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min;
}

static void setPpmFlag(uint8_t chan){
    SET_PPM_FRAME;    
}
#endif

RAM_CODE void CONTROLLER_Process(void){

#if defined(DEMO_CONTROLLER)
    angle += 0.1;
    SET_PPM_FRAME;
    delayMs(20);
#endif

    if(IS_PPM_FRAME_READY)
    {            
#if !defined(DEMO_CONTROLLER)
        uint8_t i;
        uint8_t *data = (uint8_t*)&laser4.pitch;
        for(i = 0; i < MIN_PPM_CHANNELS; i++){
            //PAUSE_CAPTURE;
            uint16_t val = ppm_data[i];
            //RESUME_CAPTURE;

            if(val < laser4.min_pulse || val > laser4.max_pulse){
                val = (laser4.max_pulse + laser4.min_pulse) / 2;
            }
            val = map(val, laser4.min_pulse, laser4.max_pulse, LOGICAL_MINIMUM, LOGICAL_MAXIMUM);
            *(data + (channel_map[i] * 2)) = val;
            *(data + 1 + (channel_map[i] * 2)) = val >> 8;
        }
        
        int16_t diff = ENC_TIM->CNT - last_tim;
        if(diff != 0){
            uint16_t tmp = laser4.aux2;
            tmp += diff*10;
            if(tmp > CHANNEL_MAX_100){
                tmp = CHANNEL_MAX_100;
            }else if(tmp < CHANNEL_MIN_100){
                tmp = CHANNEL_MIN_100;
            }
            laser4.aux2 = tmp;        
            last_tim += diff;
        }
        laser4.buttons = HW_READ_SWITCHES;
#else
        //float t = angle * 0.15915f; // Normalize t = x/2pi - floor(x/2pi)
        //t = t - (int)t;
        //float sine = 20.785f * (t - 0.0f) * (t - 0.5f) * (t - 1.0f);
        //float cosine = 20.785f * (t + 0.25f) * (t - 0.25f) * (t - 0.75f);

        laser4.roll = (LOGICAL_MAXIMUM/2) + sin(angle) * (LOGICAL_MAXIMUM/2);
        laser4.pitch = (LOGICAL_MAXIMUM/2) + cos(angle) * (LOGICAL_MAXIMUM/2);
        
        laser4.throttle = laser4.roll;
        laser4.yaw = laser4.pitch;
#endif  
        CLR_PPM_FRAME;
        USBD_HID_SendReport((uint8_t*)&laser4, REPORT_SIZE);
    }
}


/**
 *  Code for PPM input PB5, TIM3_CH1 
 * */
void CONTROLLER_Init(void){

    laser4.roll = LOGICAL_MAXIMUM;
    laser4.pitch = LOGICAL_MAXIMUM/2;
    laser4.throttle = LOGICAL_MINIMUM;
    laser4.yaw = LOGICAL_MAXIMUM/2;
    laser4.aux1 = LOGICAL_MAXIMUM/2;
    laser4.aux2 = LOGICAL_MAXIMUM/2;
    laser4.buttons = 0;
    laser4.max_pulse = PPM_MAX_PERIOD;
    laser4.min_pulse = PPM_MIN_PERIOD;

    channel_map = CH_AETR;

#if defined(ENABLE_PPM)
    multiprotocol_frameReadyAction(ppm_data, setPpmFlag);
#elif defined(ENABLE_PWM)
    RCC->APB1ENR |= (1 << 0);   // TIM2EN

    TIM2->CR1 = 0;              // Stop counter

    TIM2->PSC = SystemCoreClock/1000000;      // 1Mhz
    TIM2->CCMR1 = 0x0101;       // CC1-2S = 01, ch1-2 mapped to TI1,TI2 
    TIM2->CCMR2 = 0x0101;       // CC3-4S = 01, map to TI1 
    TIM2->CCER  = 0x1111;       // CC1E, CC1; Configure all channels as input, Capture on rising edge

    TIM2->DIER = (0x0f << 1);   // Enable interrupt on capture for all channels
    NVIC_EnableIRQ(TIM2_IRQn);  // Enable timer 2 interupt
    TIM2->CR1 |= (1 << 0);      // Start counter
#endif /* ENABLE_PPM */
}

#if defined(ENABLE_PWM)
/**
 * Handles a radio channel with the cirrespondent captured value from interrupt.
 * Hw capture flag is cleared when the correspondent regiter is read.
 * This is a synchronous algorithm, since the ready flag is set only if all channels were 
 * measured.  
 * */
RAM_CODE void HandleChannel(volatile uint16_t *dst, volatile uint16_t *ccr, uint8_t ch){

    // check if overcapture occurred
    if(TIM2->SR & (1 << (8 + ch))){ 
        // if set ignore capture and set capture for rising edge 
        TIM2->CCER &= ~(TIM_CAP_POL(ch));
        TIM2->SR &= ~((1 << (8 + ch)) | (1 << ch));
        return;
    }

    // If interrupt was from a rising edge, save capture value on the remote channel
    // and change the polarity of the capturing edge for the give channel
    if(!(TIM2->CCER & TIM_CAP_POL(ch))){
        // First capture, save it on buffer 
        *dst =  *ccr;
        TIM2->CCER |= TIM_CAP_POL(ch);
        if(ch == 1) ready = 0;
    }else{            
        // If caused by falling edge, calculate the pulse width and save it on the 
        // remote channel.
        *dst =  (*ccr > *dst) ? *ccr - *dst : (0xFFFF - *dst) + *ccr;
        TIM2->CCER &= ~(TIM_CAP_POL(ch));
        // The calculated pulse value is stored on the lower half of the buffer
        *(dst-4) = *dst;
        if(ch == 4) ready = 1;
    }    
}

/**
 * Every capture of any channel will generate an interrupt on capture event (rising or falling edge), 
 * then the handler will evaluate which channel caused the interrupt and call the channel handler
 * with the correspondent radio channel buffer.
 * As each remote channel requires two captures we send the address of
 * the position to store the first captured value, then tha handler decides were to store
 * the pulse value
 * */
RAM_CODE void TIM2_IRQHandler(void){
    if(TIM2->SR & (1 << 1)){
        HandleChannel(&remote_channels[4], (uint16_t*)&TIM2->CCR1, 1);
    }
    
    if(TIM2->SR & (1 << 2)){
        HandleChannel(&remote_channels[5], (uint16_t*)&TIM2->CCR2, 2);
    }

    if(TIM2->SR & (1 << 3)){
        HandleChannel(&remote_channels[6], (uint16_t*)&TIM2->CCR3, 3);
    }

    if(TIM2->SR & (1 << 4)){
        HandleChannel(&remote_channels[7], (uint16_t*)&TIM2->CCR4, 4);
    }  
}
#endif /* ENABLE_PWM */