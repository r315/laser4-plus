
#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/interrupt.h>


 /*
           ___  ___ 
     NC - |1  \/  8| - VCC
  PPM_I - |2      7| - NC
  PPM_O - |3      6| - AUX2
    GND - |4______5| - AUX1


Example of single channel 1500us PPM signal

    200us     1300us     200us   19800us
    |<->|<-            ->|<->|<-        ->|
     ___                  ___              _
    |   |                |   |            |
    |   |                |   |            |
____|   |________________|   |___  .... __|

*/

#define IS_LOW(_X)          !(PINB & _X)
#define IS_HIGH(_X)         (PINB & _X)

#define AUX1_PIN            (1 << 0)    // PB0
#define AUX2_PIN            (1 << 1)    // PB1
#define PPMI_PIN            (1 << 3)    // PB3
#define PPMO_PIN            (1 << 4)    // PB4
#define PPMOn_PIN           (1 << 2)    // PB2

#define STOP_TIMER          TCCR0B = 0; TCNT0 = 0;
#define START_TIMER         TCCR0B = (1 << CS02) | (0 << CS01) | (0 << CS00);  // clk/256 => 26.6us
#define SET_PULSE(_X)       OCR0A = _X  

#define PPM_PIN_HIGH        PORTB |= PPMO_PIN; PORTB &= ~PPMOn_PIN
#define PPM_PIN_LOW         PORTB &= ~PPMO_PIN; PORTB |= PPMOn_PIN

#define PPM_MIN_PULSE       0x27    // 1000us
#define PPM_CENTER_PULSE    0x35    // 1500us
#define PPM_MAX_PULSE       0x4A    // 1984us

#define AUX_CHANNELS        2
#define NUM_CHANNELS        4
#define AUX_MAX_POSITION    3

#define AUX1_NUM            0
#define AUX2_NUM            1

#define HW_CFG                                        \
{                                                     \
    DDRB = PPMO_PIN | PPMOn_PIN;                      \
    PORTB = AUX2_PIN | AUX1_PIN;   /*P-U*/            \
    CLKPR = (1<<CLKPCE); /* Change enable */          \
	CLKPR = 0;			 /* CLK Prescaler = 1 */      \
    TCCR0A = (1 << WGM01); /* Clear Timer on Match */ \
    TIMSK0 = (1 << OCIE0A);                           \
}

enum{
    NORMAL = 0,
    PRESSED,
    HOLD,
    LOW,
    HIGH
};

typedef struct _auxch{
    uint8_t state;
    uint8_t pin;
    uint8_t pos;
}AuxChannel;

volatile AuxChannel aux_channels[AUX_CHANNELS];

const uint8_t ppm_pulses[] = {
    PPM_MIN_PULSE, 
    PPM_CENTER_PULSE,
    PPM_MAX_PULSE,
};


volatile uint8_t auxcount;
static uint8_t channelcount;
volatile uint8_t aux_channels_pulses[AUX_CHANNELS];

/**
 * @brief Configure watchdog to perform a system reset
 **/
void WDT_Init(void){
  MCUSR &= ~(1<<WDRF);            // Clear Reset Flag   
  WDTCR |= (1<<WDE) | (1<<WDCE);  // Enable register change
  WDTCR  = (1<<WDE) 
          |(0<<WDP2)
          |(0<<WDP1) 
          |(1<<WDP0);  // Perform system Reset and timeout in 32ms
}

/**
 * @brief disable Watch dog to avoid reset loop
 **/
void WDT_Disable(void){
  MCUSR &= ~(1<<WDRF);    
  WDTCR |= (1<<WDCE) | (1<<WDE);
  WDTCR = 0;
}

int main(void){

    WDT_Disable();

    aux_channels[0].state = NORMAL;
    aux_channels[0].pos = 0;
    aux_channels[0].pin = AUX1_PIN;
    
    aux_channels[1].state = NORMAL;
    aux_channels[1].pos = 1;
    aux_channels[1].pin = AUX2_PIN;

    auxcount = AUX_CHANNELS;
    channelcount = NUM_CHANNELS + 1;

    HW_CFG;    

    sei();

    WDT_Init();   

    SET_PULSE(aux_channels_pulses[0]);
    PPM_PIN_HIGH;

    // initial sync, as the signal is inverted
    // we wait for rising edge
    while(IS_HIGH(PPMI_PIN));
    while(IS_LOW(PPMI_PIN));
    // The PPM frame with 4 channels from the remote
    // is 6.5ms, havig a delay we should 
    // exit right after the last pulse
    _delay_ms(7);
    wdt_reset();

    while(1){
        channelcount--;
        // wait for falling edge of the first ppm pulse 
        while(IS_HIGH(PPMI_PIN));
        PPM_PIN_LOW;
        if(channelcount == 0){
            // last pulse from remote
            // start timer to generate the aux chanels pulses
            START_TIMER;
            channelcount = NUM_CHANNELS + 1;
        }            
        while(IS_LOW(PPMI_PIN));
        PPM_PIN_HIGH;
    }
return 0;
}

// Check if aux switch was pushed and cycle
// through the positions 
void checkAux(volatile AuxChannel *aux){
    if(IS_LOW(aux->pin)){
        if(aux->state == NORMAL){
            aux->state = PRESSED;
            aux->pos++;
            if(aux->pos == AUX_MAX_POSITION){
                aux->pos = 0;
            }            
        }        
    }else if(aux->state == PRESSED){
        aux->state = NORMAL;
    }
}


ISR (TIM0_COMPA_vect)
{    
uint8_t cycles;
    PPM_PIN_LOW;

    if((--auxcount) == 0){
        STOP_TIMER;

        for(auxcount = 0; auxcount < AUX_CHANNELS; auxcount++){
            checkAux(&aux_channels[auxcount]);
            aux_channels_pulses[auxcount] = ppm_pulses[aux_channels[auxcount].pos];
        }  
        
        auxcount = AUX_CHANNELS;        
        SET_PULSE(aux_channels_pulses[0]);        
        wdt_reset();
        cycles = 215;      
    }
    else{
        SET_PULSE(aux_channels_pulses[AUX_CHANNELS - auxcount]);
        cycles = 235;
    }

    while(cycles--){
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop"); 
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        //asm("nop");       
    }    
    PPM_PIN_HIGH;
}

ISR (TIM0_COMPB_vect)
{
   
}