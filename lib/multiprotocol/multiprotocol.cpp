
/*********************************************************
					Multiprotocol Tx code
               by Midelic and Pascal Langer(hpnuts)
	http://www.rcgroups.com/forums/showthread.php?t=2165676
    https://github.com/pascallanger/DIY-Multiprotocol-TX-Module/edit/master/README.md

	Thanks to PhracturedBlue, Hexfet, Goebish, Victzh and all protocol developers
				Ported  from deviation firmware 

 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "app.h"
#include "multiprotocol.h"
#include "FrSkyDVX_Common.h"

#define TIMER_BASE TIM1
//Personal config file
#if defined(USE_MY_CONFIG)
#include "_MyConfig.h"
#endif

radio_t radio;

static uint8_t Update_All(void);
static void modules_reset(void);
//static void update_serial_data(void);
static void update_channels_aux(void);
static void protocol_init(void);
static void update_led_status(void);
int16_t map16b( int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max);
static uint32_t random_id(uint16_t address, uint8_t create_new);

//Channel mapping for protocols
uint8_t CH_AETR[]={AILERON, ELEVATOR, THROTTLE, RUDDER, CH5, CH6, CH7, CH8, CH9, CH10, CH11, CH12, CH13, CH14, CH15, CH16};
uint8_t CH_TAER[]={THROTTLE, AILERON, ELEVATOR, RUDDER, CH5, CH6, CH7, CH8, CH9, CH10, CH11, CH12, CH13, CH14, CH15, CH16};
//uint8_t CH_RETA[]={RUDDER, ELEVATOR, THROTTLE, AILERON, CH5, CH6, CH7, CH8, CH9, CH10, CH11, CH12, CH13, CH14, CH15, CH16};
uint8_t CH_EATR[]={ELEVATOR, AILERON, THROTTLE, RUDDER, CH5, CH6, CH7, CH8, CH9, CH10, CH11, CH12, CH13, CH14, CH15, CH16};

void multiprotocol_setup(void){   

    // Read status of bind button
    if(IS_HW_BIND_BUTTON_PRESSED)
    {
        BIND_BUTTON_FLAG_on;	// If bind button pressed save the status
        BIND_IN_PROGRESS;		// Request bind
        DBG_PRINT("Bind button pressed\n");
    }
    else
        BIND_DONE;

    radio.mode_select = 10; // 1...14
    uint8_t bank = 0;

    DBG_PRINT("Protocol selection switch reads as %d\n", radio.mode_select);	

    for(int8_t i = 0; i < NUM_CHN; i++){
        radio.channel_data[i] = 1024;
    }

    radio.channel_data[THROTTLE] = 0;

    #ifdef ENABLE_PPM
        // Set default PPMs' value
        for(uint8_t i=0; i < NUM_CHN; i++){
            radio.ppm_data[i] = PPM_MAX_100+PPM_MIN_100;
        }
        radio.ppm_data[THROTTLE] = PPM_MIN_100*2;
        radio.chan_order = 0;
    #endif

    modules_reset();
    
    radio.protocol_id_master = random_id(EEPROM_ID_OFFSET, 0);
    DBG_PRINT("Module Id: %lx\n", radio.protocol_id_master);

#ifdef ENABLE_PPM
    if(radio.mode_select != MODE_SERIAL)
    { // PPM

        //uint8_t bank = 0; //bank_switch();

        #ifdef MY_PPM_PROT
			const PPM_Parameters *PPM_prot_line = &My_PPM_prot[ bank * 14 + radio.mode_select -1];
		#else
			const PPM_Parameters *PPM_prot_line = &PPM_prot[bank * 14 + radio.mode_select - 1];
		#endif

        radio.protocol          = PPM_prot_line->protocol;
        radio.cur_protocol[1]   = radio.protocol;
        radio.sub_protocol      = PPM_prot_line->sub_proto;
        radio.rx_num            = PPM_prot_line->rx_num;
        radio.chan_order        = PPM_prot_line->chan_order;

        radio.option = (uint8_t)PPM_prot_line->option;	// Use radio-defined option value
        
        radio.prev_power = 0xFD; // unused power value
		
        if(PPM_prot_line->power){
            POWER_FLAG_on;
        }

        if(PPM_prot_line->autobind){
			AUTOBIND_FLAG_on;
			BIND_IN_PROGRESS;	// Force a bind at protocol startup
		}       

        protocol_init();
    }
#endif    
    DBG_PRINT("Laser4+ version: %d.%d.%d\n", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
}

void multiprotocol_loop(void){
uint16_t next_callback, diff;
uint8_t count=0;
            
    while(radio.remote_callback == NULL || IS_WAIT_BIND_on || IS_INPUT_SIGNAL_off){		
        if(!Update_All())
        {
            cli();								// Disable global int due to RW of 16 bits registers
            #ifndef STM32_BOARD	
            OCR1A=TCNT1;						// Callback should already have been called... Use "now" as new sync point.
            #else
            TIMER_BASE->CCR1 = TIMER_BASE->CNT;
            #endif
            sei();								// Enable global int
        }
        return;
    }
    TX_MAIN_PAUSE_on;
    //tx_pause();
    next_callback = radio.remote_callback() << 1;
    TX_MAIN_PAUSE_off;
    //tx_resume();
    cli();										// Disable global int due to RW of 16 bits registers
    #ifndef STM32_BOARD			
        TIFR1=OCF1A_bm;							// Clear compare A=callback flag
    #else
        TIMER_BASE->CCR1 += next_callback;			// Calc when next_callback should happen
        TIMER_BASE->SR = 0x1E5F & ~TIM_SR_CC1IF;		// Clear Timer2/Comp1 interrupt flag
        diff = TIMER_BASE->CCR1 - TIMER_BASE->CNT;			// Calc the time difference
    #endif		
    sei();										// Enable global int
    if((diff&0x8000) && !(next_callback&0x8000))
    { // Negative result=callback should already have been called... 
        DBG_PRINT("Short CB:%d\n", next_callback);
    }
    else
    {
        if(IS_RX_FLAG_on || IS_PPM_FLAG_on)
        { // Serial or PPM is waiting...
            if(++count>10)
            { //The protocol does not leave enough time for an update so forcing it
                count=0;
                DBG_PRINT("Force update\n");
                Update_All();
            }
        }
        #ifndef STM32_BOARD
            while((TIFR1 & OCF1A_bm) == 0)
        #else
            while((TIMER_BASE->SR & TIM_SR_CC1IF )==0)
        #endif
        {
            if(diff > (900*2))
            {	//If at least 1ms is available update values 
                if((diff&0x8000) && !(next_callback&0x8000))
                {//Should never get here...
                    DBG_PRINT("!!!BUG!!!\n");
                    break;
                }
                count=0;
                Update_All();
                #ifdef DEBUG_SERIAL
                    if(TIMER_BASE->SR & TIM_SR_CC1IF )
                        DBG_PRINT("Long update\n");
                #endif
                if(radio.remote_callback == NULL)
                    break;
                cli();							// Disable global int due to RW of 16 bits registers
                #ifndef STM32_BOARD
                diff = OCR1A-TCNT1;				// Calc the time difference
                #else
                diff = TIMER_BASE->CCR1 - TIMER_BASE->CNT;
                #endif
                sei();							// Enable global int
            }
        }
    }
}

static uint8_t Update_All(void){

    #ifdef ENABLE_SERIAL
        if(radio.mode_select == MODE_SERIAL && IS_RX_FLAG_on)		// Serial mode and something has been received
        {
            update_serial_data();							// Update protocol and data
            update_channels_aux();
            INPUT_SIGNAL_on;								//valid signal received
            radio.last_signal = millis();
        }
    #endif //ENABLE_SERIAL

    #ifdef ENABLE_PPM
        if(radio.mode_select != MODE_SERIAL && IS_PPM_FLAG_on)		// PPM mode and a full frame has been received
        {
            uint32_t chan_or = radio.chan_order;
            uint8_t ch;
            for(uint8_t i=0; i < radio.ppm_chan_max; i++)
            { // update servo data without interrupts to prevent bad read
                uint16_t val;
                cli();										// disable global int
                val = radio.ppm_data[i];
                sei();										// enable global int
                val = map16b(val,PPM_MIN_100*2,PPM_MAX_100*2,CHANNEL_MIN_100,CHANNEL_MAX_100);
                
                if(val&0x8000){
                    val = CHANNEL_MIN_125;
                }else if(val > CHANNEL_MAX_125){
                    val = CHANNEL_MAX_125;
                }

                if(chan_or)
                {
                    ch = chan_or>>28;
                    if(ch)
                        radio.channel_data[ch-1]=val;
                    else
                        radio.channel_data[i]=val;
                    chan_or<<=4;
                }
                else
                    radio.channel_data[i]=val;
            }
            PPM_FLAG_off;									// wait for next frame before update
            #ifdef FAILSAFE_ENABLE
                PPM_failsafe();
            #endif
            update_channels_aux();
            INPUT_SIGNAL_on;								// valid signal received
            radio.last_signal = millis();
        }
    #endif //ENABLE_PPM
    update_led_status();
    
    if(IS_CHANGE_PROTOCOL_FLAG_on)
    { // Protocol needs to be changed or relaunched for bind
        protocol_init();									//init new protocol
        return 1;
    }
    return 0;
}

static void update_led_status(void)
{
    if(IS_INPUT_SIGNAL_on)
        if(millis() - radio.last_signal > 70)
        {
            INPUT_SIGNAL_off;							//no valid signal (PPM or Serial) received for 70ms
            DBG_PRINT("No input signal\n");
        }
    if(radio.blink < millis())
    {
        if(IS_INPUT_SIGNAL_off)
        {
            if(radio.mode_select == MODE_SERIAL)
                radio.blink += BLINK_SERIAL_TIME;				//blink slowly if no valid serial input
            else
                radio.blink += BLINK_PPM_TIME;					//blink more slowly if no valid PPM input
        }
        else
            if(radio.remote_callback == NULL)
            { // Invalid protocol
                if(IS_LED_on)							//flash to indicate invalid protocol
                    radio.blink += BLINK_BAD_PROTO_TIME_LOW;
                else
                    radio.blink += BLINK_BAD_PROTO_TIME_HIGH;
            }
            else
            {
                if(IS_WAIT_BIND_on)
                {
                    if(IS_LED_on)							//flash to indicate WAIT_BIND
                        radio.blink += BLINK_WAIT_BIND_TIME_LOW;
                    else
                        radio.blink += BLINK_WAIT_BIND_TIME_HIGH;
                }
                else
                {
                    if(IS_BIND_DONE)
                        LED_off;							//bind completed force led on
                    radio.blink += BLINK_BIND_TIME;					//blink fastly during binding
                }
            }
        LED_toggle;
    }
}

// Convert 32b id to rx_tx_addr
static void set_rx_tx_addr(uint32_t id)
{ // Used by almost all protocols
    radio.rx_tx_addr[0] = (id >> 24) & 0xFF;
    radio.rx_tx_addr[1] = (id >> 16) & 0xFF;
    radio.rx_tx_addr[2] = (id >>  8) & 0xFF;
    radio.rx_tx_addr[3] = (id >>  0) & 0xFF;
    radio.rx_tx_addr[4] = (radio.rx_tx_addr[2]&0xF0)|(radio.rx_tx_addr[3]&0x0F);
}


static void protocol_init(void){
static uint16_t next_callback;

    if(IS_WAIT_BIND_off)
    {
        radio.remote_callback = NULL;	// No protocol
        next_callback = 0;				// Default is immediate call back
        LED_off;						// Led off during protocol init
        modules_reset();				// Reset all modules

        //Set global ID and rx_tx_addr
        radio.protocol_id = radio.rx_num + radio.protocol_id_master;
        set_rx_tx_addr(radio.protocol_id);
        
        #ifdef FAILSAFE_ENABLE
            FAILSAFE_VALUES_off;
        #endif
        DATA_BUFFER_LOW_off;
        
        radio.blink = millis();

        switch(radio.protocol)				// Init the requested protocol
        {			
            #ifdef CC2500_INSTALLED
                #if defined(FRSKYD_CC2500_INO)
                    case PROTO_FRSKYD:						
                        next_callback = initFrSky_2way();
                        radio.remote_callback = ReadFrSky_2way;
                        break;
                #endif
            #endif				
        }
        DBG_PRINT("Protocol selected: %d, sub proto %d, rxnum %d, option %d\n", radio.protocol, radio.sub_protocol, radio.rx_num, radio.option);
    }
    
    #if defined(WAIT_FOR_BIND) && defined(ENABLE_BIND_CH)
        if( IS_AUTOBIND_FLAG_on && IS_BIND_CH_PREV_off && (cur_protocol[1]&0x80)==0 && mode_select == MODE_SERIAL)
        { // Autobind is active but no bind requested by either BIND_CH or BIND. But do not wait if in PPM mode...
            WAIT_BIND_on;
            return;
        }
    #endif

    WAIT_BIND_off;
    CHANGE_PROTOCOL_FLAG_off;

    if(next_callback > 32000)
    { // next_callback should not be more than 32767 so we will wait here...
        uint16_t temp=(next_callback >> 10) - 2;
        delayMilliseconds(temp);
        next_callback -= temp << 10;            // between 2-3ms left at this stage
    }
    cli();											// disable global int
    TIMER_BASE->CCR1 = TIMER_BASE->CNT + next_callback * 2;		// set compare A for callback
    TIMER_BASE->SR = 0x1E5F & ~TIM_SR_CC1IF;				// Clear Timer2/Comp1 interrupt flag
    sei();										    // enable global int
    BIND_BUTTON_FLAG_off;
}

static void modules_reset(void){
    HW_CC2500_MODULE_RESET;
}

static void update_channels_aux(void){
    //Calc AUX flags
    radio.channel_aux=0;
    for(uint8_t i=0;i<8;i++)
        if(radio.channel_data[CH5+i] > CHANNEL_SWITCH)
            radio.channel_aux |= 1<<i;
}

static uint32_t random_id(uint16_t address, uint8_t create_new)
{
    #ifndef FORCE_GLOBAL_ID
        uint32_t id = 0;
        uint8_t tmp;

        EEPROM_Read((address + 10), &tmp, 1);
        if(tmp == 0xf0 && !create_new)
        //if(eeprom_read_byte((EE_ADDR)(address + 10)) == 0xf0 && !create_new)
        {  // TXID exists in EEPROM
            //for(uint8_t i=4;i>0;i--){
            //	id<<=8;
            //	id|=eeprom_read_byte((EE_ADDR)address + i-1);
            //}
            EEPROM_Read(address, (uint8_t*)&id, 4);

            if(id!=0x2AD141A7)	//ID with seed=0
            {
                DBG_PRINT("Read ID from EEPROM\n");
                return id;
            }
        }
        // Generate a random ID
        #if defined STM32_BOARD
            #define STM32_UUID ((uint32_t *)0x1FFFF7E8)
            if (!create_new)
            {
                id = STM32_UUID[0] ^ STM32_UUID[1] ^ STM32_UUID[2];
                DBG_PRINT("Generated ID from STM32 UUID\n");
            }
            else
        #endif
            {
                //TODO: 
                id ^= 1; //random(0xfefefefe) + ((uint32_t)random(0xfefefefe) << 16);
            }
        //for(uint8_t i=0;i<4;i++){
        //	eeprom_write_byte((EE_ADDR)(address+i), id >> (i*8));
        //}
        EEPROM_Write(address, (uint8_t*)&id, 4);

        //eeprom_write_byte((EE_ADDR)(address+10),0xf0);//write bind flag in eeprom.
        tmp = 0xf0;
        EEPROM_Write((address+10), &tmp, 1);
        return id;
    #else
        (void)address;
        (void)create_new;
        return FORCE_GLOBAL_ID;
    #endif
}

int16_t map16b( int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max)
{
//  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    long y ;
    x -= in_min ;
    y = out_max - out_min ;
    y *= x ;
    x = y / (in_max - in_min) ;
    return x  + out_min ;
}

#ifdef ENABLE_PPM
void PPM_decode(){	// Interrupt on PPM pin
    static int8_t chan = 0, bad_frame = 1;
    static uint16_t Prev_TCNT1 = 0;
    uint16_t Cur_TCNT1;

    Cur_TCNT1 = TIMER_BASE->CNT - Prev_TCNT1 ;	// Capture current Timer1 value
    if(Cur_TCNT1 < 1600)
        bad_frame = 1;					// bad frame
    else
        if(Cur_TCNT1 > 4400)
        {  //start of frame
            if(chan >= MIN_PPM_CHANNELS)
            {
                PPM_FLAG_on;		// good frame received if at least 4 channels have been seen
                if(chan > radio.ppm_chan_max) 
                    radio.ppm_chan_max = chan;	// Saving the number of channels received
            }
            chan = 0;								// reset channel counter
            bad_frame = 0;
        }
        else
            if(bad_frame == 0)			// need to wait for start of frame
            {  //servo values between 800us and 2200us will end up here
                radio.ppm_data[chan] = Cur_TCNT1;
                if(chan++ >= MAX_PPM_CHANNELS)
                    bad_frame = 1;		// don't accept any new channels
            }
    Prev_TCNT1 += Cur_TCNT1;
}

void multiprotocol_mock_ppm(void){
    radio.ppm_chan_max = 4;
    for (uint32_t i = 0; i < radio.ppm_chan_max; i++){
        radio.ppm_data[i] = 1500;
    }
    PPM_FLAG_on;    
}
#endif //ENABLE_PPM