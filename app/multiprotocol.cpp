
#include "app.h"
#include "radio.h"
#include "FrSkyDVX_Common.h"

#ifdef ENABLE_CONSOLE 
static Console con;
#endif

#define TIMER TIM1

radio_t radio;

void (*device_process)(void);

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

void setup(void){
    BOARD_Init();
    //MCO_EN;
    #ifdef ENABLE_VCOM
    CDC_Init();
    #endif

    #ifdef ENABLE_GAME_CONTROLLER
    HID_Init();
    CONTROLLER_Init();
    #endif

    #ifdef ENABLE_CONSOLE
    con.init(IO_CHAR, "laser4+ >");
    con.print("\b\b\b\b\b\b\b\b\b\b");
    con.registerCommandList(laser4_commands);
    #endif

    NV_Init();

    // Read status of bind button
    if(IS_HW_BIND_BUTTON_PRESSED)
    {
        BIND_BUTTON_FLAG_on(radio.flags);	// If bind button pressed save the status
        BIND_IN_PROGRESS(radio.flags);		// Request bind
        DBG_PRINT("Bind button pressed\n");
    }
    else
        BIND_DONE(radio.flags);

    radio.mode_select = 10; // 1...14
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

        radio.protocol = PROTO_FRSKYD;
        radio.cur_protocol[0] = radio.protocol;
        radio.sub_protocol = 0; //NONE;
        radio.rx_num = 0;
        radio.option = 40;
        radio.chan_order = 0;

        POWER_FLAG_on(radio.flags);
        radio.prev_power = 0xFD; // unused power value

        protocol_init();
    }
#endif    
    DBG_PRINT("Laser4+ version: %d.%d.%d\n", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
}

void loop(void){
uint16_t next_callback, diff;
uint8_t count=0;
    while(1)
    {        
        while(radio.remote_callback == NULL || IS_WAIT_BIND_on(radio.flags) || IS_INPUT_SIGNAL_off(radio.flags)){		
            if(!Update_All())
            {
                cli();								// Disable global int due to RW of 16 bits registers
                #ifndef STM32_BOARD	
                OCR1A=TCNT1;						// Callback should already have been called... Use "now" as new sync point.
                #else
                TIMER->CCR1 = TIMER->CNT;
                #endif
                sei();								// Enable global int
            }
        }
        TX_MAIN_PAUSE_on(radio.flags);
        //tx_pause();
        next_callback = radio.remote_callback(&radio) << 1;
        TX_MAIN_PAUSE_off(radio.flags);
        //tx_resume();
        cli();										// Disable global int due to RW of 16 bits registers
        #ifndef STM32_BOARD			
            TIFR1=OCF1A_bm;							// Clear compare A=callback flag
        #else
            TIMER->CCR1 += next_callback;			// Calc when next_callback should happen
            TIMER->SR = 0x1E5F & ~TIM_SR_CC1IF;		// Clear Timer2/Comp1 interrupt flag
            diff = TIMER->CCR1 - TIMER->CNT;			// Calc the time difference
        #endif		
        sei();										// Enable global int
        if((diff&0x8000) && !(next_callback&0x8000))
        { // Negative result=callback should already have been called... 
            DBG_PRINT("Short CB:%d\n", next_callback);
        }
        else
        {
            if(IS_RX_FLAG_on(radio.flags) || IS_PPM_FLAG_on(radio.flags))
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
                while((TIMER->SR & TIM_SR_CC1IF )==0)
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
                        if(TIMER->SR & TIM_SR_CC1IF )
                            DBG_PRINT("Long update\n");
                    #endif
                    if(radio.remote_callback == NULL)
                        break;
                    cli();							// Disable global int due to RW of 16 bits registers
                    #ifndef STM32_BOARD
                    diff = OCR1A-TCNT1;				// Calc the time difference
                    #else
                    diff = TIMER->CCR1 - TIMER->CNT;
                    #endif
                    sei();							// Enable global int
                }
            }
        }			
    }
}

static uint8_t Update_All(void){

    #ifdef ENABLE_SERIAL
        if(radio.mode_select == MODE_SERIAL && IS_RX_FLAG_on(radio.flags))		// Serial mode and something has been received
        {
            update_serial_data();							// Update protocol and data
            update_channels_aux();
            INPUT_SIGNAL_on(radio.flags);								//valid signal received
            radio.last_signal = millis();
        }
    #endif //ENABLE_SERIAL

    #ifdef ENABLE_PPM
        if(radio.mode_select != MODE_SERIAL && IS_PPM_FLAG_on(radio.flags))		// PPM mode and a full frame has been received
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
            PPM_FLAG_off(radio.flags);									// wait for next frame before update
            #ifdef FAILSAFE_ENABLE
                PPM_failsafe();
            #endif
            update_channels_aux();
            INPUT_SIGNAL_on(radio.flags);								// valid signal received
            radio.last_signal = millis();
        }
    #endif //ENABLE_PPM
    update_led_status();

    con.process();

    if(device_process != NULL){
        device_process();
    }
    
    if(IS_CHANGE_PROTOCOL_FLAG_on(radio.flags))
    { // Protocol needs to be changed or relaunched for bind
        protocol_init();									//init new protocol
        return 1;
    }
    return 0;
}

static void update_led_status(void)
{
    if(IS_INPUT_SIGNAL_on(radio.flags))
        if(millis() - radio.last_signal > 70)
        {
            INPUT_SIGNAL_off(radio.flags);							//no valid signal (PPM or Serial) received for 70ms
            DBG_PRINT("No input signal\n");
        }
    if(radio.blink < millis())
    {
        if(IS_INPUT_SIGNAL_off(radio.flags))
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
                if(IS_WAIT_BIND_on(radio.flags))
                {
                    if(IS_LED_on)							//flash to indicate WAIT_BIND
                        radio.blink += BLINK_WAIT_BIND_TIME_LOW;
                    else
                        radio.blink += BLINK_WAIT_BIND_TIME_HIGH;
                }
                else
                {
                    if(IS_BIND_DONE(radio.flags))
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

    if(IS_WAIT_BIND_off(radio.flags))
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
        DATA_BUFFER_LOW_off(radio.flags);
        
        radio.blink = millis();

        switch(radio.protocol)				// Init the requested protocol
        {			
            #ifdef CC2500_INSTALLED
                #if defined(FRSKYD_CC2500_INO)
                    case PROTO_FRSKYD:						
                        next_callback = initFrSky_2way(&radio);
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

    WAIT_BIND_off(radio.flags);
    CHANGE_PROTOCOL_FLAG_off(radio.flags);

    if(next_callback > 32000)
    { // next_callback should not be more than 32767 so we will wait here...
        uint16_t temp=(next_callback >> 10) - 2;
        delayMilliseconds(temp);
        next_callback -= temp << 10;            // between 2-3ms left at this stage
    }
    cli();											// disable global int
    TIMER->CCR1 = TIMER->CNT + next_callback * 2;		// set compare A for callback
    TIMER->SR = 0x1E5F & ~TIM_SR_CC1IF;				// Clear Timer2/Comp1 interrupt flag
    sei();										    // enable global int
    BIND_BUTTON_FLAG_off(radio.flags);
}

static void modules_reset(void){
    HW_CC2500_MODULE_RESET;
}

#ifdef ENABLE_SERIAL
static void update_serial_data(void){
static bool prev_ch_mapping = false;

    #if defined(TELEMETRY) && defined(INVERT_TELEMETRY_TX)
        #ifdef INVERT_TELEMETRY
            static bool prev_inv_telem=true;
        #else
            static bool prev_inv_telem=false;
        #endif
    #endif

    RX_DONOTUPDATE_on(radio.flags);
    RX_FLAG_off(radio.flags);								//data is being processed

    #ifdef SAMSON	// Extremely dangerous, do not enable this unless you know what you are doing...
        if( rx_ok_buff[0]==0x55 && (rx_ok_buff[1]&0x1F)==PROTO_FRSKYD && rx_ok_buff[2]==0x7F && rx_ok_buff[24]==217 && rx_ok_buff[25]==202 )
        {//proto==FRSKYD+sub==7+rx_num==7+CH15==73%+CH16==73%
            rx_ok_buff[1]=(rx_ok_buff[1]&0xE0) | PROTO_FLYSKY;			// change the protocol to Flysky
            memcpy((void*)(rx_ok_buff+4),(void*)(rx_ok_buff+4+11),11);	// reassign channels 9-16 to 1-8
        }
    #endif
    #ifdef BONI	// Extremely dangerous, do not enable this!!! This is really for a special case...
        if(CH14_SW)
            rx_ok_buff[2]=(rx_ok_buff[2]&0xF0)|((rx_ok_buff[2]+1)&0x0F);
    #endif

    if(radio.rx_ok_buff[1] & 0x20){             //check range
        RANGE_FLAG_on(radio.flags);
    }else{
        RANGE_FLAG_off(radio.flags);
    }

    if(radio.rx_ok_buff[1] & 0x40){             //check autobind
        AUTOBIND_FLAG_on(radio.flags);
    }else{
        AUTOBIND_FLAG_off(radio.flags);
    }

    if(radio.rx_ok_buff[2] & 0x80){             //if rx_ok_buff[2] ==1,power is low ,0-power high
        POWER_FLAG_off(radio.flags);            //power low
    }else{
        POWER_FLAG_on(radio.flags);             //power high
    }

    //Forced frequency tuning values for CC2500 protocols
    #if defined(FORCE_FRSKYD_TUNING) && defined(FRSKYD_CC2500_INO)
        if(protocol==PROTO_FRSKYD) 
            option=FORCE_FRSKYD_TUNING;			// Use config-defined tuning value for FrSkyD
        else
    #endif
    #if defined(FORCE_FRSKYV_TUNING) && defined(FRSKYV_CC2500_INO)
        if(protocol==PROTO_FRSKYV)
            option=FORCE_FRSKYV_TUNING;			// Use config-defined tuning value for FrSkyV
        else
    #endif
    #if defined(FORCE_FRSKYX_TUNING) && defined(FRSKYX_CC2500_INO)
        if(protocol==PROTO_FRSKYX)
            option=FORCE_FRSKYX_TUNING;			// Use config-defined tuning value for FrSkyX
        else
    #endif 
    #if defined(FORCE_SFHSS_TUNING) && defined(SFHSS_CC2500_INO)
        if (protocol==PROTO_SFHSS)
            option=FORCE_SFHSS_TUNING;			// Use config-defined tuning value for SFHSS
        else
    #endif
    #if defined(FORCE_CORONA_TUNING) && defined(CORONA_CC2500_INO)
        if (protocol==PROTO_CORONA)
            option=FORCE_CORONA_TUNING;			// Use config-defined tuning value for CORONA
        else
    #endif
    #if defined(FORCE_REDPINE_TUNING) && defined(REDPINE_CC2500_INO)
        if (protocol==PROTO_REDPINE)
            option=FORCE_REDPINE_TUNING;		// Use config-defined tuning value for REDPINE
        else
    #endif
    #if defined(FORCE_HITEC_TUNING) && defined(HITEC_CC2500_INO)
        if (protocol==PROTO_HITEC)
            option=FORCE_HITEC_TUNING;			// Use config-defined tuning value for HITEC
        else
    #endif
    #if defined(FORCE_HOTT_TUNING) && defined(HOTT_CC2500_INO)
        if (protocol==PROTO_HOTT)
            option=FORCE_HOTT_TUNING;			// Use config-defined tuning value for HOTT
        else
    #endif
    
    radio.option = radio.rx_ok_buff[3];         // Use radio-defined option value

    #ifdef FAILSAFE_ENABLE
        bool failsafe=false;
        if(rx_ok_buff[0]&0x02)
        { // Packet contains failsafe instead of channels
            failsafe=true;
            rx_ok_buff[0]&=0xFD;				// Remove the failsafe flag
            FAILSAFE_VALUES_on;					// Failsafe data has been received
            debugln("Failsafe received");
        }
    #endif

    DISABLE_CH_MAP_off(radio.flags);
    DISABLE_TELEM_off(radio.flags);

    if(radio.rx_len > 26)
    {//Additional flag received at the end
        radio.rx_ok_buff[0] = ( radio.rx_ok_buff[26]&0xF0) | (radio.rx_ok_buff[0]&0x0F);	// Additional protocol numbers and RX_Num available -> store them in rx_ok_buff[0]
        if(radio.rx_ok_buff[26]&0x02)
            DISABLE_TELEM_on(radio.flags);
        if(radio.rx_ok_buff[26]&0x01)
            DISABLE_CH_MAP_on(radio.flags);
        #if defined(TELEMETRY) && defined(INVERT_TELEMETRY_TX)
            if(((rx_ok_buff[26]&0x08)!=0) ^ prev_inv_telem)
            { //value changed
                if(rx_ok_buff[26]&0x08)
                {								// Invert telemetry
                    debugln("Invert telem %d,%d",rx_ok_buff[26]&0x01,prev_inv_telem);
                    #if defined (ORANGE_TX)
                        PORTC.PIN3CTRL |= 0x40 ;
                    #elif defined (STM32_BOARD)
                        TX_INV_on;
                        RX_INV_on;
                    #endif
                }
                else
                {								// Normal telemetry
                    debugln("Normal telem %d,%d",rx_ok_buff[26]&0x01,prev_inv_telem);
                    #if defined (ORANGE_TX)
                        PORTC.PIN3CTRL &= 0xBF ;
                    #elif defined (STM32_BOARD)
                        TX_INV_off;
                        RX_INV_off;
                    #endif
                }
                prev_inv_telem=rx_ok_buff[26]&0x08;
            }
        #endif
    }

    if( (radio.rx_ok_buff[0] != radio.cur_protocol[0]) || ((radio.rx_ok_buff[1]&0x5F) != (radio.cur_protocol[1]&0x5F)) || ( (radio.rx_ok_buff[2]&0x7F) != (radio.cur_protocol[2]&0x7F) ) )
    { // New model has been selected
        CHANGE_PROTOCOL_FLAG_on(radio.flags);				//change protocol
        WAIT_BIND_off(radio.flags);
        if((radio.rx_ok_buff[1]&0x80)!=0 || IS_AUTOBIND_FLAG_on(radio.flags))
            BIND_IN_PROGRESS(radio.flags);					//launch bind right away if in autobind mode or bind is set
        else
            BIND_DONE(radio.flags);
        radio.protocol = radio.rx_ok_buff[1]&0x1F;			//protocol no (0-31)
        if(!(radio.rx_ok_buff[0]&1))
            radio.protocol+=32;						//protocol no (0-63)
        if(radio.rx_len>26)
            radio.protocol |= radio.rx_ok_buff[26]&0xC0;		//protocol no (0-255)
        radio.sub_protocol = (radio.rx_ok_buff[2]>>4)& 0x07;	//subprotocol no (0-7) bits 4-6
        radio.rx_num = radio.rx_ok_buff[2]& 0x0F;				//rx_num no (0-15)
        if(radio.rx_len>26)
            radio.rx_num |= radio.rx_ok_buff[26]&0x30;		//rx_num no (0-63)
    }
    else
        if( ((radio.rx_ok_buff[1]&0x80)!=0) && ((radio.cur_protocol[1]&0x80)==0) )		// Bind flag has been set
        { // Restart protocol with bind
            CHANGE_PROTOCOL_FLAG_on(radio.flags);
            BIND_IN_PROGRESS(radio.flags);
        }
        else
            if( ((radio.rx_ok_buff[1]&0x80)==0) && ((radio.cur_protocol[1]&0x80)!=0) )	// Bind flag has been reset
            { // Request protocol to end bind
                #if defined(FRSKYD_CC2500_INO) || defined(FRSKYX_CC2500_INO) || defined(FRSKYV_CC2500_INO) || defined(AFHDS2A_A7105_INO)
                if(radio.protocol==PROTO_FRSKYD || radio.protocol == PROTO_FRSKYX || radio.protocol == PROTO_FRSKYV || radio.protocol == PROTO_AFHDS2A )
                    BIND_DONE(radio.flags);
                else
                #endif
                if(radio.bind_counter > 2)
                    radio.bind_counter = 2;
            }
            
    //store current protocol values
    for(uint8_t i=0;i<3;i++)
        radio.cur_protocol[i] =  radio.rx_ok_buff[i];

    //disable channel mapping
    //if(!IS_CHMAP_PROTOCOL(radio.flags))						//not a protocol supporting ch map to be disabled
    //	DISABLE_CH_MAP_off(radio.flags);
    if(prev_ch_mapping != IS_DISABLE_CH_MAP_on(radio.flags))
    {
        prev_ch_mapping = IS_DISABLE_CH_MAP_on(radio.flags);
        if(IS_DISABLE_CH_MAP_on(radio.flags))
        {
            for(uint8_t i=0;i<4;i++)
                CH_AETR[i] = CH_TAER[i] = CH_EATR[i] = i;
            DBG_PRINT("DISABLE_CH_MAP_on\n");
        }
        else
        {
            CH_AETR[0]=AILERON;CH_AETR[1]=ELEVATOR;CH_AETR[2]=THROTTLE;CH_AETR[3]=RUDDER;
            CH_TAER[0]=THROTTLE;CH_TAER[1]=AILERON;CH_TAER[2]=ELEVATOR;CH_TAER[3]=RUDDER;
            CH_EATR[0]=ELEVATOR;CH_EATR[1]=AILERON;CH_EATR[2]=THROTTLE;CH_EATR[3]=RUDDER;
            DBG_PRINT("DISABLE_CH_MAP_off\n");
        }
    }
    
    // decode channel/failsafe values
    volatile uint8_t *p = radio.rx_ok_buff + 3;
    uint8_t dec=-3;
    for(uint8_t i=0;i<NUM_CHN;i++)
    {
        dec+=3;
        if(dec>=8)
        {
            dec-=8;
            p++;
        }
        p++;
        uint16_t temp=((*((uint32_t *)p))>>dec)&0x7FF;
        #ifdef FAILSAFE_ENABLE
            if(failsafe)
                Failsafe_data[i]=temp;			//value range 0..2047, 0=no pulse, 2047=hold
            else
        #endif
                radio.channel_data[i] = temp;			//value range 0..2047, 0=-125%, 2047=+125%
    }

    #ifdef HOTT_FW_TELEMETRY
        HoTT_SerialRX=false;
    #endif
    if(radio.rx_len > 27)
    { // Data available for the current protocol
        #ifdef SPORT_SEND
            if(protocol==PROTO_FRSKYX && rx_len==35)
            {//Protocol waiting for 8 bytes
                #define BYTE_STUFF	0x7D
                #define STUFF_MASK	0x20
                //debug("SPort_in: ");
                SportData[SportTail]=0x7E;
                SportTail = (SportTail+1) & (MAX_SPORT_BUFFER-1);
                SportData[SportTail]=rx_ok_buff[27]&0x1F;
                SportTail = (SportTail+1) & (MAX_SPORT_BUFFER-1);
                for(uint8_t i=28;i<28+7;i++)
                {
                    if(rx_ok_buff[i]==BYTE_STUFF)
                    {//stuff
                        SportData[SportTail]=BYTE_STUFF;
                        SportTail = (SportTail+1) & (MAX_SPORT_BUFFER-1);
                        SportData[SportTail]=rx_ok_buff[i]^STUFF_MASK;
                    }
                    else
                        SportData[SportTail]=rx_ok_buff[i];
                    //debug("%02X ",SportData[SportTail]);
                    SportTail = (SportTail+1) & (MAX_SPORT_BUFFER-1);
                }
                uint8_t used = SportTail;
                if ( SportHead > SportTail )
                    used += MAX_SPORT_BUFFER - SportHead ;
                else
                    used -= SportHead ;
                if ( used >= MAX_SPORT_BUFFER-(MAX_SPORT_BUFFER>>2) )
                {
                    DATA_BUFFER_LOW_on;
                    SEND_MULTI_STATUS_on;	//Send Multi Status ASAP to inform the TX
                    debugln("Low buf=%d,h=%d,t=%d",used,SportHead,SportTail);
                }
            }
        #endif //SPORT_SEND
        #ifdef HOTT_FW_TELEMETRY
            if(protocol==PROTO_HOTT && rx_len==28)
            {//Protocol waiting for 1 byte
                HoTT_SerialRX_val=rx_ok_buff[27];
                HoTT_SerialRX=true;
            }
        #endif
    }

    RX_DONOTUPDATE_off(radio.flags);
    #ifdef ORANGE_TX
        cli();
    #else
        UCSR0B &= ~_BV(RXCIE0);					// RX interrupt disable
    #endif
    if(IS_RX_MISSED_BUFF_on(radio.flags))					// If the buffer is still valid
    {	
        if(radio.rx_idx >= 26 && radio.rx_idx<RXBUFFER_SIZE)
        {
            radio.rx_len = radio.rx_idx;
            memcpy((void*)radio.rx_ok_buff,(const void*)radio.rx_buff, radio.rx_len);// Duplicate the buffer
            RX_FLAG_on(radio.flags);							// Data to be processed next time...
        }
        RX_MISSED_BUFF_off(radio.flags);
    }
    #ifdef ORANGE_TX
        sei();
    #else
        UCSR0B |= _BV(RXCIE0) ;					// RX interrupt enable
    #endif
}
#endif /* ENABLE_SERIAL */

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

    Cur_TCNT1 = TIMER->CNT - Prev_TCNT1 ;	// Capture current Timer1 value
    if(Cur_TCNT1 < 1600)
        bad_frame = 1;					// bad frame
    else
        if(Cur_TCNT1 > 4400)
        {  //start of frame
            if(chan >= MIN_PPM_CHANNELS)
            {
                PPM_FLAG_on(radio.flags);		// good frame received if at least 4 channels have been seen
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

#ifdef MOCK_PPM
extern "C" void TIM3_IRQHandler(void){
    TIM3->SR = ~TIM3->SR;    
    radio.ppm_chan_max = 4;
    for (uint32_t i = 0; i < radio.ppm_chan_max; i++)
    {
        radio.ppm_data[i] = 1500;
    }
    PPM_FLAG_on(radio.flags);    
}
#endif
#endif //ENABLE_PPM