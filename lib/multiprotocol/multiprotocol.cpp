
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

#include "multiprotocol.h"
#include "FrSkyDVX_Common.h"
#include "app.h"
#include "board.h"
#include "debug.h"

//Personal config file
#if defined(USE_MY_CONFIG)
#include "_MyConfig.h"
#endif

#ifdef ENABLE_DEBUG_MULTI
#define DBG_TAG             "MULTI : "
#define DBG_MULTI_PRINT DBG_PRINT
#define DBG_MULTI_INF(...)  DBG_INF(DBG_TAG __VA_ARGS__)
#define DBG_MULTI_WRN(...)  DBG_WRN(DBG_TAG __VA_ARGS__)
#define DBG_MULTI_ERR(...)  DBG_ERR(DBG_TAG __VA_ARGS__)
#else
#define DBG_MULTI_PRINT(...)
#define DBG_MULTI_INF(...)
#define DBG_MULTI_WRN(...)
#define DBG_MULTI_ERR(...)
#endif

static radio_t radio;
uint16_t *eeprom_data;

static uint8_t Update_All(void);
static void modules_reset(void);
//static void update_serial_data(void);
static void protocol_init(void);
static void update_led_status(void);
static int16_t map16b( int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max);
static uint32_t random_id(uint8_t create_new);
static void set_rx_tx_addr(uint8_t *dst, uint32_t id);

//Channel mapping for protocols
uint8_t CH_AETR[]={AILERON, ELEVATOR, THROTTLE, RUDDER, CH5, CH6, CH7, CH8, CH9, CH10, CH11, CH12, CH13, CH14, CH15, CH16};
uint8_t CH_TAER[]={THROTTLE, AILERON, ELEVATOR, RUDDER, CH5, CH6, CH7, CH8, CH9, CH10, CH11, CH12, CH13, CH14, CH15, CH16};
//uint8_t CH_RETA[]={RUDDER, ELEVATOR, THROTTLE, AILERON, CH5, CH6, CH7, CH8, CH9, CH10, CH11, CH12, CH13, CH14, CH15, CH16};
uint8_t CH_EATR[]={ELEVATOR, AILERON, THROTTLE, RUDDER, CH5, CH6, CH7, CH8, CH9, CH10, CH11, CH12, CH13, CH14, CH15, CH16};

/**
 * @brief
 * */
void multiprotocol_setup(void)
{
    DBG_MULTI_PRINT("\n***** Starting Multiprotocol *****\n");
    // Read status of bind button
    if(IS_BIND_BUTTON_PRESSED)
    {
        BIND_BUTTON_FLAG_on;	// If bind button pressed save the status
        BIND_IN_PROGRESS;		// Request bind
        DBG_MULTI_INF("Bind button pressed");
    }
    else
        BIND_DONE;

    DBG_MULTI_INF("Protocol selection index : %d", radio.mode_select);

    uint16_t channel_default = (eeprom_data[IDX_CHANNEL_MAX_100] - eeprom_data[IDX_CHANNEL_MIN_100]) >> 1;

    for(uint8_t i = 0; i < MAX_CHN_NUM; i++){
        radio.channel_data[i] = channel_default;
    }

    radio.channel_data[THROTTLE] = eeprom_data[IDX_CHANNEL_MIN_125] ;

    modules_reset();

    radio.protocol_id_master = random_id(false);
    DBG_MULTI_INF("Module Id: %lx", radio.protocol_id_master);

#ifdef ENABLE_PPM
    // Setup callback for ppm frame ready
    ppm_setCallBack(multiprotocol_setChannelData);

    radio.chan_order = 0;
    uint8_t bank = HW_BANK_SWITCH;

    if(radio.mode_select != MODE_SERIAL)
    { // PPM
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

}
/**
 * @brief main loop for multiprotocol mode
 * */
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
            TIME_BASE->CCR1 = TIME_BASE->CNT;
            #endif
            sei();								// Enable global int
        }
        return;
    }

    next_callback = radio.remote_callback(&radio) << 1;

    cli();										    // Disable global int due to RW of 16 bits registers
    #ifndef STM32_BOARD
    TIFR1=OCF1A_bm;							        // Clear compare A=callback flag
    #else
    TIME_BASE->CCR1 += next_callback;			    // Calc when next_callback should happen
    TIME_BASE->SR = 0x1E5F & ~TIM_SR_CC1IF;	    // Clear Timer2/Comp1 interrupt flag
    diff = TIME_BASE->CCR1 - TIME_BASE->CNT;	    // Calc the time difference
    #endif
    sei();										    // Enable global int

    if((diff&0x8000) && !(next_callback&0x8000))
    { // Negative result=callback should already have been called...
        DBG_MULTI_WRN("Short CB:%d", next_callback);
    }
    else
    {
        if(IS_RX_FLAG_on || IS_PPM_FLAG_on)
        { // Serial or PPM is waiting...
            if(++count > 10)
            { //The protocol does not leave enough time for an update so forcing it
                count = 0;
                DBG_MULTI_WRN("Force update");
                Update_All();
            }
        }
        #ifndef STM32_BOARD
            while((TIFR1 & OCF1A_bm) == 0)
        #else
        while((TIME_BASE->SR & TIM_SR_CC1IF ) == 0)
        #endif
        {
            if(diff > (900*2))
            {	//If at least 1ms is available update values
                if((diff&0x8000) && !(next_callback&0x8000))
                {//Should never get here...
                    DBG_MULTI_WRN("!!!BUG!!!");
                    break;
                }
                count=0;
                Update_All();
                #ifdef ENABLE_DEBUG
                if(TIME_BASE->SR & TIM_SR_CC1IF )
                    DBG_MULTI_WRN("Long update");
                #endif
                if(radio.remote_callback == NULL)
                    break;
                cli();							// Disable global int due to RW of 16 bits registers
                #ifndef STM32_BOARD
                diff = OCR1A-TCNT1;				// Calc the time difference
                #else
                diff = TIME_BASE->CCR1 - TIME_BASE->CNT;
                #endif
                sei();							// Enable global int
            }
        }
    }
}
/**
 * @brief process PPM channel data and aux channels
 * @return : 1 - if protocol change was requested, 0 - otherwise
 * */
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
            for(uint8_t i = 0; i < radio.channel_aux; i++)
            { // update servo data without interrupts to prevent bad read
                uint16_t val;
                cli();										// disable global int
                val = radio.ppm_data[i];
                sei();										// enable global int
                val = map16b(val,
                            eeprom_data[IDX_PPM_MIN_100],
                            eeprom_data[IDX_PPM_MAX_100],
                            eeprom_data[IDX_CHANNEL_MIN_100],
                            eeprom_data[IDX_CHANNEL_MAX_100]);

                if(val & 0x8000){
                    val = eeprom_data[IDX_CHANNEL_MIN_125];
                }else if(val > eeprom_data[IDX_CHANNEL_MAX_125]){
                    val = eeprom_data[IDX_CHANNEL_MAX_125];
                }

                if(chan_or)
                {
                    ch = chan_or >> 28;
                    if(ch)
                        radio.channel_data[ch-1] = val;
                    else
                        radio.channel_data[i] = val;
                    chan_or<<=4;
                }
                else
                    radio.channel_data[i] = val;
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
            DBG_MULTI_WRN("Lost input signal");
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
                        LED_OFF;							//bind completed force led on
                    radio.blink += BLINK_BIND_TIME;					//blink fastly during binding
                }
            }
        LED_TOGGLE;
    }
}

static void protocol_init(void)
{
    static uint16_t next_callback;

    if(IS_WAIT_BIND_off)
    {
        radio.remote_callback = NULL;	// No protocol
        next_callback = 0;				// Default is immediate call back
        LED_OFF;						// Led off during protocol init
        modules_reset();				// Reset all modules

        //Set global ID and rx_tx_addr
        radio.protocol_id = radio.rx_num + radio.protocol_id_master;
        set_rx_tx_addr(radio.rx_tx_addr, radio.protocol_id);

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
                        next_callback = initFrSky_2way(&radio);
                        radio.remote_callback = ReadFrSky_2way;
                        break;
                #endif
            #endif

            #ifdef TX35_MHZ_INSTALLED
                    case PROTO_PPM:
                        next_callback = 10000;
                        radio.remote_callback = ppm_tx;
                        HW_TX_35MHZ_ON;
                        DBG_MULTI_INF("TX 35MHz enabled");
                        break;
            #endif
        }
        DBG_MULTI_INF("Protocol selected: %d, sub proto %d, rxnum %d, option %d", radio.protocol, radio.sub_protocol, radio.rx_num, radio.option);
        if(IS_BIND_IN_PROGRESS){
            DBG_MULTI_INF("Bind started");
        }
    }

    #if defined(WAIT_FOR_BIND) && defined(ENABLE_BIND_CH)
        if( IS_AUTOBIND_FLAG_on && IS_BIND_CH_PREV_off && (cur_protocol[1]&0x80)==0 && mode_select == MODE_SERIAL)
        { // Autobind is active but no bind requested by either BIND_CH or BIND. But do not wait if in PPM mode...
            WAIT_BIND_on;
            return;
        }
    #endif

    // Clear bind request flags
    WAIT_BIND_off;
    CHANGE_PROTOCOL_FLAG_off;
    BIND_BUTTON_FLAG_off;

    if(next_callback > 32000)
    { // next_callback should not be more than 32767 so we will wait here...
        uint16_t temp = (next_callback >> 10) - 2;
        DelayMs(temp);
        next_callback -= temp << 10;                        // between 2-3ms left at this stage
    }
    cli();											        // disable global int
    TIME_BASE->CCR1 = TIME_BASE->CNT + next_callback * 2;	// set compare A for callback
    TIME_BASE->SR = 0x1E5F & ~TIM_SR_CC1IF;				// Clear Timer2/Comp1 interrupt flag
    sei();										            // enable global int
}

/**
 * After ppm synchronization, this function is called every ~20mS
 * TODO: Move auxiliary channels to project application
 * */
void update_channels_aux(void){

#ifdef ENABLE_AUX_CHANNELS
    uint8_t switches = readAuxSwitches();

    for(uint8_t i = 0; i < MAX_AUX_CHANNELS - 1; i++){
        if((switches & (1<<i)) == 0){
            radio.channel_data[radio.channel_aux + i] = eeprom_data[IDX_CHANNEL_MIN_100];
        }else{
            radio.channel_data[radio.channel_aux + i] = eeprom_data[IDX_CHANNEL_SWITCH];
        }
    }
#endif

#ifdef ENABLE_ENCODER
    // Process encoder
    int16_t diff = encGetDiff();
    if(diff != 0){
        uint16_t tmp = radio.channel_data[radio.channel_aux + MAX_AUX_CHANNELS - 1];
        tmp += diff * 10; // speed
        if(tmp > eeprom_data[IDX_CHANNEL_MAX_100]){
            tmp = eeprom_data[IDX_CHANNEL_MAX_100];
        }else if(tmp < eeprom_data[IDX_CHANNEL_MIN_100]){
            tmp = eeprom_data[IDX_CHANNEL_MIN_100];
        }
        radio.channel_data[radio.channel_aux + MAX_AUX_CHANNELS - 1] = tmp;
    }
#endif
}

/**
 *  Private Functions, maybe move them to own file?
 * */

/**
 * @brief Convert 32b id to rx_tx_addr
 * */
static void set_rx_tx_addr(uint8_t *dst, uint32_t id)
{ // Used by almost all protocols
    dst[0] = (id >> 24) & 0xFF;
    dst[1] = (id >> 16) & 0xFF;
    dst[2] = (id >>  8) & 0xFF;
    dst[3] = (id >>  0) & 0xFF;
    dst[4] = (dst[2]&0xF0)|(dst[3]&0x0F);
}

/**
 * @brief Function to set channels data.
 * Callback for when channel data is ready from ppm_decode or
 * simulation
 *
 * @param buf   Buffer with channel data in 0.5us resolution
 * @param chan  Number of channels
 */
void multiprotocol_setChannelData(const uint16_t *buf, uint8_t chan){
    PPM_FLAG_on;
#ifdef ENABLE_PPM
    radio.ppm_data = buf;
    radio.channel_aux = chan;
#else
    (void)buf;
    (void)chan;
#endif
}

/**
 *
 * */
static int16_t map16b( int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max)
{
//  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    long y ;
    x -= in_min ;
    y = out_max - out_min ;
    y *= x ;
    x = y / (in_max - in_min) ;
    return x + out_min ;
}
/**
 *
 * */
static void modules_reset(void)
{
    #ifdef CC2500_INSTALLED
    HW_CC2500_MODULE_RESET;
    #endif

    #ifdef TX35_MHZ_INSTALLED
    HW_TX_35MHZ_OFF;
    #endif
}
/**
 *
 * */
static uint32_t random_id(uint8_t create_new)
{
    uint32_t id = 0;

    id = eeprom_data[EEPROM_ID_OFFSET + 1] << 16 | eeprom_data[EEPROM_ID_OFFSET];

    if(!create_new){
        if(id != DEFAULT_ID){	//ID with seed=0
            DBG_MULTI_INF("Using ID 0x%x from EEPROM", id);
            return id;
        }

        id = cpuGetId();
        DBG_MULTI_INF("Using Chip UUID 0x%x", id);
    }else{
        id = xrand();
    }

    eeprom_data[EEPROM_ID_OFFSET] = (uint16_t)id;
    eeprom_data[EEPROM_ID_OFFSET + 1] = (uint16_t)(id >> 16);
    *((uint8_t*)eeprom_data + EEPROM_BIND_FLAG) = BIND_FLAG_VALUE;

    appSaveEEPROM();
    return id;
}

uint32_t multiprotocol_flags_get(void)
{
    return radio.flags;
}

uint32_t multiprotocol_protocol_id_get(void)
{
    return radio.protocol_id_master;
}

uint16_t *multiprotocol_channel_data_get(void)
{
    return radio.channel_data;
}

void multiprotocol_flags_set(uint32_t flags)
{
    radio.flags |= flags;
}

void multiprotocol_flags_clr(uint32_t flags)
{
    radio.flags &= ~flags;
}

/**
 * @brief Set operation mode.
 * Usually defined at startup by hardware switches,
 * it determines which entry in the My_PPM_prot table to use.
 * mode must be set before calling multiprotocol_setup
 *
 * @param mode My_PPM_prot entry number 0-14
 */
void multiprotocol_set_mode(uint8_t mode)
{
    radio.mode_select = mode;
}