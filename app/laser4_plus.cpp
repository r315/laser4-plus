
#include "app.h"
#include "radio.h"

static Console con;

volatile uint8_t rx_buff[RXBUFFER_SIZE];
volatile uint8_t rx_ok_buff[RXBUFFER_SIZE];
volatile uint8_t discard_frame = 0;  // false
volatile uint8_t rx_idx=0, rx_len=0;

static radio_t radio;

void app_setup(void){
    BOARD_Init();
    //MCO_EN;
    CDC_Init();


    con.init(&vcom, "laser4+ >");
    con.registerCommandList(laser4_commands);

    //init_HWTimer();			//0.5us???

    if(HW_BIND_BUTTON_PRESSED){
        radio.state = STATE_BINDING;
    }

    HW_MODULE_RESET;

    //ramdomSeed(seed);

    // Read or create protocol id
	//MProtocol_id_master=random_id(EEPROM_ID_OFFSET,false);


    //radio.mode_select = MODE_SERIAL;
    radio.prev_power = 0xFD; // unused power value

    for(uint8_t i = 0; i < 3; i++){
        radio.cur_protocol[i] = 0;
    }

    for(int8_t i = 0; i < NUM_CHN; i++){
        radio.Channel_data[i] = 1024;
    }

    //radio.channel_data[THROTTLE] = 0;

    LED_ON;
}

void app_loop(void *ptr){
//static int i;

    DBG_LED_TOGGLE;
    //DBG_PIN_TOGGLE;
    vTaskDelay(10);

    con.process();
}
