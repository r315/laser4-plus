#ifndef _RADIO_H_
#define _RADIO_H_

#include <stdint.h>

#define VERSION_MAJOR               0
#define VERSION_MINOR               0
#define VERSION_PATCH               0

#define BAUD                        100000
#define RXBUFFER_SIZE               36	// 26+1+9
#define NUM_CHN                     16
#define TELEMETRY_BUFFER_SIZE       30

// Led timmings
#define BLINK_SERIAL_TIME           500
#define BLINK_PPM_TIME              1000
#define BLINK_BAD_PROTO_TIME_LOW    1000
#define BLINK_BAD_PROTO_TIME_HIGH   50
#define BLINK_WAIT_BIND_TIME_LOW	100
#define BLINK_WAIT_BIND_TIME_HIGH	1000
#define BLINK_BIND_TIME				100


/* *************System flags ******************* */
//
#define IS_RX_FLAG_on(x)		        ((x & (1<<0)) != 0)
//
#define CHANGE_PROTOCOL_FLAG_off(x)     x &= ~(1<<1)   
#define CHANGE_PROTOCOL_FLAG_on(x)      x |= (1<<1)
#define IS_CHANGE_PROTOCOL_FLAG_on(x)   ((x & (1<<1)) != 0)
//
#define IS_RANGE_FLAG_on(x)             ((x & (1<<3)) != 0)
//
#define BIND_BUTTON_FLAG_off(x)	        x &= ~(1<<5)
#define BIND_BUTTON_FLAG_on(x)	        x |= (1<<5)
#define IS_BIND_BUTTON_FLAG_on(x)	    ((x & (1<<5)) != 0)
//PPM RX OK
#define IS_PPM_FLAG_on(x)		        ((x & (1<<6)) != 0)
//Bind flag
#define BIND_IN_PROGRESS(x)	            x &= ~(1<<7)
#define BIND_DONE(x)                    x |= (1<<7)
#define IS_BIND_DONE(x)		            ((x & (1<<7)) != 0)
#define IS_BIND_IN_PROGRESS(x)          ((x & (1<<7)) == 0)
/* ****** flags2 ***** */
// _BV(0)
// _BV(1)
// _BV(2)
// _BV(3)
#define TX_MAIN_PAUSE_off(x)            x &= ~(1<<11)
#define TX_MAIN_PAUSE_on(x)             x |= (1<<11)
#define IS_TX_MAIN_PAUSE_on(x)	        ((x & (1<<11) ) != 0)
// _BV(4)

// _BV(5) Signal ok
#define INPUT_SIGNAL_off(x)	            x &= ~(1<<13)
#define INPUT_SIGNAL_on(x)		        x |= (1<<13)
#define IS_INPUT_SIGNAL_on(x)           ((x & (1<<13)) != 0)
#define IS_INPUT_SIGNAL_off(x)          ((x & (1<<13)) == 0)
// _BV(6)

// _BV(7)
#define WAIT_BIND_off(x)                x &= ~(1<<15)
#define WAIT_BIND_on(x)                 x |= (1<<15)
#define IS_WAIT_BIND_on(x)              ((x & (1<<15)) != 0)
#define IS_WAIT_BIND_off(x)             ((x & (1<<15)) == 0)
/* ****** flags3 ***** */
// _BV(0)
#define DATA_BUFFER_LOW_off(x)	        x &= ~(1<<16)
// _BV(1)
// _BV(2)
// _BV(3)
// _BV(4)
// _BV(5)
// _BV(6)
// _BV(7)

enum aetr_e{
    AILERON = 0,
    ELEVATOR,
    THROTTLE,
    RUDDER
};

enum state_e{
    STATE_BINDING
};

enum protocols_e{
    MODE_SERIAL     = 0,
    PROTO_FRSKYD    = 3
};

typedef struct radio{
    uint32_t flags;       
    uint8_t mode_select;    

    // Servo data 
    uint16_t channel_data[NUM_CHN];
    uint8_t  channel_AUX;
#ifdef FAILSAFE_ENABLE
	uint16_t Failsafe_data[NUM_CHN];
#endif

    // Protocol variables
    uint8_t  rx_tx_addr[5];
#ifdef HOTT_CC2500_INO
	uint8_t  hopping_frequency[75];
#else
	uint8_t  hopping_frequency[50];
#endif
    uint16_t state;
    uint8_t  len;
    uint8_t  packet_count;

    // Serial
    uint8_t sub_protocol;
    uint8_t protocol;
    uint8_t option;
    uint8_t cur_protocol[3];
    uint8_t prev_option;
    uint8_t prev_power; 
    uint8_t rx_num;
    

    //Telemetry
    uint8_t packet_in[TELEMETRY_BUFFER_SIZE];//telemetry receiving packets
#if defined(TELEMETRY)
#endif
    //general variables
    uint16_t counter;
    uint8_t  packet[50];
    uint32_t last_signal;
    uint32_t blink;
    uint32_t protocol_id;
    uint32_t protocol_id_master;

    //callback
    uint16_t (*remote_callback)(struct radio *);
}radio_t;

#endif /* _RADIO_H_ */