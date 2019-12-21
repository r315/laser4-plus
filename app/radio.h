#ifndef _RADIO_H_
#define _RADIO_H_

#include <stdint.h>

#define BAUD                    100000
#define RXBUFFER_SIZE           36	// 26+1+9
#define NUM_CHN                 16
#define TELEMETRY_BUFFER_SIZE   30

#define IS_RANGE_FLAG_on(x)     ((x & (1<<3)) != 0)
#define IS_BIND_DONE(x)		    ((x & (1<<7)) != 0)
#define IS_BIND_IN_PROGRESS(x)  ((x & (1<<7)) == 0)

#define BIND_DONE(x)            x |= (1<<7)

enum state_e{
    STATE_BINDING
};

typedef struct radio{
    uint8_t flags;
    uint8_t cur_protocol[3];
    uint8_t sub_protocol;
    uint8_t prev_power;
    uint8_t rx_num;
    uint8_t mode_select;
    uint8_t option;
    uint8_t prev_option;

    // Servo data 
    uint16_t Channel_data[NUM_CHN];
    uint8_t  Channel_AUX;
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

    //Telemetry
    uint8_t packet_in[TELEMETRY_BUFFER_SIZE];//telemetry receiving packets
#if defined(TELEMETRY)
#endif
    //general variables
    uint16_t counter;
    uint8_t  packet[50];
}radio_t;

#endif /* _RADIO_H_ */