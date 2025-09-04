/*
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

#ifndef _MULTIPROTOCOL_H_
#define _MULTIPROTOCOL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

//******************
// Version
//******************
#define VERSION_MAJOR               1
#define VERSION_MINOR               1
#define VERSION_PATCH               1

//******************
// Constants
//******************
#define BAUD                        100000
#define RXBUFFER_SIZE               36    // 26 + 1 + 9
#define MAX_CHN_NUM                 16
#define TELEMETRY_BUFFER_SIZE       30
#define PPM_TX_CALLBACK_INTERVAL    20000 /* us */
#define PROTOCOL_DEFAULT_INTERVAL   5000
#define US_TO_TICKS(_US)            ((_US) << 1)  /* One us takes two ticks */
#define TICKS_TO_US(_TICKS)         ((_TICKS) >> 1)
#define TICKS_1MS                   US_TO_TICKS(1000)

//********************
//*** Blink timing ***
//********************
#define BLINK_SERIAL_TIME           500
#define BLINK_PPM_TIME              1000
#define BLINK_BAD_PROTO_TIME_LOW    1000
#define BLINK_BAD_PROTO_TIME_HIGH   50
#define BLINK_WAIT_BIND_TIME_LOW    100
#define BLINK_WAIT_BIND_TIME_HIGH   1000
#define BLINK_BIND_TIME             100


/* *************System flags ******************* */
#define _FLAGS_                     radio.flags
#define FLAG_RX                     (1 << 0)    // When set indicates that a serial frame has been received
#define FLAG_PPM                    (1 << 1)    // When set idicates that a ppm frame has been received
#define FLAG_CHANGE_PROTOCOL        (1 << 2)    // When set indicates system to initialyze protocol
#define FLAG_POWER                  (1 << 3)    // Together with CC2500_ENABLE_LOW_POWER, when cleared put CC2500 in low power
#define FLAG_RANGE                  (1 << 4)    // Not used
#define FLAG_AUTOBIND               (1 << 5)    // Used to wait for bind at startup
#define FLAG_BIND_BUTTON            (1 << 6)    // When set forces current protocol initialization
#define FLAG_BIND                   (1 << 7)    // When set indicates system to start bind process
#define FLAG_TX_PAUSE               (1 << 11)
#define FLAG_INPUT_SIGNAL           (1 << 13)   // When set indicates that a valid input siganl is present
#define FLAG_WAIT_BIND              (1 << 15)   // When set indicates system that shoul wait for bind process to conclude
//
#define RX_FLAG_off                  _FLAGS_ &= ~FLAG_RX
#define RX_FLAG_on                   _FLAGS_ |= FLAG_RX
#define IS_RX_FLAG_on                ((_FLAGS_ & FLAG_RX) != 0)
//
#define CHANGE_PROTOCOL_FLAG_off     _FLAGS_ &= ~FLAG_CHANGE_PROTOCOL
#define CHANGE_PROTOCOL_FLAG_on      _FLAGS_ |= FLAG_CHANGE_PROTOCOL
#define IS_CHANGE_PROTOCOL_FLAG_on   ((_FLAGS_ & FLAG_CHANGE_PROTOCOL) != 0)
//
#define POWER_FLAG_off               _FLAGS_ &= ~FLAG_POWER
#define POWER_FLAG_on                _FLAGS_ |= FLAG_POWER
#define IS_POWER_FLAG_on             ((_FLAGS_ & FLAG_POWER) != 0)
//
#define RANGE_FLAG_off               _FLAGS_ &= ~FLAG_RANGE
#define RANGE_FLAG_on                _FLAGS_ |= FLAG_RANGE
#define IS_RANGE_FLAG_on             ((_FLAGS_ & FLAG_RANGE) != 0)
//
#define AUTOBIND_FLAG_off            _FLAGS_ &= ~FLAG_AUTOBIND
#define AUTOBIND_FLAG_on             _FLAGS_ |= FLAG_AUTOBIND
#define IS_AUTOBIND_FLAG_on          ((_FLAGS_ & FLAG_AUTOBIND) != 0)
//
#define BIND_BUTTON_FLAG_off         _FLAGS_ &= ~FLAG_BIND_BUTTON
#define BIND_BUTTON_FLAG_on          _FLAGS_ |= FLAG_BIND_BUTTON
#define IS_BIND_BUTTON_FLAG_on       ((_FLAGS_ &FLAG_BIND_BUTTON) != 0)
//PPM RX OK
#define PPM_FLAG_off                 _FLAGS_ &= ~FLAG_PPM
#define PPM_FLAG_on                  _FLAGS_ |= FLAG_PPM
#define IS_PPM_FLAG_on               ((_FLAGS_ & FLAG_PPM) != 0)

//Bind flag
#define BIND_IN_PROGRESS             _FLAGS_ &= ~FLAG_BIND
#define BIND_DONE                    _FLAGS_ |= FLAG_BIND
#define IS_BIND_DONE                 ((_FLAGS_ & FLAG_BIND) != 0)
#define IS_BIND_IN_PROGRESS          ((_FLAGS_ & FLAG_BIND) == 0)
/* ****** flags2 ***** */
// _BV(0)
// _BV(1)
#define RX_DONOTUPDATE_off           _FLAGS_ &= ~(1<<1)
#define RX_DONOTUPDATE_on            _FLAGS_ |= (1<<1)
#define IS_RX_DONOTUPDATE_on         ((_FLAGS_ & (1<<1) ) != 0)
// _BV(2)
#define RX_MISSED_BUFF_off           _FLAGS_ &= ~(1<<2)
#define RX_MISSED_BUFF_on            _FLAGS_ |= (1<<2)
#define IS_RX_MISSED_BUFF_on         ((_FLAGS_ & (1<<2)) != 0)
// _BV(3)
#define TX_MAIN_PAUSE_off            _FLAGS_ &= ~(1<<11)
#define TX_MAIN_PAUSE_on             _FLAGS_ |= (1<<11)
#define IS_TX_MAIN_PAUSE_on          ((_FLAGS_ & (1<<11) ) != 0)
// _BV(4)

// _BV(5) Signal ok
#define INPUT_SIGNAL_off             _FLAGS_ &= ~FLAG_INPUT_SIGNAL
#define INPUT_SIGNAL_on              _FLAGS_ |= FLAG_INPUT_SIGNAL
#define IS_INPUT_SIGNAL_on           ((_FLAGS_ & FLAG_INPUT_SIGNAL) != 0)
#define IS_INPUT_SIGNAL_off          ((_FLAGS_ & FLAG_INPUT_SIGNAL) == 0)
// _BV(6)

// _BV(7)
#define WAIT_BIND_off                _FLAGS_ &= ~FLAG_WAIT_BIND
#define WAIT_BIND_on                 _FLAGS_ |= FLAG_WAIT_BIND
#define IS_WAIT_BIND_on              ((_FLAGS_ & FLAG_WAIT_BIND) != 0)
#define IS_WAIT_BIND_off             ((_FLAGS_ & FLAG_WAIT_BIND) == 0)
/* ****** flags3 ***** */
// _BV(0)
#define DATA_BUFFER_LOW_off            _FLAGS_ &= ~(1<<16)
// _BV(1)
// _BV(2)
#define DISABLE_CH_MAP_off           _FLAGS_ &= ~(1<<2)
#define DISABLE_CH_MAP_on            _FLAGS_ |= (1<<2)
#define IS_DISABLE_CH_MAP_on         ((_FLAGS_ & (1<<2)) != 0)
#define IS_DISABLE_CH_MAP_off        ((_FLAGS_ & (1<<2)) == 0)
// _BV(3)
#define DISABLE_TELEM_off            _FLAGS_ &= ~(1<<3)
#define DISABLE_TELEM_on             _FLAGS_ |= (1<<3)
#define IS_DISABLE_TELEM_on          ((_FLAGS_ & (1<<3)) != 0)
#define IS_DISABLE_TELEM_off         ((_FLAGS_ & (1<<3)) == 0)
// _BV(4)
// _BV(5)
// _BV(6)
// _BV(7)


//Channel definitions
#define    CH1        0
#define    CH2        1
#define    CH3        2
#define    CH4        3
#define    CH5        4
#define    CH6        5
#define    CH7        6
#define    CH8        7
#define    CH9        8
#define    CH10       9
#define    CH11       10
#define    CH12       11
#define    CH13       12
#define    CH14       13
#define    CH15       14
#define    CH16       15

#define    AILERON    0
#define    ELEVATOR   1
#define    THROTTLE   2
#define    RUDDER     3


#define DEFAULT_ID      0x2AD141A7
#define MODE_SERIAL     0


#define NONE            0
#define P_HIGH          1
#define P_LOW           0
#define AUTOBIND        1
#define NO_AUTOBIND     0


struct meep{
    uint8_t bind;               // Not in use
    uint8_t buz_vol;            // Buzzer volume
    uint8_t rfu1;               // reserved for future use
    uint8_t rfu2;               // reserved for future use
    uint32_t uid;               // cpu unique identifier
    uint32_t vdiv;              // Battery voltage divider racio
    uint32_t rsense;            // Battery sence resistor in ohms
    uint16_t servo_max_100;     // Maximum servo value in us
    uint16_t servo_min_100;     // Minimum servo value in us
    uint16_t servo_max_125;     // 125% maximum servo value in us
    uint16_t servo_min_125;     // 125% minimum servo value in us
    uint16_t switch_on;         // Servo value in us for active switch
    uint16_t switch_off;        // Servo value in us for inactive switch
    uint16_t ppm_max_100;       // Maximum ppm pulse period in timer tick units
    uint16_t ppm_min_100;       // Minimum ppm pulse period in timer tick units
    uint8_t cksum;
} __attribute__((packed));

typedef struct meep meep_t;

enum protocols_e{
    PROTO_PROTOLIST	= 0,    // NO RF
    PROTO_FLYSKY    = 1,    // =>A7105
    PROTO_KN        = 9,    // =>NRF24L01
    PROTO_FRSKYD    = 3,
    PROTO_DEVO      = 7,    // =>CYRF6936
    PROTO_FRSKYX    = 15,
    PROTO_SFHSS     = 21,   // =>CC2500
    PROTO_FRSKYV    = 25,
    PROTO_AFHDS2A   = 28,
    PROTO_WK2x01    = 30,   // =>CYRF6936
    PROTO_FRSKY_RX	= 55,	// =>CC2500
    PROTO_TEST      = 127,
    PROTO_PPM       = 128,
    PROTO_USBHID    = 129
};

enum KN {
        WLTOYS    = 0,
        FEILUN    = 1,
};

enum Flysky {
        Flysky    = 0,
        V9X9    = 1,
        V6X6    = 2,
        V912    = 3,
        CX20    = 4,
};

enum FRSKY_RX
{
	FRSKY_RX	= 0,
	FRSKY_CLONE	= 1,
	FRSKY_ERASE	= 2,
	FRSKY_CPPM  = 3,
};

enum AFHDS2A {
        PWM_IBUS = 0,
        PPM_IBUS = 1,
        PWM_SBUS = 2,
        PPM_SBUS = 3,
};

enum Hubsan {
        H107    = 0,
        H301    = 1,
        H501    = 2,
};

enum FRSKYX {
        CH_16    = 0,
        CH_8    = 1,
        EU_16    = 2,
        EU_8    = 3,
};

enum WK2x01 {
        WK2801    = 0,
        WK2401    = 1,
        W6_5_1    = 2,
        W6_6_1    = 3,
        W6_HEL    = 4,
        W6_HEL_I= 5,
};

struct PPM_Parameters
{
        uint8_t protocol;
        uint8_t sub_proto    : 3;
        uint8_t rx_num        : 6;
        uint8_t power        : 1;
        uint8_t autobind    : 1;
        int8_t option;
        uint32_t chan_order;
};

#pragma pack (1)
typedef struct radio{
    volatile uint32_t flags;
    uint8_t mode_select;

    // Servo data
    uint16_t channel_data[MAX_CHN_NUM];
    uint8_t  nchannels;

    // Protocol variables
    uint8_t rx_tx_addr[5];
    uint8_t sub_protocol;
    uint8_t protocol;
    uint8_t option;
    uint8_t cur_protocol[3];
    uint8_t rx_num;         // RF index number
    uint32_t protocol_id;
    uint32_t protocol_id_master;

#ifdef HOTT_CC2500_INO
    uint8_t  hopping_frequency[75];
#else
    uint8_t  hopping_frequency[50];
#endif
    // RF variables
    uint16_t state;         // RF State
    uint8_t len;            // RF packet length
    uint8_t packet_count;   // RF number of packets
    uint8_t packet[50];
    uint8_t packet_in[TELEMETRY_BUFFER_SIZE];  // Received packets buffer
    uint8_t prev_option;
    uint8_t prev_power;
    uint16_t counter;       // Packet counter

#ifdef ENABLE_SERIAL
    // Serial RX
    volatile uint8_t rx_buff[RXBUFFER_SIZE];
    volatile uint8_t rx_ok_buff[RXBUFFER_SIZE];
    volatile uint8_t discard_frame;
    volatile uint8_t rx_idx;
    volatile uint8_t rx_len;
#endif

#ifdef ENABLE_PPM
    // PPM variable
    const uint16_t *ppm_data;
    uint8_t chan_order;
#endif

#if defined(TELEMETRY)
    //Telemetry
#endif

#ifdef ENABLE_FAILSAFE
        uint16_t Failsafe_data[NUM_CHN];
#endif

    //general variables

    uint32_t last_signal;
    uint32_t blink;

    //callback
    uint16_t (*remote_callback)(struct radio *radio);
}radio_t;

//extern radio_t radio;
extern uint8_t CH_AETR[];
extern uint8_t CH_TAER[];
extern uint8_t CH_EATR[];
extern meep_t *eeprom;

void multiprotocol_setup(void);
void multiprotocol_loop(void);
uint32_t multiprotocol_flags_get(void);
void multiprotocol_flags_set(uint32_t flags);
void multiprotocol_flags_clr(uint32_t flags);
uint32_t multiprotocol_protocol_id_get(void);
void multiprotocol_channel_data_ready(void);
void multiprotocol_channel_data_get(const uint16_t **buf, uint8_t *nchannels);
void multiprotocol_mode_set(uint8_t mode);

int16_t map16b(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max);

void ppm_init(uint16_t *ppmdata, void(*cb)(void));
uint16_t ppm_tx(struct radio *radio);
uint8_t ppm_nchannel_get(void);
void ppm_sim_handler(void);

#ifdef __cplusplus
}
#endif

#endif /* _MULTIPROTOCOL_H_ */