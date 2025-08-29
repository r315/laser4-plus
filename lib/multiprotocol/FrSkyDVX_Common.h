
#ifndef _FRSKYDVX_COMMON_H_
#define _FRSKYDVX_COMMON_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "multiprotocol.h"
#include "debug.h"

#ifdef ENABLE_DEBUG_FRSKY
#define DBG_FRSKY_TAG       "FRSKY : "
#define DBG_FRSKY_PRINT DBG_PRINT
#define DBG_FRSKY_INF(...)  DBG_INF(DBG_FRSKY_TAG __VA_ARGS__)
#define DBG_FRSKY_WRN(...)  DBG_WRN(DBG_FRSKY_TAG __VA_ARGS__)
#define DBG_FRSKY_ERR(...)  DBG_ERR(DBG_FRSKY_TAG __VA_ARGS__)
#define DBG_FRSKY_HEXDUMP(...) DBG_HEXDUMP(__VA_ARGS__)
#else
#define DBG_FRSKY_PRINT(...)
#define DBG_FRSKY_INF(...)
#define DBG_FRSKY_WRN(...)
#define DBG_FRSKY_ERR(...)
#define DBG_FRSKY_HEXDUMP(...)
#endif

#ifndef PROGMEM
#define PROGMEM
#define pgm_read_byte_near(_X) (*(const unsigned char *)(_X))
#endif

#if defined(FRSKYD_CC2500_INO) || defined(FRSKYX_CC2500_INO)
enum {
	FRSKY_BIND		= 0,
	FRSKY_BIND_DONE	= 1000,
	FRSKY_DATA1,
	FRSKY_DATA2,
	FRSKY_DATA3,
	FRSKY_DATA4,
	FRSKY_DATA5,
};

#endif

#define CC2500_CALLBACK_INTERVAL            10000 /* us */

extern const PROGMEM uint8_t FRSKYD_cc2500_conf[];

void Frsky_init_hop(radio_t *radio);
void FRSKY_init_cc2500(radio_t *radio, const uint8_t *ptr);
void Frsky_SetPower(radio_t *radio);
uint16_t convert_channel_frsky(uint16_t val);

//FrSkyD
uint16_t FRSKYD_init(radio_t *radio);
uint16_t FRSKYD_callback(radio_t *radio);


#ifdef __cplusplus
}
#endif

#endif /* #ifndef _FRSKYDVX_COMMON_H_ */